
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time
import numpy as np
from uuid import uuid4
from collections import deque
np.float=float
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import utils


class ObstacleFilter:
    def __init__(self, scan_ranges, max_scan_range, min_scan_range):
        self.scan_ranges = scan_ranges
        self.max_scan_range = max_scan_range
        self.min_scan_range = min_scan_range

    def classify_obstacles(self, scan, robot_pose, yaw, ground_truth_scans):
        scan_range = utils.get_scan_ranges(scan, self.scan_ranges, self.max_scan_range)
        obstacle_poses = utils.convert_laserscan_to_coordinate(scan_range, self.scan_ranges, robot_pose, yaw, 360)

        ground_truth_poses = utils.convert_laserscan_to_coordinate(
            ground_truth_scans, self.scan_ranges, robot_pose, yaw, 360)
        bbox_size = utils.compute_average_bounding_box_size(ground_truth_poses)

        filtered_poses = []
        for i in range(len(obstacle_poses)):
            if not (1.0 * ground_truth_scans[i] <= scan_range[i] <= 1.0 * ground_truth_scans[i]):
                filtered_poses.append(obstacle_poses[i])
            else:
                filtered_poses.append(None)

        valid_indices = [i for i, p in enumerate(filtered_poses) if p is not None]
        gradients = []
        for idx, i in enumerate(valid_indices):
            # Ensure filtered_poses[i] and filtered_poses[...] are not None
            if filtered_poses[i] is None:
                gradients.append(0.0)
                continue
            if idx == len(valid_indices) - 1:
                j = valid_indices[0]
            else:
                j = valid_indices[idx + 1]
            if filtered_poses[j] is None:
                gradients.append(0.0)
                continue
            dx = filtered_poses[i][0] - filtered_poses[j][0]
            dy = filtered_poses[i][1] - filtered_poses[j][1]
            if dy == 0.0:
                gradients.append(0.0)
            else:
                try:
                    grad = dx / dy
                except ZeroDivisionError:
                    grad = 0.0
                gradients.append(grad)

        object_types = []
        for grad in gradients:
            if abs(grad) < 0.05:
                object_types.append("w")
            else:
                object_types.append("o")

        estimated_objects = []
        for i, idx in enumerate(valid_indices):
            if idx < len(scan_range) and obstacle_poses[idx] is not None:
                estimated_objects.append([object_types[i], obstacle_poses[idx], scan_range[idx]])

        return self.segment_and_confirm_objects(estimated_objects, bbox_size)

    def segment_and_confirm_objects(self, objects, bbox_size):
        if not objects:
            return []

        segments = [[objects[0]]]
        for i in range(1, len(objects)):
            # Ensure objects[i][1] and objects[i-1][1] are not None
            if objects[i][1] is None or objects[i-1][1] is None:
                segments.append([objects[i]])
                continue
            if utils.is_associated(objects[i][1], objects[i - 1][1], bbox_size):
                segments[-1].append(objects[i])
            else:
                segments.append([objects[i]])

        confirmed = []
        for seg in segments:
            if not seg:
                continue
            # Only use non-None poses
            types = [s[0] for s in seg if s[1] is not None]
            poses = [s[1] for s in seg if s[1] is not None]
            dists = [s[2] for s in seg if s[1] is not None]
            if not poses or not dists or not types:
                continue
            center = poses[len(poses) // 2]
            score = types.count("o") / len(types)
            label = "o" if score > 0.5 else "w"
            confirmed.append([label, center, dists[len(dists) // 2]])
        return confirmed


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracking_node')

        self.k_obstacle_count = 8
        self.robot_width = 0.178
        self.position = Pose()
        self.orientation = None
        self.linear_twist = None
        self.angular_twist = None
        self.robot_yaw = 0.0
        self.agent_pose_deque = deque(maxlen=2)
        self.tracked_obstacles = {}
        self.tracked_obstacles_keys = []
        self.agent_vel_timestep = 0.1

        self.declare_parameter('scan_ranges', 360)
        self.declare_parameter('max_scan_range', 3.5)
        self.declare_parameter('min_scan_range', 0.12)

        self.scan_ranges = self.get_parameter('scan_ranges').value
        self.max_scan_range = self.get_parameter('max_scan_range').value
        self.min_scan_range = self.get_parameter('min_scan_range').value

        self.ground_truth_scans = [self.max_scan_range] * (self.scan_ranges - 1)
        self.ob_filter = ObstacleFilter(self.scan_ranges, self.max_scan_range, self.min_scan_range)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_twist = msg.twist.twist.linear
        self.angular_twist = msg.twist.twist.angular

    def get_yaw(self):
        from tf_transformations import euler_from_quaternion
        q = self.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = yaw
        return yaw

    def update_agent_pose(self):
        self.agent_pose_deque.append([round(self.position.x, 3), round(self.position.y, 3)])

    def track_obstacles(self, confirmed_obstacles):
        tracked_copy = self.tracked_obstacles.copy()
        tracked_keys = self.tracked_obstacles_keys[:]
        iou_matrix = []
        checked = [False] * len(confirmed_obstacles)

        for key in tracked_keys:
            past_pose = tracked_copy[key][1]
            ious = [utils.get_iou(past_pose, obs[1], 0.05) for obs in confirmed_obstacles]
            iou_matrix.append(ious)

        for i, ious in enumerate(iou_matrix):
            if not ious:
                continue
            max_iou = max(ious)
            idx = ious.index(max_iou)
            if max_iou > 0.0:
                obs = confirmed_obstacles[idx]
                self.tracked_obstacles[tracked_keys[i]][1] = obs[1]
                self.tracked_obstacles[tracked_keys[i]][2] = obs[2]
                self.tracked_obstacles[tracked_keys[i]][3].append(obs[1])
                self.tracked_obstacles[tracked_keys[i]][4] = time.time() - self.tracked_obstacles[tracked_keys[i]][4]
                checked[idx] = True
            else:
                del self.tracked_obstacles[tracked_keys[i]]
                self.tracked_obstacles_keys.remove(tracked_keys[i])

        for i, c in enumerate(checked):
            if not c and confirmed_obstacles[i][0] == 'o':
                uid = uuid4()
                obs = confirmed_obstacles[i]
                self.tracked_obstacles[uid] = [obs[0], obs[1], obs[2], deque([obs[1]]), time.time(), -1, [0.0, 0.0]]
        self.tracked_obstacles_keys = list(self.tracked_obstacles.keys())

    def estimate_velocities(self):
        for key in self.tracked_obstacles_keys:
            dq = self.tracked_obstacles[key][3]
            if len(dq) > 1:
                p0, p1 = dq[0], dq[1]
                dt = self.tracked_obstacles[key][4]
                if dt == 0.0:
                    v = 0.0
                    vx = 0.0
                    vy = 0.0
                else:
                    try:
                        v = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / dt
                        vx = (p1[0] - p0[0]) / dt
                        vy = (p1[1] - p0[1]) / dt
                    except ZeroDivisionError:
                        v = 0.0
                        vx = 0.0
                        vy = 0.0
                self.tracked_obstacles[key][5] = v
                self.tracked_obstacles[key][6] = [vx, vy]

    def compute_collision_probabilities(self):
        if len(self.agent_pose_deque) < 2:
            return []

        p0, p1 = self.agent_pose_deque[0], self.agent_pose_deque[1]
        agent_vel = utils.get_timestep_velocity(self.agent_pose_deque, self.agent_vel_timestep)

        collision_probs = []
        for key in self.tracked_obstacles_keys:
            obs = self.tracked_obstacles[key]
            if len(obs[3]) < 2:
                continue
            obs_pose = obs[1]
            rel_vel = agent_vel - obs[5]
            dist = utils.get_collision_point([p0, p1], obs_pose, self.robot_width)
            if dist is None or rel_vel == 0.0:
                cp = utils.compute_general_collision_prob(obs[2], self.max_scan_range, self.min_scan_range)
            else:
                try:
                    ttc = dist / rel_vel if rel_vel != 0.0 else float('inf')
                except ZeroDivisionError:
                    ttc = float('inf')
                cp = 0.5 * utils.compute_collision_prob(ttc) + 0.5 * utils.compute_general_collision_prob(
                    obs[2], self.max_scan_range, self.min_scan_range)
            collision_probs.append((cp, obs_pose, obs[6]))
        return collision_probs

    def scan_callback(self, scan):
        try:
            yaw = self.get_yaw()
            self.update_agent_pose()
            confirmed = self.ob_filter.classify_obstacles(scan, self.position, yaw, self.ground_truth_scans)
            self.track_obstacles(confirmed)
            self.estimate_velocities()
            cp_list = self.compute_collision_probabilities()

            self.get_logger().info(f"Tracked Obstacles: {len(cp_list)}")
            for cp, pos, vel in cp_list:
                self.get_logger().info(f"CP={cp:.2f}, Pos=({pos[0]:.2f},{pos[1]:.2f}), Vel=({vel[0]:.2f},{vel[1]:.2f})")
        except Exception as e:
            self.get_logger().warn(f"Tracking error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
