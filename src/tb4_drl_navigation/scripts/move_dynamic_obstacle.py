import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
import math
import time

OBSTACLES = {
    "obstacle_1": (2.0, 2.0),
    "obstacle_2": (2.0, -2.0),
    "obstacle_3": (-2.0, -2.0),
    "obstacle_4": (-2.0, 2.0),
    "obstacle_5": (4.0, 4.0),
    "obstacle_6": (4.0, -4.0),
    "obstacle_7": (-4.0, -4.0),
    "obstacle_8": (-4.0, 4.0),
}

class DynamicObstacleMover(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_mover')
        self.cli = self.create_client(SetEntityPose, '/world/static_world/set_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/static_world/set_pose service...')
        self.circle_entity_name = 'dynamic_obstacle_1'
        self.radius = 1.0 # Circle radius for dynamic_obstacle_1
        self.speed = 0.25  # radians/sec for all
        self.z = 0.1

    def move_all_obstacles_smooth(self):
        t = 0.0
        phase_offsets_diag = {
            "obstacle_5": 0.0,
            "obstacle_6": math.pi / 2,
            "obstacle_7": math.pi,
            "obstacle_8": 3 * math.pi / 2,
        }
        phase_offsets_vert = {
            "obstacle_1": 0.0,
            "obstacle_2": math.pi / 2,
            "obstacle_3": math.pi,
            "obstacle_4": 3 * math.pi / 2,
        }
        while rclpy.ok():
            # Move obstacles 1-4 vertically (y oscillates by ±0.5m)
            for name in ["obstacle_1", "obstacle_2", "obstacle_3", "obstacle_4"]:
                orig_x, orig_y = OBSTACLES[name]
                phase = phase_offsets_vert[name]
                offset = 0.7 * math.sin(self.speed * t + phase)
                new_y = orig_y + offset
                pose = Pose()
                pose.position.x = orig_x
                pose.position.y = new_y
                pose.position.z = 0.1
                pose.orientation.w = 1.0

                entity = Entity()
                entity.name = name
                entity.type = 2  # MODEL

                req = SetEntityPose.Request()
                req.entity = entity
                req.pose = pose

                self.cli.call_async(req)

            # # Move obstacles 5-8 diagonally (x and y oscillate together)
            # for name in ["obstacle_5", "obstacle_6", "obstacle_7", "obstacle_8"]:
            #     orig_x, orig_y = OBSTACLES[name]
            #     phase = phase_offsets_diag[name]
            #     offset = 0.7 * math.sin(self.speed * t + phase)
            #     new_x = orig_x + offset
            #     new_y = orig_y + offset
            #     pose = Pose()
            #     pose.position.x = new_x
            #     pose.position.y = new_y
            #     pose.position.z = 0.1
            #     pose.orientation.w = 1.0

            #     entity = Entity()
            #     entity.name = name
            #     entity.type = 2  # MODEL

            #     req = SetEntityPose.Request()
            #     req.entity = entity
            #     req.pose = pose

            #     self.cli.call_async(req)

            # Move dynamic_obstacle_1 in a circle
            x = self.radius * math.cos(self.speed * t*0.8)
            y = self.radius * math.sin(self.speed * t*0.8)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.z
            pose.orientation.w = 1.0

            entity = Entity()
            entity.name = self.circle_entity_name
            entity.type = 2  # MODEL

            req = SetEntityPose.Request()
            req.entity = entity
            req.pose = pose

            self.cli.call_async(req)

            t += 0.05
            time.sleep(0.05)  # ~20 Hz update for smooth motion

def main(args=None):
    rclpy.init(args=args)
    mover = DynamicObstacleMover()
    try:
        mover.move_all_obstacles_smooth()
    except KeyboardInterrupt:
        pass
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()