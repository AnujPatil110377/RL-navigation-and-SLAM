import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity, SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose
import random
import math
import time

class DynamicObstacleManager(Node):
    def reset_obstacles(self, request, response):
        self.get_logger().info('Resetting dynamic obstacles...')
        # Remove all obstacles (not strictly necessary, but can be implemented if needed)
        self.positions = []
        self.velocities = []
        self.spawned_names = []
        self.spawn_all_obstacles()
        return response

    def __init__(self):
        super().__init__('dynamic_obstacle_manager')
        self.spawn_cli = self.create_client(SpawnEntity, '/world/static_world/create')
        self.pose_cli = self.create_client(SetEntityPose, '/world/static_world/set_pose')

        # Add a service to reset obstacles
        from std_srvs.srv import Trigger
        self.reset_srv = self.create_service(Trigger, '/reset_dynamic_obstacles', self.reset_obstacles)

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /create service...')
        while not self.pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_pose service...')

        self.num = 8
        self.names = [f'obstacle_{i}' for i in range(self.num)]
        self.positions = []
        self.velocities = []
        self.radius = 0.5
        self.world_bounds = (-5, 5, -5, 5)

        self.spawn_all_obstacles()
        self.timer = self.create_timer(0.05, self.update)

    def spawn_all_obstacles(self):
        sdf_template = """
        <sdf version='1.6'>
          <model name='{name}'>
            <pose>{x} {y} 0.1 0 0 0</pose>
            <static>false</static>
            <include>
              <uri>model://obstacle</uri>
            </include>
          </model>
        </sdf>
        """
        self.spawned_names = []
        # Get robot spawn region (assume robot is spawned in one of the 4 regions)
        robot_spawn_xy = getattr(self, 'robot_spawn_xy', None)
        robot_region = None
        if robot_spawn_xy is not None:
            x, y = robot_spawn_xy
            if x < 0 and y < 0:
                robot_region = 0  # Bottom-left
            elif x >= 0 and y < 0:
                robot_region = 1  # Bottom-right
            elif x < 0 and y >= 0:
                robot_region = 2  # Top-left
            elif x >= 0 and y >= 0:
                robot_region = 3  # Top-right
        for i in range(self.num):
            # Assign region, skip robot region
            region = i % 4
            if region == robot_region:
                region = (region + 1) % 4  # Shift to next region if matches robot
            region_bounds = [
                (-4.5, 0, -4.5, 0),    # Bottom-left
                (0, 4.5, -4.5, 0),     # Bottom-right
                (-4.5, 0, 0, 4.5),     # Top-left
                (0, 4.5, 0, 4.5)       # Top-right
            ]
            x_min, x_max, y_min, y_max = region_bounds[region]
            max_tries = 100
            for _ in range(max_tries):
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                too_close = False
                for px, py in self.positions:
                    if math.hypot(px - x, py - y) < self.radius * 4:
                        too_close = True
                        break
                if not too_close:
                    break
            else:
                x, y = (x_min + x_max) / 2, (y_min + y_max) / 2
            self.positions.append([x, y])
            vx, vy = self.random_velocity(region)
            self.velocities.append([vx, vy])
            sdf = sdf_template.format(name=self.names[i], x=x, y=y)
            req = SpawnEntity.Request()
            req.entity_factory.sdf = sdf
            req.entity_factory.name = self.names[i]
            req.entity_factory.pose.position.x = x
            req.entity_factory.pose.position.y = y
            req.entity_factory.pose.position.z = 0.1
            req.entity_factory.pose.orientation.w = 1.0
            future = self.spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None and hasattr(result, 'success') and result.success:
                self.spawned_names.append(self.names[i])
                self.get_logger().info(f"Spawned obstacle: {self.names[i]} at ({x:.2f}, {y:.2f})")
            else:
                self.get_logger().error(f"Failed to spawn obstacle: {self.names[i]} at ({x:.2f}, {y:.2f})")
            time.sleep(0.1)

    def random_velocity(self, region=None):
        # Assign uniform random directions for each region
        if region is None:
            region = len(self.positions) % 4
        base_angles = [math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4]
        angle = base_angles[region] + random.uniform(-math.pi/8, math.pi/8)
        speed = 0.2
        return [speed * math.cos(angle), speed * math.sin(angle)]

    def update(self):
        for i in range(self.num):
            # Only update pose if obstacle was spawned successfully
            if self.names[i] not in getattr(self, 'spawned_names', self.names):
                continue
            x, y = self.positions[i]
            vx, vy = self.velocities[i]

            new_x = x + vx * 0.05
            new_y = y + vy * 0.05

            if new_x < self.world_bounds[0] or new_x > self.world_bounds[1]:
                vx *= -1
                new_x = x
            if new_y < self.world_bounds[2] or new_y > self.world_bounds[3]:
                vy *= -1
                new_y = y

            for j in range(self.num):
                if i == j:
                    continue
                xj, yj = self.positions[j]
                if math.hypot(new_x - xj, new_y - yj) < self.radius * 2:
                    vx *= -1
                    vy *= -1
                    new_x = x
                    new_y = y
                    break

            self.positions[i] = [new_x, new_y]
            self.velocities[i] = [vx, vy]

            pose = Pose()
            pose.position.x = new_x
            pose.position.y = new_y
            pose.position.z = 0.1
            pose.orientation.w = 1.0

            entity = Entity()
            entity.name = self.names[i]
            entity.type = 2

            req = SetEntityPose.Request()
            req.entity = entity
            req.pose = pose
            self.pose_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
