import math
from pathlib import Path
import time
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
)
from geometry_msgs.msg import Pose
import gymnasium as gym
from gymnasium import spaces
import numpy as np
np.float = float # Added for compatibility with older numpy versions
import rclpy
from rclpy.executors import MultiThreadedExecutor
from tb4_drl_navigation.envs.diffdrive.scenario_generator import ScenarioGenerator
from tb4_drl_navigation.envs.utils.ros_gz import (
    Publisher,
    Sensors,
    SimulationControl,
)
from tb4_drl_navigation.utils.dtype_convertor import (
    PoseConverter,
    TwistConverter,
)
from tb4_drl_navigation.utils.launch import Launcher
from transforms3d.euler import (
    euler2quat,
    quat2euler,
)

class Turtlebot4Env(gym.Env):
    def _get_waypoint_reward(self, current_pos, waypoints, last_waypoint_idx):
        """
        Returns a reward if the robot passes a new waypoint.
        Args:
            current_pos: np.array([x, y])
            waypoints: list of (x, y) tuples
            last_waypoint_idx: int, last reached waypoint index
        Returns:
            reward: float
            new_idx: int, updated last waypoint index
        """
        reward = 0.0
        new_idx = last_waypoint_idx
        if waypoints and last_waypoint_idx + 1 < len(waypoints):
            next_wp = np.array(waypoints[last_waypoint_idx + 1])
            if np.linalg.norm(current_pos - next_wp) < 0.4:  # 0.4m threshold
                reward = 10.0
                new_idx = last_waypoint_idx + 1
        return reward, new_idx
    """
    Enhanced Gymnasium environment for a ROS2/Gazebo Turtlebot4 navigation task with
    full research paper implementation including:
    - Collision Probability (CP) calculation as per research paper
    - K most dangerous obstacles prioritization
    - Social and ego safety violation tracking
    - Waypoint-based learning enhancement
    - Risk perception integrated into observation space

    Uses Turtlebot4-specific parameters, not paper's TurtleBot3 specs.
    """

    import subprocess
    import threading

    def _kill_tracker_processes(self):
        import psutil
        # Kill all processes with names containing 'obstacle_tracke' or 'obstacle_extrac'
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info.get('cmdline', []))
                if ('obstacle_tracke' in cmdline) or ('obstacle_extrac' in cmdline):
                    print(f"Killing tracker process PID {proc.pid}: {cmdline}")
                    proc.kill()
            except Exception as e:
                print(f"Error killing process {proc.pid}: {e}")

    def _start_tracker_node(self):
        # Kill previous tracker processes if running
        self._kill_tracker_processes()
        # Kill previous subprocess if running
        if hasattr(self, '_tracker_proc') and self._tracker_proc is not None:
            if self._tracker_proc.poll() is None:
                print("Terminating previous tracker node...")
                self._tracker_proc.terminate()
                try:
                    self._tracker_proc.wait(timeout=5)
                except Exception as e:
                    print(f"Failed to terminate previous tracker node: {e}")

        # Launch new tracker node using ros2 launch
        try:
            self._tracker_proc = self.subprocess.Popen([
                'ros2', 'launch', 'obstacle_detector', 'obstacle_extractor_and_tracker.launch'
            ])
            print("Launched: ros2 launch obstacle_detector obstacle_extractor_and_tracker.launch")
        except Exception as e:
            print(f"Failed to launch tracker node: {e}")

    def _restart_tracker_periodically(self):
        # Restart tracker every 5 minutes
        def restart():
            self._start_tracker_node()
            self._tracker_timer = self.threading.Timer(300, restart)
            self._tracker_timer.daemon = True
            self._tracker_timer.start()
        restart()

    def _close_tracker_node(self):
        self._kill_tracker_processes()
        if hasattr(self, '_tracker_proc') and self._tracker_proc is not None:
            if self._tracker_proc.poll() is None:
                print("Terminating tracker node on close...")
                self._tracker_proc.terminate()
                try:
                    self._tracker_proc.wait(timeout=5)
                except Exception as e:
                    print(f"Failed to terminate tracker node: {e}")
        if hasattr(self, '_tracker_timer') and self._tracker_timer is not None:
            self._tracker_timer.cancel()

    def __init__(
        self,
        world_name: str = 'static_world',
        robot_name: str = 'turtlebot4',
        map_path: Optional[Path] = None,
        yaml_path: Optional[Path] = None,
        sim_launch_name: Optional[Path] = None,
        robot_radius: float = 0.3, # TurtleBot4 radius
        min_separation: float = 1.5,
        goal_sampling_bias: str = 'uniform',
        obstacle_prefix: str = 'obstacle',
        obstacle_clearance: float = 2.0,
        num_bins: int = 30,
        goal_threshold: float = 0.35,
        collision_threshold: float = 0.4,
        time_delta: float = 0.15,
        shuffle_on_reset: bool = True
    ):
        super(Turtlebot4Env, self).__init__()

        # Store original obstacle positions for restoring when shuffle_on_reset is False
        self._original_obstacle_positions = None
        self.world_name = world_name
        self.robot_name = robot_name

        # Always use the source directory for maps, not the install directory
        src_maps_dir = Path('/home/robotics/ros2_ws/gym-turtlebot/src/tb4_drl_navigation/tb4_drl_navigation/envs/diffdrive/maps')
        self.map_path = map_path or src_maps_dir / f'{world_name}.pgm'
        self.yaml_path = yaml_path or src_maps_dir / f'{world_name}.yaml'
        self.sim_launch_name = sim_launch_name
        self.robot_radius = robot_radius # TurtleBot4 radius

        # --- RESEARCH PAPER CP PARAMETERS ---
        self.cp_alpha = 0.0        # α weighting factor from paper
        self.l_max = 2.5           # far-field distance threshold
        self.l_min = self.robot_radius + 0.05  # robot radius + safety buffer
        self.horizon = 5.0         # time horizon for TTC calculation

        self.min_separation = min_separation
        self.goal_sampling_bias = goal_sampling_bias
        self.obstacle_clearance = obstacle_clearance
        self.obstacle_prefix = obstacle_prefix

        # --- K-OBSTACLE PRIORITIZATION PARAMETERS (from paper) ---
        self.k_obstacle_count = 4 # Paper uses K=8 most dangerous obstacles

        # --- SOCIAL AND EGO SAFETY TRACKING (from paper) ---
        self.social_safety_violation_count = 0
        self.ego_safety_violation_count = 0
        self.obstacle_present_step_counts = 0

        if self.sim_launch_name:
            self._launch_simulation()

        self.num_bins = num_bins
        self.time_delta = time_delta
        self.shuffle_on_reset = shuffle_on_reset
        self.goal_threshold = goal_threshold
        self.collision_threshold = collision_threshold

        self.sensors = Sensors(node_name=f'{self.robot_name}_sensors')
        self.ros_gz_pub = Publisher(node_name=f'{self.robot_name}_gz_pub')
        self.simulation_control = SimulationControl(
            world_name=self.world_name, node_name=f'{self.robot_name}_world_control'
        )

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.sensors)
        self.executor.add_node(self.ros_gz_pub)
        self.executor.add_node(self.simulation_control)
        self.executor.spin_once(timeout_sec=1.0)

        self.pose_converter = PoseConverter()
        self.twist_converter = TwistConverter()

        self.nav_scenario = ScenarioGenerator(
            map_path=self.map_path,
            yaml_path=self.yaml_path,
            robot_radius=self.robot_radius,
            min_separation=self.min_separation,
            obstacle_clearance=self.obstacle_clearance,
            seed=None
        )

        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self.observation_space = self._build_observation_space()
        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)
        self._goal_pose = None
        self._start_pose = None
        self._prev_dist_to_goal: float = 0.0

        # --- Obstacle tracking from /obstacles topic ---
        self.tracked_circles = [] # List of dicts with keys: uid, center, velocity, radius

        try:
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import Header
            import threading
            from rclpy.node import Node
            # Import the message type for /obstacles topic
            from obstacle_detector.msg import Obstacles
        except ImportError:
            Obstacles = None

        self._obstacles_msg_type = Obstacles

        if Obstacles is not None:
            self.sensors.create_subscription(
                Obstacles,
                '/obstacles',
                self._obstacles_callback,
                10
            )

    def _obstacles_callback(self, msg):
        """Callback to update tracked circles from /obstacles topic"""
        # Only track circles
        self.tracked_circles = []
        for circle in getattr(msg, 'circles', []):
            self.tracked_circles.append({
                'uid': getattr(circle, 'uid', None),
                'center': np.array([circle.center.x, circle.center.y]),
                'velocity': np.array([circle.velocity.x, circle.velocity.y]),
                'radius': 0.25  # Set obstacle radius to 0.25 for all obstacles
            })

    def _launch_simulation(self) -> None:
        workspace_dir = Path(__file__).parent
        while workspace_dir.name != 'gym-turtlebot' and workspace_dir.parent != workspace_dir:
            workspace_dir = workspace_dir.parent
        launcher = Launcher(workspace_dir=workspace_dir)
        launcher.launch(
            'tb4_gz_sim',
            self.sim_launch_name,
            'use_sim_time:=true',
            build_first=True,
        )

    def _build_observation_space(self) -> spaces.Dict:
        self.simulation_control.pause_unpause(pause=False)

        # Wait for scan and odometry to initialize
        while True:
            self.executor.spin_once(timeout_sec=0.1)
            range_min, range_max = self.sensors.get_range_min_max()
            angle_min, angle_max = self.sensors.get_angle_min_max()
            if None not in (range_min, range_max, angle_min, angle_max):
                break

        return spaces.Dict({
            'min_ranges': spaces.Box(
                low=range_min, high=range_max, shape=(self.num_bins,), dtype=np.float32
            ),
            # 'min_ranges_angle': spaces.Box(
            #     low=angle_min, high=angle_max, shape=(self.num_bins,), dtype=np.float32
            # ),
            'dist_to_goal': spaces.Box(low=0.0, high=100.0, shape=(1,), dtype=np.float32),
            'orient_to_goal': spaces.Box(low=-math.pi, high=math.pi, shape=(1,), dtype=np.float32),
            'action': spaces.Box(
                low=self.action_space.low,
                high=self.action_space.high,
                shape=self.action_space.shape,
                dtype=np.float32
            ),
            # --- FIXED: K-obstacle observation space (paper implementation) ---
            'obs_k': spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(self.k_obstacle_count * 5,), # [x, y, vx, vy, CP] for each of K obstacles
                dtype=np.float32
            ),
        })

    def _compute_obs_k(self) -> np.ndarray:
        """
        FIXED: Research paper's exact collision probability calculation:
        CP = α × Pc_ttc + (1 - α) × Pc_dto
        """
        # Get robot state
        robot_pos = self._current_position
        robot_vel = np.array([self._last_action[0], 0.0], dtype=np.float32)
        
        cp_list = []
        
        for c in self.tracked_circles:
            obs_pos = c['center']
            obs_vel = c.get('velocity', np.zeros(2, dtype=np.float32))
            obs_radius = c.get('radius', 0.0)

            # Calculate relative position and velocity
            rel_pos = obs_pos - robot_pos
            rel_vel = robot_vel - obs_vel  # Vr - Vo as per paper

            distance_to_obstacle = np.linalg.norm(rel_pos)

            # --- 1. Time-to-Collision Component (Pc-ttc) ---
            ttc = self._calculate_time_to_collision(rel_pos, rel_vel, obs_radius)
            pc_ttc = self._compute_pc_ttc(ttc)

            # --- 2. Distance-to-Obstacle Component (Pc-dto) ---  
            pc_dto = self._compute_pc_dto(distance_to_obstacle)

            # --- 3. Final CP calculation (paper's exact formula) ---
            cp = self.cp_alpha * pc_ttc + (1 - self.cp_alpha) * pc_dto

            cp_list.append((cp, obs_pos, obs_vel))

            # Debug output matching paper's format, handle None values
            # pos_str = f"[{obs_pos[0]:.2f}, {obs_pos[1]:.2f}]" if obs_pos is not None else "[N/A, N/A]"
            # ttc_str = f"{ttc:.3f}" if ttc is not None else "N/A"
            # pc_ttc_str = f"{pc_ttc:.3f}" if pc_ttc is not None else "N/A"
            # pc_dto_str = f"{pc_dto:.3f}" if pc_dto is not None else "N/A"
            # cp_str = f"{cp:.3f}" if cp is not None else "N/A"
            # rel_vel_str = f"[{rel_vel[0]:.2f}, {rel_vel[1]:.2f}]"
            # print(f"[CP] Obs at {pos_str}: TTC={ttc_str}s, Pc-ttc={pc_ttc_str}, Pc-dto={pc_dto_str}, CP={cp_str}, RelVel={rel_vel_str}")

        # Sort by collision probability (descending) and take top K
        cp_list.sort(key=lambda x: x[0], reverse=True)
        
        # Format for top K obstacles: [x, y, vx, vy, CP] for each obstacle
        flat_obs_k = []
        for i in range(self.k_obstacle_count):
            if i < len(cp_list):
                cp, pos, vel = cp_list[i]
                # Compute relative velocity for printing
                rel_vel = robot_vel - vel
                flat_obs_k.extend([pos[0], pos[1], rel_vel[0], rel_vel[1], cp])
            else:
                # Placeholder values for missing obstacles
                flat_obs_k.extend([robot_pos[0], robot_pos[1], 0.0, 0.0, 0.0])
        
        return np.array(flat_obs_k, dtype=np.float32)

    def _calculate_time_to_collision(self, rel_pos, rel_vel, obs_radius):
        combined_radius = self.robot_radius + obs_radius + 0.05
        a = np.dot(rel_vel, rel_vel)
        b = 2.0 * np.dot(rel_pos, rel_vel)
        c = np.dot(rel_pos, rel_pos) - combined_radius**2
        discriminant = b*b - 4*a*c

        if a < 1e-6:
            return None
        if discriminant < 0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        collision_times = [t for t in [t1, t2] if 0.0 < t < self.horizon]
        return min(collision_times) if collision_times else None

    def _compute_pc_ttc(self, ttc: Optional[float]) -> float:
        """
        Compute collision probability based on time-to-collision (paper's exact formula).
        """
        if ttc is None:
            return 0.0
        
        # Paper's exact formula: Pc-ttc = min(1, 0.15/ttc)
        return min(1.0, 0.5 / ttc)

    def _compute_pc_dto(self, distance: float) -> float:
        """
        Compute collision probability based on distance-to-obstacle (paper's exact formula).
        """
        if distance >= self.l_max:
            return 0.0
        if distance <= self.l_min:
            return 1.0
        
        norm = 0.8*(self.l_max - distance) / (self.l_max - self.l_min)
        return min(1.0, max(0.0, norm))

    def _get_info(self) -> Dict[str, Any]:
        """Return episode info including safety scores as per paper"""
        info = {}
        # Calculate social and ego safety scores as per paper
        if self.obstacle_present_step_counts > 0:
            social_safety_score = 1.0 - (self.social_safety_violation_count / self.obstacle_present_step_counts)
            ego_safety_score = 1.0 - (self.ego_safety_violation_count / self.obstacle_present_step_counts)
            info['social_safety_score'] = social_safety_score
            info['ego_safety_score'] = ego_safety_score
            info['social_safety_violations'] = self.social_safety_violation_count
            info['ego_safety_violations'] = self.ego_safety_violation_count
            info['obstacle_present_steps'] = self.obstacle_present_step_counts
        return info

    def _get_obs(self) -> Dict:
        min_ranges, min_ranges_angle = self._process_lidar()
        dist_to_goal, orient_to_goal = self._process_odom()

        obs = {
            'min_ranges': np.array(min_ranges, dtype=np.float32),
            # 'min_ranges_angle': np.array(min_ranges_angle, dtype=np.float32),
            'dist_to_goal': np.array([dist_to_goal], dtype=np.float32),
            'orient_to_goal': np.array([orient_to_goal], dtype=np.float32),
            'action': self._last_action.astype(np.float32),
        }

        # --- Add top-K obstacle pose+velocity+CP ---
        obs['obs_k'] = self._compute_obs_k()

        return obs

    def _process_lidar(self) -> Tuple[List[float], List[float]]:
        # Get laser scan data
        ranges = self.sensors.get_latest_scan()
        range_min, range_max = self.sensors.get_range_min_max()
        angle_min, angle_max = self.sensors.get_angle_min_max()

        num_ranges = len(ranges)
        # Calculate bin width and mid
        self.num_bins = min(max(1, self.num_bins), num_ranges)
        bin_width = (angle_max - angle_min) / self.num_bins

        # Initialize bins with default values centred at bin centre
        min_ranges = [range_max] * self.num_bins
        min_ranges_angle = [
            angle_min + (i * bin_width) + bin_width/2 for i in range(self.num_bins)
        ]

        # Process ranges
        for i in range(num_ranges):
            current_range = ranges[i]
            current_angle = angle_min + i * (angle_max - angle_min) / (num_ranges - 1)

            # Clip current_angle to handle floating point precision
            current_angle = max(angle_min, min(current_angle, angle_max))

            # Take the default for invalid range
            if not (range_min <= current_range <= range_max) or not math.isfinite(current_range):
                continue

            # Calculate bin index
            bin_idx = (current_angle - angle_min) // bin_width
            bin_idx = int(max(0, min(bin_idx, self.num_bins - 1)))

            # Update min range and angle
            if current_range < min_ranges[bin_idx]:
                min_ranges[bin_idx] = current_range
                min_ranges_angle[bin_idx] = current_angle

        return min_ranges, min_ranges_angle

    def _process_odom(self) -> Tuple[float, float]:
        # Get current pose
        pose_stamped = self.sensors.get_latest_pose_stamped()
        agent_pose = pose_stamped.pose

        # Extract positions
        agent_x = agent_pose.position.x
        agent_y = agent_pose.position.y

        # Use only _goal_pose for navigation
        goal_pose = self._goal_pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        # Update current position for reward function and K-obstacle computation
        self._current_position = np.array([agent_x, agent_y], dtype=np.float32)

        # Calculate relative distance
        dx = goal_x - agent_x
        dy = goal_y - agent_y
        distance = math.hypot(dx, dy)

        # Handle zero-distance edge case
        if math.isclose(distance, 0.0, abs_tol=1e-3):
            return (0.0, 0.0)

        # Calculate bearing to goal (global frame)
        bearing = math.atan2(dy, dx)

        # Extract current orientation
        q = [
            agent_pose.orientation.w,
            agent_pose.orientation.x,
            agent_pose.orientation.y,
            agent_pose.orientation.z
        ]
        _, _, yaw = quat2euler(q, 'sxyz')

        # Calculate relative angle (robot's frame)
        relative_angle = bearing - yaw

        # Normalize angle to [-pi, pi]
        relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))

        return distance, relative_angle

    def step(
        self,
        action: np.ndarray,
        debug: Optional[bool] = None,
        evaluation: bool = False
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:

        # Store action for inclusion in next observation
        self._last_action = action.copy()

        # Store evaluation flag
        self._is_evaluation = evaluation

        # Execute the action, propagate for time_delta
        twist_msg = self.twist_converter.from_dict({
            'linear': (float(action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(action[1]))
        })

        self.ros_gz_pub.pub_cmd_vel(twist_msg)
        observation, info = self._propagate_state(time_delta=self.time_delta)
        self.ros_gz_pub.pub_robot_path(pose_stamped=self.sensors.get_latest_pose_stamped())

        # Track step count in episode
        if not hasattr(self, '_episode_step_count'):
            self._episode_step_count = 0
        self._episode_step_count += 1

        # --- Social and Ego Safety Tracking (from paper) ---
        if len(self.tracked_circles) > 0:
            self.obstacle_present_step_counts += 1

            # Check for ego safety violations (obstacle too close)
            min_distance = float('inf')
            for c in self.tracked_circles:
                dist = np.linalg.norm(c['center'] - self._current_position)
                min_distance = min(min_distance, dist)

                # Ego safety violation: obstacle within robot's ego radius (adapted for TurtleBot4)
                if dist < self.robot_radius * 0.787: # Paper's ratio but with TurtleBot4 radius
                    self.ego_safety_violation_count += 1
                    break

            # Check for social safety violations using collision probability
            obs_k_data = self._compute_obs_k()
            for i in range(self.k_obstacle_count):
                if i * 5 + 4 < len(obs_k_data): # Check if CP data exists
                    cp = obs_k_data[i * 5 + 4] # CP is the 5th element
                    if cp > 0.4: # Paper's social safety threshold
                        self.social_safety_violation_count += 1
                        break

        # Check termination conditions BEFORE reward calculation
        goal_reached = self._goal_reached(dist_to_goal=observation['dist_to_goal'].item())
        collision = self._collision(min_ranges=observation['min_ranges'])

        # Check for timeout penalty (treat as special penalty after 500 steps)
        timeout_collision = False
        if self._episode_step_count >= 500:
            timeout_collision = True
            collision = True
            self.sensors.get_logger().info(f'Episode timeout at step {self._episode_step_count} - applying timeout penalty!')

        # Log termination events immediately when they occur
        if goal_reached:
            self.sensors.get_logger().info('Goal reached!')
        elif collision and not timeout_collision:
            self.sensors.get_logger().info('Collision detected!')

        # Calculate reward using new reward function signature
        if timeout_collision:
            reward = -50.0
        else:
            reward = self._get_reward(
                action=action,
                min_ranges=observation['min_ranges'],
                dist_to_goal=observation['dist_to_goal'].item(),
                orient_to_goal=observation['orient_to_goal'].item()
            )
        # Print reward per step for debugging
        print(f"Step {self._episode_step_count}: Reward = {reward:.3f}")

        # If collision occurs within first 8 steps, apply zero penalty (exploration phase)
        early_collision = False
        if collision and not timeout_collision and self._episode_step_count < 8:
            reward = 0.0
            early_collision = True
            self.sensors.get_logger().info(f'Early collision at step {self._episode_step_count} - zero penalty: {reward}')

        # --- Print collision probabilities of tracked obstacles ---
        # if hasattr(self, 'tracked_circles') and self.tracked_circles:
        #     print(f"Tracked {len(self.tracked_circles)} circle obstacles:")
        #     # Get collision probabilities from observation
        #     obs_k_data = self._compute_obs_k()
        #     print(f"Top {self.k_obstacle_count} most dangerous obstacles:")
        #     for i in range(self.k_obstacle_count):
        #         if i * 5 + 4 < len(obs_k_data):
        #             x, y, rvx, rvy, cp = obs_k_data[i*5:(i+1)*5]
        #             if cp > 0.0: # Only show obstacles with non-zero CP
        #                 print(f" {i+1}. CP: {cp:.3f}, Pos: [{x:.2f}, {y:.2f}], RelVel: [{rvx:.2f}, {rvy:.2f}]")
        # else:
        #     print("No circle obstacles tracked from /obstacles topic.")

        # Log reward to terminal
        # print(f"Step {self._episode_step_count}: Reward = {reward:.3f}")

        # MDP
        truncated = False # Handled by TimeLimit wrapper in the SB3 setup
        terminated = goal_reached or collision

        # Add safety scores to info
        info.update(self._get_info())

        # If early collision, set a flag in info so RL buffer can ignore this transition
        if early_collision:
            info = dict(info)
            info['ignore_in_buffer'] = True

        # If timeout collision, mark it in info for RL buffer handling
        if timeout_collision:
            info = dict(info)
            info['timeout_collision'] = True

        if debug:
            self.ros_gz_pub.publish_observation(
                observation=observation,
                robot_pose=self.sensors.get_latest_pose_stamped(),
                goal_pose=self._goal_pose
            )

        # Reset step count if episode ends
        if terminated:
            self._episode_step_count = 0

        # Print per-step reward breakdown for debugging
        print(f"Step {self._episode_step_count}: Reward = {reward:.3f}")
        # Optionally, print more details (uncomment if needed):
        # print(f"  Progress: {locals().get('progress_reward', 0):.3f}, Orientation: {locals().get('orientation_reward', 0):.3f}, Action: {locals().get('action_reward', 0):.3f}, Waypoint: {locals().get('waypoint_reward', 0):.3f}, Obstacle: {locals().get('obstacle_reward', 0):.3f}, Avoidance: {locals().get('avoidance_bonus', 0):.3f}, Stationary: {locals().get('stationary_penalty', 0):.3f}, Time: {locals().get('time_penalty', 0):.3f}")
        return observation, reward, terminated, truncated, info

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
        evaluation: bool = False
    ) -> Tuple[Dict, Dict[str, Any]]:

        super().reset(seed=seed)

        # Store evaluation flag
        self._is_evaluation = evaluation

        self.simulation_control.reset_world()

        # Start/restart tracker node and periodic timer
        self._start_tracker_node()
        if not hasattr(self, '_tracker_timer') or self._tracker_timer is None:
            self._restart_tracker_periodically()

        # Reset safety tracking counters
        self.social_safety_violation_count = 0
        self.ego_safety_violation_count = 0
        self.obstacle_present_step_counts = 0

        # Initialize last action to zeros
        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)

        twist_msg = self.twist_converter.from_dict({
            'linear': (float(self._last_action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(self._last_action[1]))
        })

        self.ros_gz_pub.pub_cmd_vel(twist_msg)

        options = options or {}
        start_pos = options.get('start_pos') # (x, y, yaw)
        goal_pos = options.get('goal_pos') # (x, y, yaw)

        # Get obstacle names
        obstacles = self.simulation_control.get_obstacles(starts_with=self.obstacle_prefix)

        # On first reset, record original obstacle positions
        if self._original_obstacle_positions is None:
            self._original_obstacle_positions = []
            for obs_name in obstacles:
                obs_pose = self.simulation_control.get_entity_pose(obs_name)
                if obs_pose is not None:
                    self._original_obstacle_positions.append((obs_pose.position.x, obs_pose.position.y, obs_pose.position.z))
                else:
                    self._original_obstacle_positions.append((0.0, 0.0, 0.01))


        # === CUSTOM SPAWN LOGIC: robot in lower left, goal at random top position ===
        map_x_min, map_x_max = -4.0, 4.0  # Adjust to your map size
        map_y_min, map_y_max = -4.0, 4.0
        if start_pos is None:
            # Robot in lower left corner
            start_xy = (map_x_min, map_y_min)
            start_yaw = 0.0
            start_pos = (*start_xy, start_yaw)
        if goal_pos is None:
            # Goal at random position along the top edge (y = map_y_max)
            goal_x = np.random.uniform(map_x_min + 0.5, map_x_max - 0.5)
            goal_y = map_y_max
            goal_yaw = 0.0
            goal_pos = (goal_x, goal_y, goal_yaw)

        # === Generate obstacles with clearance from both robot and goal ===
        if self.shuffle_on_reset:
            # Obstacles must be at least obstacle_clearance from both robot and goal
            obstacles_pos = []
            max_attempts = 100
            for i in range(len(obstacles)):
                for attempt in range(max_attempts):
                    # Sample random position in map bounds
                    x = np.random.uniform(-3.5, 3.5)
                    y = np.random.uniform(-3.5, 3.5)
                    dist_robot = np.linalg.norm(np.array([x, y]) - np.array(start_pos[:2]))
                    dist_goal = np.linalg.norm(np.array([x, y]) - np.array(goal_pos[:2]))
                    if dist_robot > self.obstacle_clearance and dist_goal > self.obstacle_clearance:
                        obstacles_pos.append((x, y))
                        break
                else:
                    # Fallback: place far from robot
                    obstacles_pos.append((start_pos[0] + self.obstacle_clearance + 1.0, start_pos[1]))
            for obs_pos, obs_name in zip(obstacles_pos, obstacles[:len(obstacles_pos)]):
                obs_pose = self.pose_converter.from_dict({
                    'position': (obs_pos[0], obs_pos[1], 0.01),
                    'orientation': (0.0, 0.0, 0.0, 1.0)
                })
                self.simulation_control.set_entity_pose(
                    entity_name=obs_name, pose=obs_pose
                )
        else:
            # Restore obstacles to their original positions
            for obs_name, orig_pos in zip(obstacles, self._original_obstacle_positions):
                obs_pose = self.pose_converter.from_dict({
                    'position': (orig_pos[0], orig_pos[1], orig_pos[2]),
                    'orientation': (0.0, 0.0, 0.0, 1.0)
                })
                self.simulation_control.set_entity_pose(
                    entity_name=obs_name, pose=obs_pose
                )
            obstacles_pos = [(pos[0], pos[1]) for pos in self._original_obstacle_positions]

        # Convert orientations to quaternions
        start_quat = euler2quat(ai=0.0, aj=0.0, ak=start_pos[2], axes='sxyz')
        goal_quat = euler2quat(ai=0.0, aj=0.0, ak=goal_pos[2], axes='sxyz')

        # Convert pos to pose
        self._start_pose = self.pose_converter.from_dict({
            'position': (start_pos[0], start_pos[1], 0.01),
            'orientation': (start_quat[1], start_quat[2], start_quat[3], start_quat[0])
        })

        self._goal_pose = self.pose_converter.from_dict({
            'position': (goal_pos[0], goal_pos[1], 0.01),
            'orientation': (goal_quat[1], goal_quat[2], goal_quat[3], goal_quat[0])
        })

        # Store original goal for future reference
        self.original_goal_pose = self._goal_pose

        self.simulation_control.set_entity_pose(
            entity_name=self.robot_name,
            pose=self._start_pose
        )

        self.simulation_control.set_pose(
            pose=self._start_pose, frame_id='odom'
        )

        self.ros_gz_pub.pub_goal_marker(goal_pose=self._goal_pose)

        observation, info = self._propagate_state(time_delta=self.time_delta)
        self._prev_dist_to_goal = observation['dist_to_goal'].item()

        if options.get('debug'):
            self.ros_gz_pub.publish_observation(
                observation=observation,
                robot_pose=self.sensors.get_latest_pose_stamped(),
                goal_pose=self._goal_pose
            )

        self.ros_gz_pub.clear_path()

        return observation, info

    def _propagate_state(self, time_delta: float = 0.2) -> Tuple[Dict, Dict[str, Any]]:
        self.simulation_control.pause_unpause(pause=False)

        end_time = time.time() + time_delta
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=max(0, end_time - time.time()))

        self.simulation_control.pause_unpause(pause=True)

        observation = self._get_obs()
        info = {'distance_to_goal': observation['dist_to_goal'].item()}

        return observation, info

    def _shuffle_obstacles(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> None:
        """Get random obstacle locations and shuffle them"""
        obstacles = self.simulation_control.get_obstacles(starts_with=self.obstacle_prefix)
        obstacles_pos = self.nav_scenario.generate_obstacles(
            num_obstacles=len(obstacles), start_pos=start_pos, goal_pos=goal_pos
        )

        for obs_pos, obs_name in zip(obstacles_pos, obstacles[:len(obstacles_pos)]):
            # Convert to Pose
            obs_pose = self.pose_converter.from_dict({
                'position': (obs_pos[0], obs_pos[1], 0.01),
                'orientation': (0.0, 0.0, 0.0, 1.0)
            })
            self.simulation_control.set_entity_pose(
                entity_name=obs_name, pose=obs_pose
            )

    def _get_reward(
        self,
        action: np.ndarray,
        min_ranges: np.ndarray,
        dist_to_goal: float,
        orient_to_goal: float
    ) -> float:
        """Calculate the reward based on the current state and action taken.
        Args:
            action: The action taken [linear_vel, angular_vel]
            min_ranges: Array of minimum distance readings from lidar
            dist_to_goal: Distance to the goal
            orient_to_goal: Orientation to the goal
        Returns:
            float: The calculated reward
        """
        # Check if target is reached
        target = self._goal_reached(dist_to_goal=dist_to_goal)
        if target:
            return 200.0

        # Check if collision occurred
        collision = self._collision(min_ranges=min_ranges)
        if collision:
            return -200.0

        # Get minimum laser reading for obstacle avoidance
        min_laser = np.min(min_ranges)

        # === 1. PROGRESS REWARD (normalized and capped) ===
        progress_reward = 0.0
        if hasattr(self, '_prev_dist_to_goal'):
            progress = self._prev_dist_to_goal - dist_to_goal
            progress_reward = np.clip(5.0 * progress, -1.2, 1.2)

        # === 2. ORIENTATION REWARD (stronger guidance) ===
        laser_factor= 0.5 if min_laser < 2.0 else 1.0
        distance_factor = min(1.0, dist_to_goal / 5.0)
        orientation_reward = 0.5 * laser_factor * math.cos(orient_to_goal)

        # # === 3. SMOOTH ACTION REWARD ===
        # linear_reward = 3 * action[0]
        # angular_penalty = -1 * abs(action[1]) if abs(orient_to_goal) < 0.3 else 0.0
        # action_reward = linear_reward + angular_penalty
        action_reward = 0.0
        # === WAYPOINT REWARD ===
        waypoint_reward = 0.0
        if hasattr(self, 'waypoints') and hasattr(self, '_last_waypoint_idx'):
            wp_reward, new_idx = self._get_waypoint_reward(self._current_position, self.waypoints, self._last_waypoint_idx)
            waypoint_reward = wp_reward
            self._last_waypoint_idx = new_idx

        # === 4. OBSTACLE AVOIDANCE (multi-level) ===
        obstacle_reward = 0.0
        if min_laser < 2.0:
            obstacle_reward = -1.5 * (2.0 - min_laser) / 2.0
        

        # === 5. DYNAMIC OBSTACLE AVOIDANCE ===
        avoidance_bonus = 0.0
        # if hasattr(self, '_prev_min_laser'):
        #     if min_laser > self._prev_min_laser and min_laser < 2.0:
        #         avoidance_bonus = 3
            
        self._prev_min_laser = min_laser

        # # === 6. EMERGENCY ESCAPE BEHAVIOR ===
        escape_bonus = 0.0
        # if min_laser < 1.0:
        #     if action[0] > 0.1:
        #         escape_bonus += 0.2
        #     escape_bonus += 0.3 * abs(action[1])

        # === 7. STATIONARY PENALTY ===
        stationary_penalty = -0.5 if np.linalg.norm(action[0]) < 0.1 else 0.0

        # === 8. TIME PENALTY (encourage efficiency) ===
        time_penalty = -0.5

        # === COMBINE ALL COMPONENTS ===
        reward = (
            progress_reward +
            orientation_reward +
            action_reward +
            waypoint_reward +
            obstacle_reward +
            avoidance_bonus +
            escape_bonus +
            stationary_penalty +
            time_penalty
        )
        return np.clip(reward, -5.0, 5.0)
    def set_waypoints(self, waypoints):
        """Set the list of waypoints for the environment."""
        self.waypoints = waypoints
        self._last_waypoint_idx = -1

    def _goal_reached(self, dist_to_goal: float) -> bool:
        """Check if final goal is reached (no waypoint logic)."""
        return dist_to_goal < self.goal_threshold

    def _collision(self, min_ranges) -> bool:
        if np.min(min_ranges) < self.collision_threshold: # Use TurtleBot4 threshold
            return True
        return False

    def close(self) -> None:
        self._close_tracker_node()
        self.simulation_control.destroy_node()
        self.ros_gz_pub.destroy_node()
        self.sensors.destroy_node()
        rclpy.try_shutdown()

def main():
    import tb4_drl_navigation.envs # noqa: F401
    from torch.utils.tensorboard import SummaryWriter

    rclpy.init(args=None)
    env = gym.make('Turtlebot4Env-v0', world_name='static_world')
    writer = SummaryWriter(log_dir="tb4_tensorboard_logs")

    try:
        global_step = 0
        num_episodes = 0
        num_success = 0
        num_collision = 0

        while True:
            obs, info = env.reset(options={'debug': True})
            done = False
            episode_success = False
            episode_collision = False

            while not done:
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                writer.add_scalar('Reward/step', reward, global_step)
                global_step += 1

                min_ranges = obs.get('min_ranges', np.array([]))
                min_ranges_angles = obs.get('min_ranges_angle', np.array([]))
                obs_k = obs.get('obs_k', np.array([]))

                # print(f"Step {global_step}: Reward = {reward:.3f}")
                # print('orient_to_goal:', obs.get('orient_to_goal'), '\nInfo: ', info)
                # print('Min range:', np.min(min_ranges) if len(min_ranges) > 0 else 'N/A')
                # print('K-obstacle data shape:', obs_k.shape)
                # print('Social Safety Score:', info.get('social_safety_score', 'N/A'))
                # print('Ego Safety Score:', info.get('ego_safety_score', 'N/A'))

                # if len(min_ranges) > 0:
                #     print('Min range angle:', min_ranges_angles[np.argmin(min_ranges)])
                # else:
                #     print('Min range angle: N/A')

                # if terminated:
                #     # Check if goal was reached or collision occurred
                #     if info.get('ego_safety_score', 1.0) == 1.0 and info.get('social_safety_score', 1.0) == 1.0:
                #         # If both safety scores are perfect, likely a success
                #         episode_success = True
                #     if reward == -200.0 or info.get('timeout_collision', False):
                #         episode_collision = True

                #     print("Episode ended (collision or goal). Resetting environment.")
                #     break

            num_episodes += 1
            if episode_success or info.get('goal_reached', False):
                num_success += 1
            if episode_collision:
                num_collision += 1

            # For demonstration, stop after 100 episodes
            if num_episodes >= 100:
                break

        # Print accuracy at the end
        accuracy = num_success / num_episodes if num_episodes > 0 else 0.0
        print(f"\nEvaluation finished: {num_episodes} episodes")
        print(f"Successes: {num_success}, Collisions: {num_collision}")
        print(f"Accuracy (Success Rate): {accuracy*100:.2f}%")

    except KeyboardInterrupt:
        pass
    finally:
        writer.close()
        env.close()

if __name__ == '__main__':
    main()
