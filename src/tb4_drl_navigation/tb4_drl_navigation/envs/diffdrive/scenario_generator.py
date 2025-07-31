import argparse
import logging
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import KDTree
import yaml


class ScenarioGenerator:
    """
    Generate randomized navigation scenarios from a static occupancy map.

    How it works:

      1. Load a occupancy map image and its YAML metadata (resolution, origin,
         thresholds).
      2. Process the map into “buffered free space” by eroding obstacles
         according to robot radius and clearance.
      3. Build a KD-tree of valid free cells.
      4. Provide `generate_start_goal()` to sample start/goal pairs subject to
         a minimum separation constraint and optional distance bias.
      5. Provide `generate_obstacles()` to place N obstacles at valid locations
         while respecting clearance from start/goal.

    Parameters
    ----------
    map_path : str
        Path to the occupancy map image.
    yaml_path : str
        Path to the corresponding YAML map metadata file.
    robot_radius : float, optional
        Robot's circular radius [m] used for map buffering (default: 0.3).
    min_separation : float, optional
        Minimum start-to-goal distance [m] (default: 2.0).
    obstacle_clearance : float, optional
        Clearance [m] required around any obstacle (default: 1.5).

    Examples
    --------
    >>> gen = ScenarioGenerator(
    ...     map_path='maps/static_world.pgm',
    ...     yaml_path='maps/static_world.yaml',
    ...     robot_radius=0.4,
    ...     min_separation=4.0,
    ...     obstacle_clearance=2.0
    ... )
    >>> start, goal = gen.generate_start_goal(
    ...     max_attempts=50, goal_sampling_bias='uniform', eps=1e-4
    ... )
    >>> obstacles = gen.generate_obstacles(
    ...     num_obstacles=10, start_pos=start, goal_pos=goal
    ... )

    """

    def __init__(
            self,
            map_path: Path,
            yaml_path: Path,
            robot_radius: float = 0.3,
            min_separation: float = 2.0,
            obstacle_clearance: float = 1.5,
            seed: Optional[int] = None,
    ):
        """
        Initialize scenario generator.

        Parameters
        ----------
        map_path : Path
            Path to the occupancy map image file
        yaml_path : Path
            Path to the map metadata YAML file
        robot_radius : float
            Robot's circular radius [m]
        min_sep : float
            Minimum start-to-goal separation [m]
        obs_clear : float
            Clearance required around obstacles [m]
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self._validate_inputs(
            map_path, yaml_path, robot_radius, min_separation, obstacle_clearance
        )
        self.map_path = map_path
        self.yaml_path = yaml_path
        self.robot_radius = robot_radius
        self.min_separation = min_separation
        self.obstacle_clearance = obstacle_clearance

        self.seed = seed
        self._rng = np.random.default_rng(seed=self.seed)

        self.metadata = self._load_metadata()
        self.origin = self._parse_origin()
        self.map_img, self.processed_map = self._process_map()
        self.free_cells = self._get_free_cells()
        self.kdtree = KDTree(self.free_cells)

    @staticmethod
    def _validate_inputs(
        map_path: Path,
        yaml_path: Path,
        robot_radius: float,
        min_sep: float,
        obs_clear: float
    ) -> None:
        """
        Validate scenario generator inputs.

        Parameters
        ----------
        map_path : Path
            Path to the occupancy map image file
        yaml_path : Path
            Path to the map metadata YAML file
        robot_radius : float
            Robot's circular radius [m]
        min_sep : float
            Minimum start-to-goal separation [m]
        obs_clear : float
            Clearance required around obstacles [m]

        Raises
        ------
        FileNotFoundError
            If either map file is not found
        ValueError
            If any clearance/distance parameter is not positive
        """
        if not map_path.exists():
            raise FileNotFoundError(f'Map file {map_path} not found')
        if not yaml_path.exists():
            raise FileNotFoundError(f'Map metadata file {yaml_path} not found')
        if any(val <= 0 for val in [robot_radius, min_sep, obs_clear]):
            raise ValueError('All clearance/distance parameters must be positive')

    def _load_metadata(self) -> dict:
        """Load and validate map metadata."""
        with open(self.yaml_path) as f:
            metadata = yaml.safe_load(f)

        required_keys = {'resolution', 'origin', 'occupied_thresh', 'free_thresh'}
        if not required_keys.issubset(metadata):
            missing = required_keys - metadata.keys()
            raise ValueError(f'Missing required YAML keys: {missing}')

        return metadata

    def _parse_origin(self) -> Tuple[float, float, float]:
        """Convert origin to (x, y, yaw)."""
        # TODO: is there a slam pkg that provide origin as pos + q?
        origin = self.metadata['origin']
        if len(origin) == 3:
            return (origin[0], origin[1], origin[2])
        else:
            raise ValueError('Invalid origin format')

    def _process_map(self) -> Tuple[np.ndarray, np.ndarray]:
        """Process map image with morphological operations."""
        # Load map image
        map_img = cv2.imread(str(self.map_path), cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            raise FileNotFoundError(f'Failed to load map image from {self.map_path}')

        # Apply thresholds to identify free space
        free_thresh = self.metadata['free_thresh']
        free_thresh_pixel = (1.0 - free_thresh) * 255
        free_mask = map_img > free_thresh_pixel

        # Check if any free cells are present
        if not np.any(free_mask):
            raise ValueError('No free cells found in the map.')

        # Convert to uint8 (0 and 255)
        free_mask_uint8 = np.where(free_mask, 255, 0).astype(np.uint8)

        # Calculate buffer in pixels
        resolution = self.metadata['resolution']
        # The buffer for the map processing should consider both robot radius and a small safety margin
        # to ensure the robot stays away from walls
        buffer_distance = self.robot_radius * 1.5  # 20% extra margin
        buffer_pixels = int(np.ceil(buffer_distance / resolution))

        if buffer_pixels <= 0:
            self.logger.warning('Buffer pixels is zero. No erosion applied.')
            processed_map = free_mask.astype(int)
        else:
            # Create circular kernel for erosion
            kernel_size = buffer_pixels * 2 + 1 # Ensures an odd kernel size for proper centering
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )

            # Erode the free areas to create buffer
            eroded_free = cv2.erode(free_mask_uint8, kernel, iterations=1)
            processed_map = (eroded_free == 255).astype(int)

        # Validate processed map
        if np.sum(processed_map) == 0:
            raise ValueError(
                'Processed map has no navigable areas. '
                'Check robot_radius parameter and map size.' # Adjusted message
            )

        return map_img, processed_map

    def _get_free_cells(self) -> np.ndarray:
        """Get array of valid (row, col) positions from the processed map.
        
        The processed map already accounts for:
        1. Wall boundaries (via map image)
        2. Robot radius (via erosion)
        3. Additional safety margin
        """
        # Get indices where processed_map == 1 (completely free space)
        free_cell_indices = np.where(self.processed_map == 1)
        
        # Stack into (N,2) array of (row,col) coordinates
        free_cells = np.column_stack(free_cell_indices)
        
        # Add padding from edges of free space for extra safety
        # Use a more conservative margin (minimum 2 pixels, or more based on robot radius)
        height, width = self.processed_map.shape
        safety_margin = max(2.5, int(self.robot_radius / self.metadata['resolution'] * 0.5))
        
        self.logger.info(f"Using safety margin of {safety_margin} pixels from walls")
        
        edge_mask = (
            (free_cells[:, 0] > safety_margin) &  # Not too close to top
            (free_cells[:, 0] < height - safety_margin) &  # Not too close to bottom
            (free_cells[:, 1] > safety_margin) &  # Not too close to left
            (free_cells[:, 1] < width - safety_margin)  # Not too close to right
        )
        free_cells = free_cells[edge_mask]
        
        if len(free_cells) == 0:
            raise ValueError(
                'No valid free cells found after processing. '
                'Try reducing robot_radius or check if map is valid.'
            )
            
        return free_cells

    def generate_start_goal(
            self,
            max_attempts: int = 100,
            goal_sampling_bias: str = 'uniform',
            eps: float = 1e-5,
            potential_obstacles: Optional[List[Tuple[float, float]]] = None
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Sample valid (start, goal) pairs in free space, avoiding obstacles.
        Ensures both are in free space and separated by at least min_separation.
        Also ensures the goal is not too close to any obstacles.
        """
        for _ in range(max_attempts):
            # Sample start position from the processed map's free cells
            start_idx = self._rng.choice(len(self.free_cells))
            start_cell = self.free_cells[start_idx]
            
            # Verify that the start position is in free space according to the processed map
            if not self.processed_map[start_cell[0], start_cell[1]]:
                continue

            start_pos_world = self.map_to_world(start_cell)

            # Find cells that are far enough from start
            distances, indices = self.kdtree.query(
                [start_cell], k=len(self.free_cells), return_distance=True
            )
            distances = distances.squeeze()
            indices = indices.squeeze()

            # Filter cells by minimum separation and processed map
            valid_mask = distances >= self.min_separation / self.metadata['resolution']
            valid_indices = indices[valid_mask]

            if not valid_indices.size > 0:
                continue

            # Try all valid goal candidates, shuffle for randomness
            shuffled_goal_indices = self._rng.permutation(valid_indices)
            
            # Ensure obstacles is a list
            obstacles = potential_obstacles if potential_obstacles is not None else []
            
            # Add extra safety margin (50% more) for obstacle clearance during goal selection
            safety_factor = 1.5
            safe_distance = (self.robot_radius + self.obstacle_clearance) * safety_factor
            
            # Add extra safety margin from walls (50% more than robot radius)
            wall_safety_margin = self.robot_radius * 1.5
            wall_safety_pixels = int(np.ceil(wall_safety_margin / self.metadata['resolution']))
            height, width = self.processed_map.shape
            for goal_idx in shuffled_goal_indices:
                goal_cell = self.free_cells[goal_idx]
                
                # Double check goal is in processed free space
                if not self.processed_map[goal_cell[0], goal_cell[1]]:
                    continue
                    
                # Extra check for wall clearance - reject goals too close to walls
                if (goal_cell[0] < wall_safety_pixels or 
                    goal_cell[0] >= height - wall_safety_pixels or
                    goal_cell[1] < wall_safety_pixels or 
                    goal_cell[1] >= width - wall_safety_pixels):
                    continue

                goal_pos_world = self.map_to_world(goal_cell)

                # Check that the goal is not too close to any obstacles
                too_close = False
                if obstacles:
                    for obs in obstacles:
                        # Calculate actual distance in world units for more precision
                        dist = np.sqrt((obs[0] - goal_pos_world[0])**2 + (obs[1] - goal_pos_world[1])**2)
                        if dist < safe_distance:
                            too_close = True
                            break
                            
                if too_close:
                    continue
                    
                # All checks passed, we have a valid goal!
                return start_pos_world, goal_pos_world

        raise RuntimeError(f'Failed to sample valid start/goal after {max_attempts} attempts')

    def generate_obstacles(
            self,
            num_obstacles: int,
            start_pos: Tuple[float, float],
            goal_pos: Optional[Tuple[float, float]] = None
    ) -> List[Tuple[float, float]]:
        """Generate obstacle positions with clearance requirements."""
        resolution = self.metadata['resolution']
        # The clearance for placing obstacles.
        # This means the center of the obstacle must be this far from start/goal
        # and other obstacles. It should be obstacle_clearance + robot_radius to be safe.
        min_px_clear_from_start_goal = (self.obstacle_clearance + self.robot_radius) / resolution
        min_px_clear_between_obstacles = (2 * self.obstacle_clearance) / resolution # Double for distance between centers

        # Convert positions to map cells
        start_cell = self.world_to_map(start_pos)
        goal_cell = self.world_to_map(goal_pos) if goal_pos is not None else None

        # Filter candidate cells - Ensure candidates are far from walls (handled by processed_map)
        # And far from start/goal
        start_distances = np.linalg.norm(self.free_cells - start_cell, axis=1)
        if goal_cell is not None:
            goal_distances = np.linalg.norm(self.free_cells - goal_cell, axis=1)
            valid_mask = (start_distances > min_px_clear_from_start_goal) & \
                        (goal_distances > min_px_clear_from_start_goal)
        else:
            valid_mask = start_distances > min_px_clear_from_start_goal
        candidates = self.free_cells[valid_mask]

        if len(candidates) < num_obstacles:
            self.logger.warning(
                f'Requested {num_obstacles} obstacles, only {len(candidates)} valid positions. '
                f'Placing {len(candidates)} obstacles.'
            )

        # KDTree for clearance checking for obstacles against each other
        kdtree_candidates = KDTree(candidates) # KDTree of all potential candidate spots
        obstacle_positions = [] # In map cells initially
        
        # Keep track of indices in 'candidates' array that are still available
        remaining_candidate_indices = np.arange(len(candidates))

        for _ in range(min(num_obstacles, len(candidates))):
            if len(remaining_candidate_indices) == 0:
                break
            # Randomly select from remaining *available* candidates
            chosen_candidate_idx_in_remaining = self._rng.choice(len(remaining_candidate_indices))
            actual_idx_in_candidates = remaining_candidate_indices[chosen_candidate_idx_in_remaining]
            selected_cell = candidates[actual_idx_in_candidates]
            
            obstacle_positions.append(selected_cell)

            # Find all neighbors within the required clearance from the *newly selected* obstacle
            # These neighbors, including the selected_cell itself, must be removed from future consideration
            # We query on the 'candidates' array, not 'self.free_cells'
            nearby_candidate_indices_mask = kdtree_candidates.query_radius([selected_cell], r=min_px_clear_between_obstacles)[0]
            
            # Convert these indices (which are relative to the 'candidates' array)
            # to indices within the 'remaining_candidate_indices' list
            # This is tricky because `remaining_candidate_indices` stores the original indices in `candidates`
            # The simplest way is to convert `nearby_candidate_indices_mask` to actual values
            # and then use setdiff1d to filter `remaining_candidate_indices`
            
            # Get the actual candidate cells that are too close
            cells_to_remove = candidates[nearby_candidate_indices_mask]
            
            # Find which elements of `remaining_candidate_indices` correspond to `cells_to_remove`
            # This is less efficient but robust. For very large maps, a more direct indexing might be needed.
            current_remaining_cells = candidates[remaining_candidate_indices]
            
            # Create a boolean mask for items in `current_remaining_cells` that are *not* in `cells_to_remove`
            # More efficiently, find the indices to remove from `remaining_candidate_indices`
            
            # Option 1: Convert cells_to_remove back to indices in `candidates` and filter `remaining_candidate_indices`
            # This requires knowing which `nearby_candidate_indices_mask` map to which `actual_idx_in_candidates`
            
            # The original logic `np.setdiff1d(remaining_indices, neighbors[0])` was almost correct,
            # but `neighbors[0]` returns indices relative to the KDTree *input*, which is `candidates`.
            # So, `remaining_indices` itself should store indices relative to `candidates`.

            # Let's fix this part to be more robust.
            # `remaining_indices` should be the indices within the `candidates` array.
            
            # For simplicity, let's rebuild the `remaining_indices`
            # This means `remaining_candidate_indices` should directly map to `candidates` indices.
            
            # `nearby_candidate_indices_mask` are indices directly into `candidates`.
            # We need to remove these from `remaining_candidate_indices`.
            # A more direct way using boolean masking is to filter `candidates` directly,
            # then rebuild the KDTree, but that's less efficient.
            
            # The current KDTree logic in generate_obstacles for `remaining_indices` is tricky.
            # `remaining_indices` should hold the indices *into the original `candidates` array*
            # of the cells that are still available.
            # `neighbors[0]` gives indices in `candidates`.
            
            # Correct approach for updating `remaining_indices`:
            # `remaining_indices` holds indices into `candidates`.
            # `neighbors[0]` holds indices into `candidates`.
            # We want `remaining_indices = remaining_indices` where element is NOT in `neighbors[0]`
            remaining_candidate_indices = np.array([
                idx for idx in remaining_candidate_indices if idx not in nearby_candidate_indices_mask
            ])
            # This conversion to list and back to array can be slow for very many candidates.
            # For potentially large arrays, a boolean mask might be faster:
            # Create a boolean mask where `True` means keep the index
            # keep_mask = np.ones(len(remaining_candidate_indices), dtype=bool)
            # for idx_to_remove in nearby_candidate_indices_mask:
            #    # Find where `remaining_candidate_indices` contains `idx_to_remove`
            #    # This can be slow if `remaining_candidate_indices` is large.
            #    # Consider a reverse mapping or a different data structure for `remaining_indices`
            #    # A set is better for `nearby_candidate_indices_mask` if it's large.
            
            # For typical DRL map sizes, the current approach with setdiff1d (which is what you had)
            # or the list comprehension for `remaining_indices` should be fine.
            
            # Let's revert to your original, which is usually correct for set operations
            # `np.setdiff1d` takes two arrays and returns the unique values in arr1 that are not in arr2.
            # `neighbors[0]` is an array of indices in `candidates` that are too close.
            # `remaining_indices` should be indices in `candidates` that are still available.
            # So, `np.setdiff1d(remaining_indices, neighbors[0])` is indeed the correct way to update it.
            # Your original code for this line was correct:
            # remaining_indices = np.setdiff1d(remaining_indices, neighbors[0])


        return [self.map_to_world(cell) for cell in obstacle_positions]

    def map_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert map cell (row, col) to world coordinates (x, y)."""
        resolution = self.metadata['resolution']
        x_map = cell[1] * resolution  # Column to X
        y_map = cell[0] * resolution  # Row to Y

        # Apply origin transformation
        x = self.origin[0] + x_map * np.cos(self.origin[2]) - y_map * np.sin(self.origin[2])
        y = self.origin[1] + x_map * np.sin(self.origin[2]) + y_map * np.cos(self.origin[2])

        return x, y

    def world_to_map(self, pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates (x, y) to map cell (row, col)."""
        resolution = self.metadata['resolution']
        dx = pos[0] - self.origin[0]
        dy = pos[1] - self.origin[1]

        # Inverse rotation
        rot_x = dx * np.cos(-self.origin[2]) - dy * np.sin(-self.origin[2])
        rot_y = dx * np.sin(-self.origin[2]) + dy * np.cos(-self.origin[2])

        col = int(rot_x / resolution)
        row = int(rot_y / resolution)

        return row, col

    def plot_debug(
            self,
            start_pos: Optional[Tuple[float, float]] = None,
            goal_pos: Optional[Tuple[float, float]] = None,
            obstacles: Optional[List[Tuple[float, float]]] = None
    ) -> None:
        """Visualize map with positions."""
        # Set up fonts
        plt.rcParams.update({
            'font.family': 'sans-serif',
            'font.sans-serif': ['DejaVu Sans', 'Liberation Sans', 'Arial'],
            'mathtext.fontset': 'cm',
            'font.size': 12,
            'axes.labelsize': 14,
            'axes.titlesize': 16,
            'legend.fontsize': 12,
            'xtick.labelsize': 12,
            'ytick.labelsize': 12,
            'text.usetex': False
        })

        plt.figure(figsize=(12, 12))

        # Extent
        extent = self._get_map_extent()
        # Original map background
        plt.imshow(
            self.map_img,
            cmap='binary',
            extent=extent,
            alpha=0.6,
            zorder=1,
        )

        # Processed free space overlay
        free_mask = np.ma.masked_where(self.processed_map == 0, self.processed_map)
        plt.imshow(
            free_mask,
            cmap='Greens',
            alpha=0.3,
            extent=extent,
            zorder=2,
        )

        # Clearance Zones
        clearance_kwargs = {
            'fill': True,
            'alpha': 0.2,
            'linestyle': '--',
            'linewidth': 2,
        }

        # Start clearance (robot_radius + obstacle_clearance)
        if start_pos:
            plt.gca().add_patch(
                plt.Circle(
                    start_pos,
                    self.robot_radius + self.obstacle_clearance, # Visualizing combined clearance
                    color='red',
                    label='Start Clearance (Robot+Obstacles)',
                    **clearance_kwargs,
                )
            )

        # Goal clearance (robot_radius + obstacle_clearance)
        if goal_pos:
            plt.gca().add_patch(
                plt.Circle(
                    goal_pos,
                    self.robot_radius + self.obstacle_clearance, # Visualizing combined clearance
                    color='blue',
                    label='Goal Clearance (Robot+Obstacles)',
                    **clearance_kwargs,
                )
            )

        # Obstacle clearances (obstacle_clearance) -- Or maybe 2 * self.obstacle_clearance for between obstacles
        # For simplicity of visualization, let's just plot obstacle_clearance around each obstacle point
        # if obstacles:
        #     for idx, obstacle in enumerate(obstacles):
        #         plt.gca().add_patch(
        #             plt.Circle(
        #                 obstacle,
        #                 self.obstacle_clearance, # This is the clearance *around* the obstacle
        #                 color='orange',
        #                 label='Obstacle Zone' if idx == 0 else None,
        #                 **clearance_kwargs,
        #             )
        #         )
        # Instead, let's plot the robot_radius + obstacle_clearance from obstacle centers
        if obstacles:
            for idx, obstacle in enumerate(obstacles):
                plt.gca().add_patch(
                    plt.Circle(
                        obstacle,
                        self.obstacle_clearance, # This is the radius of the physical obstacle model
                        color='orange',
                        label='Obstacle Physical' if idx == 0 else None,
                        fill=True, alpha=0.5, zorder=3
                    )
                )
                plt.gca().add_patch(
                    plt.Circle(
                        obstacle,
                        self.obstacle_clearance + self.robot_radius, # The zone the robot cannot enter
                        color='purple',
                        label='Obstacle Buffer Zone' if idx == 0 else None,
                        fill=False, linestyle=':', linewidth=1.5, zorder=2
                    )
                )


        # Position Markers
        marker_style = {
            's': 300,
            'edgecolors': 'black',
            'linewidths': 2,
            'zorder': 4,
        }

        if start_pos:
            plt.scatter(
                *start_pos,
                marker='D',
                c='lime',
                label='Start Position',
                **marker_style,
            )

        if goal_pos:
            plt.scatter(
                *goal_pos,
                marker='*',
                c='gold',
                label='Goal Position',
                **marker_style,
            )

        if obstacles:
            obs_x, obs_y = zip(*obstacles) if obstacles else ([], [])
            plt.scatter(
                obs_x,
                obs_y,
                marker='X',
                c='darkred',
                s=150,
                edgecolors='white',
                label='Obstacles Center',
                zorder=5, # Higher zorder to be on top of circles
            )

        # Add some final touches
        # plt.colorbar(label="Map Intensity", fraction=0.03, pad=0.01)
        plt.xlabel('World X [m]', fontsize=12)
        plt.ylabel('World Y [m]', fontsize=12)
        plt.title('Navigation Scenario Debug View', fontsize=14, pad=20)

        # Configure grid and background
        plt.grid(True, color='white', alpha=0.3, linestyle='--')
        plt.gca().set_facecolor('lightcyan')

        # Legend handling
        handles, labels = plt.gca().get_legend_handles_labels()
        unique_labels = dict(zip(labels, handles))
        plt.legend(
            unique_labels.values(),
            unique_labels.keys(),
            loc='upper left',
            bbox_to_anchor=(1.01, 1.0),
            fontsize=10,
            framealpha=0.95,
            facecolor='white',
            edgecolor='black',
            borderpad=1.5,
            title=r'Legend:',
            title_fontsize=11,
            handletextpad=2.0,
            handlelength=2.0,
            labelspacing=1.25,
            markerscale=0.8
        )
        # plt.subplots_adjust(right=0.78)

        plt.tight_layout()
        plt.show()

    def _get_map_extent(self) -> List[float]:
        """Calculate world coordinate extent."""
        height, width = self.map_img.shape
        bl = self.map_to_world((height - 1, 0))  # Bottom-left
        tr = self.map_to_world((0, width - 1))   # Top-right
        return [bl[0], tr[0], bl[1], tr[1]]

# --- NEW HELPER FUNCTION for the main loop ---
def _is_position_too_close_to_obstacles(
    position: Tuple[float, float],
    obstacles: List[Tuple[float, float]],
    clearance_radius: float # typically robot_radius + obstacle_clearance
) -> bool:
    """Checks if a given position is too close to any of the obstacles."""
    if not obstacles:
        return False
    
    # Convert obstacles to numpy array for KDTree
    obs_array = np.array(obstacles)
    
    # Build a KDTree for the current obstacles
    obs_kdtree = KDTree(obs_array)
    
    # Query the KDTree for the position
    dists, _ = obs_kdtree.query(np.array([position]), k=1, return_distance=True)
    
    # Check if the closest distance is less than the required clearance
    return dists.item() < clearance_radius

def main():
    logging.basicConfig(level=logging.INFO)

    current_dir = Path(__file__).parent
    default_map = current_dir / 'maps' / 'static_world.pgm'
    default_yaml = current_dir / 'maps' / 'static_world.yaml'

    parser = argparse.ArgumentParser(
        description='Load and process a static world map and its metadata YAML.'
    )
    parser.add_argument(
        '-m', '--map',
        type=Path,
        default=default_map,
        help=f'Path to the PGM map file (default: {default_map})'
    )
    parser.add_argument(
        '-y', '--yaml',
        type=Path,
        default=default_yaml,
        help=f'Path to the YAML metadata file (default: {default_yaml})'
    )
    parser.add_argument(
        '--num_obstacles',
        type=int,
        default=12,
        help='Number of dynamic obstacles to place.'
    )
    parser.add_argument(
        '--max_scenario_attempts',
        type=int,
        default=50,
        help='Maximum attempts to generate a valid scenario (start, goal, obstacles).'
    )

    args = parser.parse_args()

    try:
        # Instantiate ScenarioGenerator
        nav_scenario = ScenarioGenerator(
            map_path=args.map,
            yaml_path=args.yaml,
            robot_radius=0.35,  # Increased from 0.3 for more safety
            min_separation=4,  # Increased from 2.0 for better start-goal distance
            obstacle_clearance=0.4,  # Increased from 0.5 for safer obstacle spacing
            seed=None,
        )
        
        # --- Iterative Scenario Generation ---
        start, goal, obstacles = None, None, None
        scenario_generated = False
        for attempt in range(args.max_scenario_attempts):
            logging.info(f"Attempting to generate scenario {attempt + 1}/{args.max_scenario_attempts}...")
            try:
                # 1. Generate start position first
                # Ensure we're sampling from cells that are well inside free space
                # by using the processed map which already accounts for robot radius
                valid_start_cells = nav_scenario.free_cells
                
                # For extra safety, we can filter cells that are too close to the edges
                # of free space by checking for a larger neighborhood of free cells
                buffer_distance = 2 * nav_scenario.robot_radius  # Extra buffer for safety
                buffer_pixels = int(np.ceil(buffer_distance / nav_scenario.metadata['resolution']))
                
                if buffer_pixels > 0 and len(valid_start_cells) > 100:  # Only apply if we have enough free cells
                    # Use more central cells for spawning to avoid being too close to walls
                    height, width = nav_scenario.processed_map.shape
                    edge_mask = (
                        (valid_start_cells[:, 0] > buffer_pixels) &  # Not too close to top
                        (valid_start_cells[:, 0] < height - buffer_pixels) &  # Not too close to bottom
                        (valid_start_cells[:, 1] > buffer_pixels) &  # Not too close to left
                        (valid_start_cells[:, 1] < width - buffer_pixels)  # Not too close to right
                    )
                    safer_start_cells = valid_start_cells[edge_mask]
                    
                    # Only use the filtered cells if we have enough left
                    if len(safer_start_cells) > 10:
                        valid_start_cells = safer_start_cells
                
                start_cell_idx = nav_scenario._rng.choice(len(valid_start_cells))
                start = nav_scenario.map_to_world(valid_start_cells[start_cell_idx])
                
                # Log the selected start position
                nav_scenario.logger.info(f'Selected start position: {start}')
                
                # 2. Generate obstacles (keeping clear of start position)
                obstacles = nav_scenario.generate_obstacles(
                    num_obstacles=args.num_obstacles,
                    start_pos=start
                )
                
                # 3. Generate goal, passing the existing obstacles
                # Use more attempts for finding a safe goal position
                start, goal = nav_scenario.generate_start_goal(
                    max_attempts=200,  # Increase attempts to find a better goal
                    goal_sampling_bias='uniform',
                    potential_obstacles=obstacles  # Pass existing obstacles
                )
                
                # Extra safety check for goal position
                combined_clearance = nav_scenario.robot_radius + nav_scenario.obstacle_clearance
                if _is_position_too_close_to_obstacles(goal, obstacles, combined_clearance * 1.5):  # 50% extra margin
                    nav_scenario.logger.warning("Goal position is too close to obstacles, trying again...")
                    continue
                
                # Additional check for wall proximity using the processed map
                goal_cell = nav_scenario.world_to_map(goal)
                height, width = nav_scenario.processed_map.shape
                wall_margin = int(nav_scenario.robot_radius * 1.5 / nav_scenario.metadata['resolution'])
                
                if (goal_cell[0] < wall_margin or 
                    goal_cell[0] >= height - wall_margin or
                    goal_cell[1] < wall_margin or 
                    goal_cell[1] >= width - wall_margin):
                    nav_scenario.logger.warning("Goal position is too close to walls, trying again...")
                    continue
                
                # Check if there's enough free space around the goal (at least robot radius * 2)
                safety_radius = nav_scenario.robot_radius * 2
                safety_pixels = int(np.ceil(safety_radius / nav_scenario.metadata['resolution']))
                
                # Extract a region around the goal cell
                row_min = max(0, goal_cell[0] - safety_pixels)
                row_max = min(height, goal_cell[0] + safety_pixels + 1)
                col_min = max(0, goal_cell[1] - safety_pixels)
                col_max = min(width, goal_cell[1] + safety_pixels + 1)
                
                goal_region = nav_scenario.processed_map[row_min:row_max, col_min:col_max]
                free_ratio = np.sum(goal_region) / goal_region.size
                
                if free_ratio < 0.9:  # At least 90% of surrounding area should be free
                    nav_scenario.logger.warning(f"Goal doesn't have enough free space around it (free ratio: {free_ratio:.2f}), trying again...")
                    continue
                
                # Goal position is valid
                scenario_generated = True
                break  # Valid scenario found!

            except RuntimeError as e: # Catch errors from generate_start_goal (e.g., no valid pair)
                logging.warning(f"Scenario generation failed in attempt {attempt + 1}: {e}. Retrying.")
                continue

        if not scenario_generated:
            raise RuntimeError(f"Failed to generate a valid scenario after {args.max_scenario_attempts} attempts.")


        nav_scenario.plot_debug(
            start_pos=start, goal_pos=goal, obstacles=obstacles
        )

        nav_scenario.logger.info(f'Start: {start}')
        nav_scenario.logger.info(f'Goal: {goal}')
        nav_scenario.logger.info(f'Obstacles: {obstacles}')

    except Exception as e:
        logging.error(f'Error: {e}')
        raise


if __name__ == '__main__':
    main()
