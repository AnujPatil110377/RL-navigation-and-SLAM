import argparse
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
import random
import sys
from typing import Any, Dict, List, Optional, Tuple, Type

import gymnasium as gym
import numpy as np

import rclpy
# Import all the algorithms you want to use
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise
import tb4_drl_navigation.envs  # noqa: F401
import torch
import torch.nn as nn
import yaml

# --- Environment Configuration ---
@dataclass(frozen=True)
class EnvConfig:
    """Configuration for the Gymnasium environment."""
    env_id: str = 'Turtlebot4Env-v0'
    world_name: str = 'static_world'
    robot_name: str = 'turtlebot4'
    obstacle_prefix: str = 'obstacle'
    robot_radius: float = 0.3
    obstacle_clearance: float = 3.0
    min_separation: float = 3.0
    goal_sampling_bias: str = 'uniform'
    num_bins: int = 40
    goal_threshold: float = 0.45
    collision_threshold: float = 0.325
    time_delta: float = 0.1
    shuffle_on_reset: bool = False
    map_path: Optional[Path] = None
    yaml_path: Optional[Path] = None
    sim_launch_name: Optional[Path] = None

# --- Algorithm-Specific Configurations ---

# --- Custom SAC with Gradient Clipping ---

from stable_baselines3 import SAC
from stable_baselines3.sac.sac import polyak_update
import torch

class SACWithGradClip(SAC):

 pass    # Use default SAC.train() method (no gradient clipping)
@dataclass(frozen=True)
class SACConfig:
    """Configuration for the SAC algorithm."""
    # Algorithm
    policy_type: str = 'MlpPolicy'
    buffer_size: int = 1_000_000
    batch_size: int = 256
    gamma: float = 0.99
    tau: float = 0.001
    target_update_interval: int = 2
    learning_rate: float = 3e-4
    train_freq: Tuple[int, str] = (1, 'step')
    gradient_steps: int = 1
    # ent_coef: Any = "auto_0.05"

    # Network
    # policy_kwargs: Dict[str, Any] = field(
    #     default_factory=lambda: {
    #         # Deeper but reasonable network for SAC
    #         'net_arch': {'pi': [512, 512, 256, 128], 'qf': [1024, 1024, 512, 256]},
    #         'activation_fn': nn.ReLU,
    #         'net_arch': {'pi': [512, 512, 256, 128], 'qf': [1024, 1024, 512, 256]},
    #         'activation_fn': nn.ReLU,
    #     }
    # )

    policy_kwargs: Dict[str, Any] = field(
        default_factory=lambda: {
            # Simpler network for SAC to prevent gradient explosion
            'net_arch': {
                'pi': [256, 256],  # Actor network
                'qf': [256, 256]   # Critic network
            },
            'activation_fn': nn.ReLU,
            'normalize_images': False,
        }
    )
    
@dataclass(frozen=True)
class PPOConfig:
    """Configuration for the PPO algorithm."""
    # Algorithm
    policy_type: str = 'MlpPolicy'
    n_steps: int = 2048
    batch_size: int = 64
    n_epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    ent_coef: float = 0.0
    vf_coef: float = 0.5
    learning_rate: float = 1e-4

    # Network
    policy_kwargs: Dict[str, Any] = field(
        default_factory=lambda: {
            # Deeper but reasonable network for PPO (deeper than before)
        }
    )

@dataclass(frozen=True)
class TD3Config:
    """Configuration for the TD3 algorithm."""
    policy_type: str = 'MlpPolicy'
    buffer_size: int = 1_000_000
    batch_size: int = 256
    gamma: float = 0.99
    tau: float = 0.005
    policy_delay: int = 2
    learning_rate: float = 1e-4
    learning_starts: int = 15000  # Increased for better initial exploration
    target_policy_noise: float = 0.2  # Added noise to target actions
    target_noise_clip: float = 0.5    # Clipping parameter for target policy noise
    action_noise_mean: float = 0.0
    action_noise_std: float = 0.1     # Increased for better exploration
    policy_kwargs: Dict[str, Any] = field(
        default_factory=lambda: {
            # Deeper but reasonable network for TD3
            'net_arch': {'pi': [512, 512, 256, 128], 'qf': [512, 512, 256, 128]},
            'activation_fn': nn.ReLU,
        }
    )

# --- Main Experiment Configuration ---
@dataclass
class ExperimentConfig:
    """Master configuration for the experiment."""
    env: EnvConfig = field(default_factory=EnvConfig)
    
    # Algorithm configs
    ppo: PPOConfig = field(default_factory=PPOConfig)
    sac: SACConfig = field(default_factory=SACConfig)
    td3: TD3Config = field(default_factory=TD3Config)

    # General training parameters
    algo: str = 'ppo'  # Default algorithm
    total_timesteps: int = 1_000_000
    save_freq: int = 10000
    eval_freq: int = 10000
    n_eval_episodes: int = 10
    seed: Optional[int] = 42
    use_deterministic_cudnn: bool = False

    log_dir: Path = Path('experiments')
    
    @property
    def experiment_name(self) -> str:
        """Generates the experiment name based on the chosen algorithm."""
        return f"{self.algo}_navigation"

# --- MAPPING ---
# Maps algorithm names to their respective classes and configs
ALGO_MAP = {
    'ppo': (PPO, PPOConfig),
    'sac': (SACWithGradClip, SACConfig),
    'td3': (TD3, TD3Config),
}

# --- Environment Creation ---
def make_env(config: ExperimentConfig) -> gym.Env:
    """Initializes the ROS node and creates the Gym environment."""
    if not rclpy.ok():
        rclpy.init(args=None)
    env_params = asdict(config.env)
    env = gym.make(env_params.pop('env_id'), **env_params)
    env = gym.wrappers.FlattenObservation(env)
    # The Monitor wrapper records episode statistics
    monitor_path = config.log_dir / config.experiment_name / 'monitor.csv'
    monitor_path.parent.mkdir(parents=True, exist_ok=True)
    return Monitor(env, filename=str(monitor_path))

# --- Callback for Adapting TD3 Exploration Noise ---
class AdaptiveExplorationCallback(BaseCallback):
    """Callback for adapting TD3's exploration noise over time."""
    def __init__(
        self,
        start_noise: float = 0.3,
        end_noise: float = 0.05,
        decay_steps: int = 800000,
        verbose: int = 0
    ):
        super().__init__(verbose)
        self.start_noise = start_noise
        self.end_noise = end_noise
        self.decay_steps = decay_steps
        self.current_noise = start_noise
    def _on_step(self) -> bool:
        progress = min(1.0, self.num_timesteps / self.decay_steps)
        new_noise = self.start_noise - progress * (self.start_noise - self.end_noise)
        if hasattr(self.model, 'action_noise') and self.model.action_noise is not None:
            action_dim = self.model.action_space.shape[-1]
            self.model.action_noise.sigma = np.full(action_dim, new_noise)
            if self.verbose > 1 and self.num_timesteps % 10000 == 0:
                print(f"Exploration noise updated: {new_noise:.3f}")
        return True

# --- Unified Experiment Class ---
class Experiment:
    """A unified class to run training experiments for PPO, SAC, or TD3."""
    def __init__(self, config: ExperimentConfig):
        self.config = config
        self.env = make_env(config)

        # Create experiment paths
        self.experiment_path = self.config.log_dir / self.config.experiment_name
        self.checkpoints_path = self.experiment_path / 'checkpoints'
        self.best_model_path = self.experiment_path / 'best_model'
        self.logs_path = self.experiment_path / 'logs'
        self.eval_logs_path = self.experiment_path / 'eval_logs'
        self.final_model_path = self.experiment_path / 'final_model.zip'

        self.experiment_path.mkdir(parents=True, exist_ok=True)
        self.checkpoints_path.mkdir(exist_ok=True)
        self.best_model_path.mkdir(exist_ok=True)
        self.logs_path.mkdir(exist_ok=True)
        self.eval_logs_path.mkdir(exist_ok=True)

        self._set_seeds()
        self._setup_model()

    def _set_seeds(self):
        """Sets random seeds for reproducibility."""
        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)
            torch.manual_seed(self.config.seed)
            if self.config.use_deterministic_cudnn:
                torch.backends.cudnn.deterministic = True

    def _setup_model(self):
        """Initializes the correct SB3 model based on the config."""
        algo_name = self.config.algo
        if algo_name not in ALGO_MAP:
            raise ValueError(f"Unknown algorithm: {algo_name}. Available: {list(ALGO_MAP.keys())}")

        model_class, _ = ALGO_MAP[algo_name]

        # Get the specific algorithm's configuration from the master config
        algo_params = asdict(getattr(self.config, algo_name))

        # Special handling for TD3 action noise
        if algo_name == 'td3':
            action_dim = self.env.action_space.shape[-1]
            algo_params['action_noise'] = NormalActionNoise(
                mean=np.full(action_dim, algo_params.pop('action_noise_mean')),
                sigma=np.full(action_dim, algo_params.pop('action_noise_std'))
            )


        # Remove grad_clip_norm from kwargs for SAC, set as attribute after instantiation
        grad_clip_norm = None
        if algo_name == 'sac' and 'grad_clip_norm' in algo_params:
            grad_clip_norm = algo_params.pop('grad_clip_norm')

        self.model = model_class(
            policy=algo_params.pop('policy_type'),
            env=self.env,
            **algo_params,
            verbose=1,
            tensorboard_log=str(self.logs_path),
            device='auto',
        )
        # Set grad_clip_norm attribute for SACWithGradClip
        if algo_name == 'sac' and grad_clip_norm is not None:
            setattr(self.model, 'grad_clip_norm', grad_clip_norm)
        print(f"--- Using {algo_name.upper()} model ---")
        print(self.model.policy)
        print('Device:', self.model.device)

    def _get_callbacks(self) -> List[BaseCallback]:
        """Configures training callbacks."""
        class OverwriteCheckpointCallback(CheckpointCallback):
            def _save_model(self, num_timesteps: int) -> None:
                # Always overwrite the checkpoint file with the same name
                filename = f"{self.name_prefix}_{num_timesteps}_steps.zip"
                path = os.path.join(self.save_path, filename)
                self.model.save(path)
                if self.verbose > 0:
                    print(f"Saving model checkpoint to {path}")

        callbacks = [
            OverwriteCheckpointCallback(
                save_freq=self.config.save_freq,
                save_path=str(self.checkpoints_path),
                name_prefix=f'{self.config.algo}_model'
            ),
            EvalCallback(
                eval_env=self.env,
                best_model_save_path=str(self.best_model_path),
                log_path=str(self.eval_logs_path),
                eval_freq=self.config.eval_freq,
                n_eval_episodes=self.config.n_eval_episodes,
                deterministic=True
            )
        ]
        if self.config.algo == 'td3':
            callbacks.append(
                AdaptiveExplorationCallback(
                    start_noise=self.config.td3.action_noise_std,
                    end_noise=0.05,
                    decay_steps=int(self.config.total_timesteps * 0.8),
                    verbose=1
                )
            )
        return callbacks

    def train(self):
        """Starts the training process."""
        class PrintRewardCallback(BaseCallback):
            def __init__(self):
                super().__init__()
                self.episode_rewards = []
                self.episode_steps = []
                self.current_reward = 0
                self.current_steps = 0
                self.collision_count = 0
                self.goal_count = 0

            def _on_step(self) -> bool:
                infos = self.locals.get('infos', [])
                dones = self.locals.get('dones', [])
                rewards = self.locals.get('rewards', [])
                for i, done in enumerate(dones):
                    self.current_reward += rewards[i]
                    self.current_steps += 1
                    
                    # Check if this is a terminal step and classify the ending
                    if done:
                        episode_type = "UNKNOWN"
                        if len(infos) > i:
                            info = infos[i]
                            # Check for collision indicators
                            if (rewards[i] < -50.0 or  # Large negative reward indicates collision
                                info.get('timeout_collision', False)):
                                episode_type = "COLLISION"
                                self.collision_count += 1
                            elif rewards[i] > 50.0:  # Large positive reward indicates goal
                                episode_type = "GOAL"
                                self.goal_count += 1
                            elif self.current_steps >= 500:
                                episode_type = "TIMEOUT"
                            else:
                                episode_type = "OTHER"
                        
                        print(f"[TRAIN] Episode END ({episode_type}): Reward: {self.current_reward:.2f}, Steps: {self.current_steps}")
                        print(f"        Last step reward: {rewards[i]:.2f} | Goals: {self.goal_count}, Collisions: {self.collision_count}")
                        
                        self.episode_rewards.append(self.current_reward)
                        self.episode_steps.append(self.current_steps)
                        self.current_reward = 0
                        self.current_steps = 0
                return True

        try:
            self.model.learn(
                total_timesteps=self.config.total_timesteps,
                callback=[PrintRewardCallback()] + self._get_callbacks(),
                log_interval=1 if self.config.algo == 'ppo' else 4, # PPO logs per rollout, others per episode
                progress_bar=True,
                tb_log_name=f'{self.config.algo}_training'
            )
        except KeyboardInterrupt:
            print("\nTraining interrupted by user.")
        finally:
            print(f"Saving final model to {self.final_model_path}")
            self.model.save(self.final_model_path)
            
            # Save the full configuration used for this run
            config_path = self.experiment_path / 'config.yaml'
            with open(config_path, 'w') as f:
                yaml.dump(asdict(self.config), f, sort_keys=False)

            self.env.close()
            rclpy.try_shutdown()

# --- Unified Inference Class ---
class Inference:
    """A unified class to run inference with a trained model."""
    def __init__(self, model_path: Path, algo: str, config: ExperimentConfig):
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found at {model_path}")

        if algo not in ALGO_MAP:
            raise ValueError(f"Unknown algorithm: {algo}. Available: {list(ALGO_MAP.keys())}")

        self.env = make_env(config)
        model_class, _ = ALGO_MAP[algo]
        self.model = model_class.load(model_path, env=self.env)
        # --- Disable action noise for evaluation ---
        if hasattr(self.model, 'action_noise'):
            self.model.action_noise = None
        print(f"--- Loaded {algo.upper()} model from {model_path} ---")

    def run(self, num_episodes: int = 10):
        """Runs the inference loop for a number of episodes."""
        episode_rewards = []
        try:
            for i in range(num_episodes):
                # Remove 'evaluation' kwarg for compatibility with wrappers
                obs_result = self.env.reset()
                if isinstance(obs_result, tuple):
                    obs = obs_result[0]
                else:
                    obs = obs_result
                done = False
                episode_reward = 0
                step = 0
                while not done:
                    # Remove 'evaluation' kwarg for compatibility
                    action, _ = self.model.predict(obs, deterministic=True)
                    step_result = self.env.step(action)
                    if len(step_result) == 5:
                        obs, reward, terminated, truncated, _ = step_result
                        done = terminated or truncated
                    else:
                        obs, reward, done, _ = step_result
                    episode_reward += reward
                    step += 1
                
                episode_rewards.append(episode_reward)
                print(f"[EVAL] Episode {i+1}: Reward = {episode_reward:.2f} in {step} steps")
            
            # Print summary statistics
            if episode_rewards:
                avg_reward = np.mean(episode_rewards)
                std_reward = np.std(episode_rewards)
                min_reward = np.min(episode_rewards)
                max_reward = np.max(episode_rewards)
                print(f"\n--- Evaluation Summary ---")
                print(f"Episodes: {len(episode_rewards)}")
                print(f"Average Reward: {avg_reward:.2f} ± {std_reward:.2f}")
                print(f"Min Reward: {min_reward:.2f}")
                print(f"Max Reward: {max_reward:.2f}")
                
        except KeyboardInterrupt:
            print("\nInference interrupted by user.")
        finally:
            self.env.close()
            rclpy.try_shutdown()

# --- Main Execution ---
def main():
    parser = argparse.ArgumentParser(
        description='Turtlebot4 Navigation with DRL (PPO, SAC, TD3)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # Use subparsers for 'train' and 'eval' commands
    subparsers = parser.add_subparsers(dest='command', required=True)

    # --- Training Parser ---
    train_parser = subparsers.add_parser('train')
    train_parser.add_argument(
        '--algo', 
        type=str, 
        default='ppo', 
        choices=list(ALGO_MAP.keys()),
        help='The reinforcement learning algorithm to use.'
    )
    train_parser.add_argument(
        '--continue_from',
        type=Path,
        help='Path to a pre-trained model to continue training from.'
    )
    # Future: You could add '--config file.yaml' to load custom configs

    # --- Evaluation Parser ---
    eval_parser = subparsers.add_parser('eval')
    eval_parser.add_argument(
        '--algo', 
        type=str, 
        required=True, 
        choices=list(ALGO_MAP.keys()),
        help='The algorithm the model was trained with.'
    )
    eval_parser.add_argument(
        '--model_path', 
        type=Path, 
        help='Path to the trained model .zip file. If not provided, uses the default final_model.zip.'
    )
    eval_parser.add_argument('--episodes', type=int, default=50, help='Number of episodes to run.')

    args = parser.parse_args()

    # Create a default config and update the chosen algorithm
    config = ExperimentConfig(algo=args.algo)

    if args.command == 'train':
        experiment = Experiment(config=config)
        if hasattr(args, 'continue_from') and args.continue_from is not None:
            # Load the pre-trained model
            model_class, _ = ALGO_MAP[config.algo]
            experiment.model = model_class.load(
                args.continue_from,
                env=experiment.env,
                tensorboard_log=str(experiment.logs_path),
                device='auto'
            )
            print(f"Loaded pre-trained model from {args.continue_from}")
            
            # IMPORTANT: Restore algorithm-specific attributes that were lost during load
            algo_params = asdict(getattr(config, config.algo))
            
            # # For SAC, restore gradient clipping configuration
            # if config.algo == 'sac' and 'grad_clip_norm' in algo_params:
            #     grad_clip_norm = algo_params['grad_clip_norm']
            #     setattr(experiment.model, 'grad_clip_norm', grad_clip_norm)
            #     print(f"Restored gradient clipping: {grad_clip_norm}")
            
            # --- Force learning rate to config value ---
            # Set learning rate for continued training
            experiment.model.learning_rate = getattr(config, config.algo).learning_rate
            # For SAC/TD3, update all optimizers (actor, critic, etc.)
            if hasattr(experiment.model, 'actor_optimizer'):
                for param_group in experiment.model.actor_optimizer.param_groups:
                    param_group['lr'] = getattr(config, config.algo).learning_rate
            if hasattr(experiment.model, 'critic_optimizer'):
                for param_group in experiment.model.critic_optimizer.param_groups:
                    param_group['lr'] = getattr(config, config.algo).learning_rate
            if hasattr(experiment.model, 'ent_coef_optimizer'):
                for param_group in experiment.model.ent_coef_optimizer.param_groups:
                    param_group['lr'] = getattr(config, config.algo).learning_rate
            # PPO uses model.policy.optimizer
            if hasattr(experiment.model.policy, 'optimizer'):
                for param_group in experiment.model.policy.optimizer.param_groups:
                    param_group['lr'] = getattr(config, config.algo).learning_rate
            # Print actual learning rates for verification
            print("--- Learning rates after update ---")
            print("experiment.model.learning_rate:", experiment.model.learning_rate)
            if hasattr(experiment.model, 'actor_optimizer'):
                print("actor_optimizer lr:", [pg['lr'] for pg in experiment.model.actor_optimizer.param_groups])
            if hasattr(experiment.model, 'critic_optimizer'):
                print("critic_optimizer lr:", [pg['lr'] for pg in experiment.model.critic_optimizer.param_groups])
            if hasattr(experiment.model, 'ent_coef_optimizer'):
                print("ent_coef_optimizer lr:", [pg['lr'] for pg in experiment.model.ent_coef_optimizer.param_groups])
            if hasattr(experiment.model.policy, 'optimizer'):
                print("policy.optimizer lr:", [pg['lr'] for pg in experiment.model.policy.optimizer.param_groups])
        experiment.train()
    elif args.command == 'eval':
        # Use the algorithm specified for evaluation
        eval_algo = args.algo
        # Build config for the eval algorithm
        eval_config = ExperimentConfig(algo=eval_algo)
        model_path = args.model_path
        # If no path is provided, construct the default path for the selected algo
        if model_path is None:
            model_path = (
                eval_config.log_dir / eval_config.experiment_name / 'final_model.zip'
            )
        inference = Inference(model_path=model_path, algo=eval_algo, config=eval_config)
        inference.run(num_episodes=args.episodes)

if __name__ == '__main__':
    main()

