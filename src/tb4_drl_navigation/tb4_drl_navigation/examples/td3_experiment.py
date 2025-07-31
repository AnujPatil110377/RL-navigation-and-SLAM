#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TD3 training script for Turtlebot4Env-v0
Unified with PPO/SAC/TD3 experiment structure from ppo.py
"""
import argparse
import numpy as np
import torch
import random
import rclpy
from pathlib import Path
from dataclasses import asdict
from td3 import Agent
import gymnasium as gym

# --- Environment registration ---
from gymnasium.envs.registration import register
register(
    id='Turtlebot4Env-v0',
    entry_point='tb4_drl_navigation.envs.diffdrive.turtlebot4:Turtlebot4Env',
)

# --- Experiment configuration ---
class TD3Config:
    def __init__(self):
        self.state_size = None
        self.action_size = None
        self.hidden_size = 256
        self.actor_learning_rate = 1e-3
        self.critic_learning_rate = 1e-3
        self.batch_size = 128
        self.buffer_size = 1000000
        self.discount_factor = 0.99
        self.softupdate_coefficient = 1e-2
        self.noise_std = 0.2
        self.noise_clip = 0.5
        self.policy_update = 2
        self.max_lin_vel = None
        self.max_ang_vel = None
        self.num_episodes = 1000
        self.max_steps = 500
        self.min_buffer_size = 1000
        self.save_every = 100
        self.seed = 42
        self.model_dir = Path('./td3_models')
        self.model_dir.mkdir(exist_ok=True)

# --- Unified TD3 Experiment ---
class TD3Experiment:
    def __init__(self, config: TD3Config):
        self.config = config
        random.seed(config.seed)
        np.random.seed(config.seed)
        torch.manual_seed(config.seed)
        rclpy.init()
        env = gym.make('Turtlebot4Env-v0', world_name='static_world')
        print("[DEBUG] env.observation_space:", env.observation_space)
        print("[DEBUG] env.action_space:", env.action_space)
        if env.observation_space is None or env.action_space is None:
            raise ValueError("Your environment's observation_space or action_space is None. Please ensure they are set in Turtlebot4Env.__init__().")
        if not hasattr(env.observation_space, 'shape') or env.observation_space.shape is None:
            raise ValueError("env.observation_space.shape is None. Please set observation_space with a valid shape in Turtlebot4Env.")
        if not hasattr(env.action_space, 'shape') or env.action_space.shape is None:
            raise ValueError("env.action_space.shape is None. Please set action_space with a valid shape in Turtlebot4Env.")
        config.state_size = env.observation_space.shape[0]
        config.action_size = env.action_space.shape[0]
        config.max_lin_vel = env.action_space.high[0]
        config.max_ang_vel = env.action_space.high[1]
        self.env = env
        self.agent = Agent(
            state_size=config.state_size,
            action_size=config.action_size,
            hidden_size=config.hidden_size,
            actor_learning_rate=config.actor_learning_rate,
            critic_learning_rate=config.critic_learning_rate,
            batch_size=config.batch_size,
            buffer_size=config.buffer_size,
            discount_factor=config.discount_factor,
            softupdate_coefficient=config.softupdate_coefficient,
            max_lin_vel=config.max_lin_vel,
            max_ang_vel=config.max_ang_vel,
            noise_std=config.noise_std,
            noise_clip=config.noise_clip,
            policy_update=config.policy_update
        )
        self.model_dir = config.model_dir

    def train(self):
        global_step = 0
        for episode in range(self.config.num_episodes):
            obs, info = self.env.reset()
            state = np.array(obs, dtype=np.float32)
            episode_reward = 0
            for step in range(self.config.max_steps):
                action = self.agent.act(state, global_step, add_noise=True)
                next_obs, reward, terminated, truncated, info = self.env.step(action[0])
                next_state = np.array(next_obs, dtype=np.float32)
                done = terminated or truncated
                self.agent.step(state, action[0], reward, next_state, done)
                if len(self.agent.memory) > self.agent.batch_size and len(self.agent.memory) > self.config.min_buffer_size:
                    self.agent.learn(global_step)
                state = next_state
                episode_reward += reward
                global_step += 1
                if done:
                    break
            print(f"Episode {episode}: Reward = {episode_reward:.2f}")
            if episode % self.config.save_every == 0:
                self.agent.save_actor_model(str(self.model_dir), f'actor_ep{episode}.pth')
                self.agent.save_critic1_model(str(self.model_dir), f'critic1_ep{episode}.pth')
                self.agent.save_critic2_model(str(self.model_dir), f'critic2_ep{episode}.pth')
        self.env.close()
        rclpy.try_shutdown()
        print("TD3 training finished.")

    def eval(self, num_episodes=10, actor_path=None):
        if actor_path is not None:
            self.agent.actor_local.load_state_dict(torch.load(actor_path))
        rewards = []
        for ep in range(num_episodes):
            obs, info = self.env.reset()
            state = np.array(obs, dtype=np.float32)
            episode_reward = 0
            done = False
            step = 0
            while not done:
                action = self.agent.act(state, step, add_noise=False)
                next_obs, reward, terminated, truncated, info = self.env.step(action[0])
                state = np.array(next_obs, dtype=np.float32)
                episode_reward += reward
                done = terminated or truncated
                step += 1
            rewards.append(episode_reward)
            print(f"[EVAL] Episode {ep+1}: Reward = {episode_reward:.2f} in {step} steps")
        print(f"\n--- Evaluation Summary ---")
        print(f"Episodes: {len(rewards)}")
        print(f"Average Reward: {np.mean(rewards):.2f} ± {np.std(rewards):.2f}")
        print(f"Min Reward: {np.min(rewards):.2f}")
        print(f"Max Reward: {np.max(rewards):.2f}")
        self.env.close()
        rclpy.try_shutdown()

# --- Main Execution ---
def main():
    parser = argparse.ArgumentParser(description='TD3 Training/Evaluation for Turtlebot4Env-v0')
    subparsers = parser.add_subparsers(dest='command', required=True)
    train_parser = subparsers.add_parser('train')
    eval_parser = subparsers.add_parser('eval')
    eval_parser.add_argument('--episodes', type=int, default=10, help='Number of episodes to run for evaluation')
    eval_parser.add_argument('--actor_path', type=str, help='Path to actor model for evaluation')
    args = parser.parse_args()
    config = TD3Config()
    experiment = TD3Experiment(config)
    if args.command == 'train':
        experiment.train()
    elif args.command == 'eval':
        experiment.eval(num_episodes=args.episodes, actor_path=args.actor_path)

if __name__ == '__main__':
    main()
