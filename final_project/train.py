# final_project/train.py
import os
import time
import argparse
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

# import the env (this file should be in your package)
from final_project.envs.turtlebot_env import Turtlebot3GymEnv

def make_env():
    return Turtlebot3GymEnv(
        node_name='turtlebot3_env_rl',
        scan_topic='/scan',
        odom_topic='/odom',
        cmd_topic='/cmd_vel',
        lidar_dims=360,
        max_env_steps=500
    )

def main(total_timesteps=200000):
    env = DummyVecEnv([make_env])
    model = PPO('MlpPolicy', env, verbose=1, n_steps=2048, batch_size=64, learning_rate=3e-4)
    timestamp = int(time.time())
    save_path = os.path.expanduser(f'~/turtlebot3_ppo_{timestamp}')
    os.makedirs(save_path, exist_ok=True)

    print("Starting training. Make sure Gazebo + turtlebot are running.")
    model.learn(total_timesteps=total_timesteps)
    model.save(os.path.join(save_path, 'ppo_turtlebot3'))

    print(f"Model saved to {save_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=200000, help='Total timesteps for training')
    args = parser.parse_args()
    main(total_timesteps=args.timesteps)
