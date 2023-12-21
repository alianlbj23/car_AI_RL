import gymnasium as gym
from gym import spaces
import numpy as np
from AI_node import AiNode
import rclpy
import sys
import threading
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from env import CustomCarEnv

from stable_baselines3.common.callbacks import BaseCallback
import time

    
def spin_ros_node(node):
    rclpy.spin(node)
    node.destroy_node()

def main():
    gym.register(
        id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
        entry_point='env:CustomCarEnv',  # 模組名稱:類名稱
    )

    rclpy.init()
    node = AiNode()
    pros = threading.Thread(target=spin_ros_node, args=(node,))
    pros.start()  
    # model = DDPG.load("./Model/ddpg_custom_car_model")

    env = gym.make("CustomCarEnv-v0", AI_node=node)

    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    model = DDPG("MlpPolicy", env, action_noise=action_noise ,verbose=1)
    total_timesteps = 10000
    # progress_callback = ProgressBarCallback(total_timesteps)
    
    model.learn(total_timesteps=total_timesteps)
    model.save("./Model/ddpg_custom_car_model")  # 保存模型
    rclpy.shutdown()
    pros.join()
    


if __name__ == '__main__':
    main()