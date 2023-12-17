import gymnasium as gym
from gym import spaces
import numpy as np
from AI_node import AiNode
import rclpy
import sys
import threading
from stable_baselines3 import DDPG
from env import CustomCarEnv

def spin_pros(node):
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(node)
    exe.spin()
    rclpy.shutdown()
    sys.exit(0)

def main():
    gym.register(
        id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
        entry_point='env:CustomCarEnv',  # 模組名稱:類名稱
    )
    rclpy.init()
    node = AiNode()
    pros = threading.Thread(target=spin_pros, args=(node,))
    pros.start()  
    env = gym.make("CustomCarEnv-v0", AI_node=node)
    model = DDPG("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=10000)
    rclpy.shutdown()


if __name__ == '__main__':
    main()