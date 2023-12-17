import gymnasium as gym
from gym import spaces
import numpy as np
from AI_node import AiNode
import rclpy
import sys
import threading
from stable_baselines3 import DDPG
from env import CustomCarEnv

from stable_baselines3.common.callbacks import BaseCallback
import time

class ProgressBarCallback(BaseCallback):
    def __init__(self, total_timesteps):
        super(ProgressBarCallback, self).__init__()
        self.total_timesteps = total_timesteps
        self.start_time = time.time()

    def _on_training_start(self):
        # 在训练开始时调用
        self.current_step = 0
        sys.stdout.write("\n")  # 开始新的一行

    def _on_step(self):
        # 每个步骤调用一次
        self.current_step += 1
        progress = self.current_step / self.total_timesteps
        filled_length = int(50 * progress)
        bar = '#' * filled_length + '-' * (50 - filled_length)
        sys.stdout.write(f"\rProgress: |{bar}| {progress:.1%}")
        sys.stdout.flush()
        return True
    def _on_training_end(self):
        # 训练结束时调用
        sys.stdout.write("\n")  # 结束进度条行
    
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
    env = gym.make("CustomCarEnv-v0", AI_node=node)
    model = DDPG("MlpPolicy", env, verbose=1)
    total_timesteps = 10000
    progress_callback = ProgressBarCallback(total_timesteps)
    model.learn(total_timesteps=10000, callback=progress_callback)
    rclpy.shutdown()
    pros.join()


if __name__ == '__main__':
    main()