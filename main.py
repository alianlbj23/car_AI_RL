import gymnasium as gym
from gym import spaces
import numpy as np
from AI_node import AiNode
import rclpy
import sys
import threading
from stable_baselines3 import PPO
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from env import CustomCarEnv

from stable_baselines3.common.callbacks import BaseCallback
import time
import datetime




class CustomCallback(BaseCallback):
    def __init__(self, save_path, save_freq, verbose=0):
        super(CustomCallback, self).__init__(verbose)
        self.save_path = save_path
        self.save_freq = save_freq
        self.last_save_step = 0

    def _on_step(self):
        # 基於某些條件調整噪聲
        # 注意: 您可能需要自定義邏輯來獲取和評估 loss
        # self.model.action_noise.sigma = ...

        # 定期保存模型
        if self.n_calls - self.last_save_step >= self.save_freq:
            now = datetime.datetime.now()
            self.time_name = datetime.datetime.timestamp(now)
            self.model.save(f"{self.save_path}_{self.n_calls}_{self.time_name}")
            self.last_save_step = self.n_calls

        return True

    
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

    # n_actions = env.action_space.shape[-1]
    try:
        model = PPO.load("./Model/ppo_custom_car_model_278000_1703457082.884543") #  load model
        model.set_env(env)
    except:
        model = PPO("MlpPolicy", env ,verbose=1,learning_rate=0.001)
    total_timesteps = 1000000
    custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
    model.learn(total_timesteps=total_timesteps, callback=custom_callback)
    rclpy.shutdown()
    pros.join()
    


if __name__ == '__main__':
    main()