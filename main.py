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
        if self.current_step > self.total_timesteps:
            # 添加调试输出以检测问题
            print(f"Error: current_step ({self.current_step}) exceeded total_timesteps ({self.total_timesteps})")
        progress = self.current_step / self.total_timesteps
        progress = min(max(progress, 0), 1)  # 确保进度在0和1之间
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

    # model = DDPG.load("./Model/ddpg_custom_car_model")

    env = gym.make("CustomCarEnv-v0", AI_node=node)

    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    model = DDPG("MlpPolicy", env, action_noise=action_noise ,verbose=1)
    total_timesteps = 10000
    progress_callback = ProgressBarCallback(total_timesteps)
    
    model.learn(total_timesteps=total_timesteps)
    model.save("./Model/ddpg_custom_car_model")  # 保存模型
    rclpy.shutdown()
    pros.join()
    


if __name__ == '__main__':
    main()