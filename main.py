import gymnasium as gym
from ROS_receive_and_data_processing.AI_node import AiNode
import rclpy
import threading
from stable_baselines3 import PPO
from RL.env import CustomCarEnv
from RL.custom_callback import CustomCallback

def init_ros_node():
    '''node初始化並開一個thread跑ros node'''
    rclpy.init()
    node = AiNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

#  
def load_or_create_model(env, model_path):
    '''讀取model或是重新train一個'''
    try:
        model = PPO.load(model_path) #  load model
        model.set_env(env)
    except FileNotFoundError: #  找不到就重新train一個
        model = PPO("MlpPolicy", env ,verbose=1,learning_rate=0.001)
    return model

def train_model(env):
    model = load_or_create_model(env, "./Model/ppo_custom_car_model_278000_1703457082.884543")
    custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
    total_timesteps = 100000 # 訓練回合數
    model.learn(total_timesteps=total_timesteps, callback=custom_callback) #  進入env開始訓練

def gym_env_register(AI_node):
    gym.register(
        id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
        entry_point='RL.env:CustomCarEnv',  # 模組名稱:類名稱
    )  
    return gym.make("CustomCarEnv-v0", AI_node=AI_node)

def main():
    node, ros_thread = init_ros_node()
    env = gym_env_register(node)
    train_model(env)
    
    #  中斷node
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()