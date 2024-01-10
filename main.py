import gymnasium as gym
from AI_node import AiNode
import rclpy
import threading
from stable_baselines3 import PPO
from env import CustomCarEnv
from custom_callback import CustomCallback

def init_ros_node():
    rclpy.init()
    node = AiNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

#  如果有找到該model就讀取
def load_or_create_model(env, model_path):
    try:
        model = PPO.load(model_path) #  load model
        model.set_env(env)
    except FileNotFoundError:
        model = PPO("MlpPolicy", env ,verbose=1,learning_rate=0.001)
    return model

def train_model(model, total_timesteps, callback):
    model.learn(total_timesteps=total_timesteps, callback=callback)

def gym_env_register():
    gym.register(
        id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
        entry_point='env:CustomCarEnv',  # 模組名稱:類名稱
    )

def main():
    gym_env_register()
    node, ros_thread = init_ros_node()
    env = gym.make("CustomCarEnv-v0", AI_node=node)
    model = load_or_create_model(env, "./Model/ppo_custom_car_model_278000_1703457082.884543")
    custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
    train_model(model, 1000000, custom_callback)
    
    #  中斷node
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()