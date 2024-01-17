import gymnasium as gym
from ROS_receive_and_data_processing.AI_node import AiNode
import rclpy
import threading
from stable_baselines3 import PPO
from RL.env import CustomCarEnv
from RL.custom_callback import CustomCallback
from avoidance_rule.rule_base import RuleBasedController
from manual.manual_base import ManualBasedController
import sys
from supervised.LSTM_inference import LSTMInference

#  或許之後要改成以node為主, 所以根本不用thread
def init_ros_node():
    '''node初始化並開一個thread跑ros node'''
    rclpy.init()
    node = AiNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

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
    mode = "rl"
    if len(sys.argv) >= 2:
        mode = sys.argv[1]
        
    node, ros_thread = init_ros_node()
    
    if mode.lower() == "rl":
        env = gym_env_register(node)
        train_model(env)
    elif mode.lower() == "rule":
        rule_controller = RuleBasedController(
            node, 
            './Simulated_Annealing_model/parameters.pkl',
            load_parameters=True,
            save_to_csv=True
            )
        rule_controller.run()
    elif mode.lower() == "manual":
        manual_controller = ManualBasedController(node)
        manual_controller.run()
    # elif mode.lower() == "lstm":
    #     lstm_controller = LSTMInferenceController(
    #     node=node,
    #     model_path='final_model.pth',
    #     input_size=100,  # 输入尺寸
    #     hidden_layer_size=100,  # 隐藏层尺寸
    #     output_size=1,  # 输出尺寸
    #     scaler_params=([0, 0, 0], [1, 1, 1]),  # 标准化器参数
    #     time_steps=3  # 时间步长
    # )
    else:
        print("Invalid mode. Please use 'RL' or 'rule'.")
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()