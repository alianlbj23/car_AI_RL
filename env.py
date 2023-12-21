import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from tools import *
from collections import deque
from curious_model import *
import torch


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"
    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)

        #  好奇心模組
        state_dim = self.observation_space.shape[0]
        action_dim = self.action_space.n
        self.prediction_model = PredictionModel(state_dim, action_dim)
        self.prediction_model_optimizer = torch.optim.Adam(self.prediction_model.parameters(), lr=0.001)

        # 初始化状态
        self.state = None

        # ROS 和 Unity 接口的节点
        self.AI_node = AI_node

        self.previous_state = None
        self.previous_action = None
        self.training_data = []

    def _process_data(self, unity_data):  #  將data轉成numpy
        while unity_data is None:
            # time.sleep(0.1)
            unity_data = self.AI_node.get_latest_data()

        # 现在处理数据
        flat_list = []
        for value in unity_data.values():
            if isinstance(value, list):
                flat_list.extend(value)
            else:
                flat_list.append(value)
        return np.array(flat_list, dtype=np.float32)

    def reward_calculate(self, current_state, action, next_state):
        reward = 0
        if self.previous_state is not None and self.previous_action is not None:
            # 将动作转换为模型可接受的形式（如果需要）
            action_tensor = torch.tensor([self.previous_action], dtype=torch.float32)
            current_state_tensor = torch.tensor(self.previous_state, dtype=torch.float32)
            next_state_tensor = torch.tensor(next_state, dtype=torch.float32)
            curiosity_reward = calculate_curiosity_reward(self.prediction_model,
                                                          current_state_tensor,
                                                          action_tensor,
                                                          next_state_tensor)
            reward += curiosity_reward
        return reward

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退
        #  不設定sleep，ros會因為訊息過大而崩潰
        # time.sleep(0.5)

        self.AI_node.reset()
        # action_flag = self.AI_node.start_work()
        self.AI_node.publish_to_unity(action)   

        if action == 2 or action == 3:
            self.current_action = action


        current_state = self._wait_for_data()
        current_state = data_dict_pop(current_state)
        self.state = self._process_data(current_state)

        if self.previous_state is not None and self.previous_action is not None:
            self.training_data.append((self.previous_state, self.previous_action, self.state))
        self.previous_state = self.state
        self.previous_action = action
        BATCH_SIZE = 32
        if len(self.training_data) >= BATCH_SIZE:
            train_model(self.prediction_model, self.training_data, self.prediction_model_optimizer)
            self.training_data.clear()
            
        reward = self.reward_calculate(current_state, action, self.state)

        # print("self.step_count : ", self.step_count)
        terminated = current_state['car_target_distance'] < 1 or min(current_state['lidar_data']) < 0.2 #or self.step_count == 100
        return self.state, reward, terminated, False, {}
    
    def _wait_for_data(self): #  等待最新的data
        unity_data = self.AI_node.get_latest_data()
        while unity_data is None:
            unity_data = self.AI_node.get_latest_data()
        return unity_data


    def reset(self,seed=None, options=None):
        self.AI_node.reset()
        self.AI_node.publish_to_unity_RESET()
        unity_data_reset_state = self.AI_node.get_latest_data()
        
        while unity_data_reset_state == None: # waiting for unity state
            unity_data_reset_state = self.AI_node.get_latest_data()

        unity_data_reset_state = data_dict_pop(unity_data_reset_state)
            
        self.state = self._process_data(unity_data_reset_state)

        self.previous_state = None
        self.previous_action = None

        print("Reset Game")
        print(unity_data_reset_state)
        self.AI_node.reset()
        time.sleep(1)
        return self.state, {}
    
    
def data_dict_pop(data_dict):
    data_dict.pop('car_quaternion', None)
    data_dict.pop('car_pos', None)
    data_dict.pop('target_pos', None)
    return data_dict