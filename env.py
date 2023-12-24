import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from tools import *
from collections import deque


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"
    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

        # 初始化状态
        self.state = None

        # ROS 和 Unity 接口的节点
        self.AI_node = AI_node

        self.reset_done_flag = 0

        self.last_car_target_distance = 0
        self.last_car_position = np.inf
        self.previous_steering_angle = np.inf
        self.previous_action = None
        self.previous_direction = None

        self.action_history = deque(maxlen=10) 

        self.start_time = time.time() 

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

    def reward_calculate(self, state_dict):

        reward = 0
        car_pos = state_dict['car_pos']
        car_quaternion = state_dict['car_quaternion']
        target_pos = state_dict['target_pos']
        current_distance = state_dict['car_target_distance']
        lidar_data = state_dict['lidar_data']
        
        #  與目標距離
        reward += calculate_distance_change(current_distance, 1.5)
        
        #紀錄上一次的距離
        # self.last_car_target_distance = current_distance

        #  lidar
        reward += calculate_lidar_based_reward(lidar_data, 0.45, self.current_action)*100

        #  利用偏行角算分 待觀察
        reward += calculate_angle_point(car_quaternion[0], car_quaternion[1], car_pos, target_pos)

        #  平穩駕駛獎勵
        # reward += calculate_drive_reward(current_steering_angle, self.previous_steering_angle)
        # self.previous_steering_angle = current_steering_angle
        
        #  防止左右一直擺頭
        # reward += calculate_drive_reward(self.current_action, self.previous_direction)
        # self.previous_direction = self.current_action
        # print(reward)
        return reward

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退
        #  不設定sleep，ros會因為訊息過大而崩潰
        elapsed_time = time.time() - self.start_time
        
        # action_flag = self.AI_node.start_work()
        self.AI_node.publish_to_unity(action) 
        self.AI_node.reset()  

        if action == 2 or action == 3:
            self.current_action = action
        if elapsed_time > 180:
            print("time up")

        unity_data = self._wait_for_data()

        unity_data_for_reward = unity_data.copy()
        unity_data = data_dict_pop(unity_data)
        # unity_data.pop('car_quaternion', None)
        self.state = self._process_data(unity_data)
        
        reward = self.reward_calculate(unity_data_for_reward)

        # print("self.step_count : ", self.step_count)
        terminated = unity_data['car_target_distance'] < 1 or min(unity_data['lidar_data']) < 0.2 or elapsed_time > 180
        return self.state, reward, terminated, False, {}
    
    def _wait_for_data(self): #  等待最新的data
        unity_data = self.AI_node.get_latest_data()
        while unity_data is None:
            unity_data = self.AI_node.get_latest_data()
        return unity_data


    def reset(self,seed=None, options=None):
        
        self.AI_node.publish_to_unity_RESET()
        self.AI_node.reset()
        unity_data_reset_state = self._wait_for_data()
        
        while unity_data_reset_state == None: # waiting for unity state
            unity_data_reset_state = self.AI_node.get_latest_data()
        unity_data_reset_state = data_dict_pop(unity_data_reset_state)
        # unity_data_reset_state.pop('car_quaternion', None)
        # unity_data_reset_state.pop('car_pos', None)
        # unity_data_reset_state.pop('target_pos', None)
            
        self.state = self._process_data(unity_data_reset_state)
        self.last_car_target_distance = 0
        self.last_car_position = np.inf
        self.previous_steering_angle = np.inf
        

        self.current_action = 0
        self.previous_direction = 0
        self.action_history = deque(maxlen=10) 
        self.start_time = time.time()

        # self.AI_node.not_work()
        print("Reset Game")
        self.AI_node.reset()
        time.sleep(1)
        return self.state, {}
    
    
def data_dict_pop(data_dict):
    data_dict.pop('car_quaternion', None)
    data_dict.pop('car_pos', None)
    data_dict.pop('target_pos', None)
    return data_dict