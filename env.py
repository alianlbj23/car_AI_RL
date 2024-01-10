import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from tools import *
from avoidance import refined_obstacle_avoidance_with_target_orientation

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
        
        self.obs_for_avoidance = None
        self.start_time = time.time() 
        self.mode = "rule"


    def _process_data(self, unity_data):  #  將data轉成numpy
        while unity_data is None:
            unity_data = self.AI_node.get_latest_data()

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

        #  lidar
        reward += calculate_lidar_based_reward(lidar_data, 0.45)*100

        #  利用偏行角算分 待觀察
        reward += calculate_angle_point(car_quaternion[0], car_quaternion[1], car_pos, target_pos)*10
        
        return reward

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退
        #  不設定sleep，ros會因為訊息過大而崩潰

        elapsed_time = time.time() - self.start_time
        
        #  設定為rule模式，如果要用reward跑action的話self.mode射程其他的字眼
        if self.mode == "rule":
            if self.obs_for_avoidance != None:
                action = rule_action(self.obs_for_avoidance)
            else:
                pass
        
        self.AI_node.publish_to_unity(action) #  送出後會等到unity做完動作後
        self.AI_node.reset()  

        unity_data = self._wait_for_data()

        unity_data_for_reward = unity_data.copy() #  複製一個給reward計算用

        unity_data = data_dict_pop(unity_data) #  將不該給observation的拿掉
        self.state = self._process_data(unity_data)  #  轉成np array

        self.obs_for_avoidance = unity_data_for_reward #  給rule用的

        reward = self.reward_calculate(unity_data_for_reward)

        if elapsed_time > 180: #  3分鐘限制
            print("time up")

        terminated = unity_data['car_target_distance'] < 1 or min(unity_data['lidar_data']) < 0.2 or elapsed_time > 180
        return self.state, reward, terminated, False, {}
    
    def _wait_for_data(self): #  等待最新的data
        unity_data = self.AI_node.get_latest_data()
        while unity_data is None:
            unity_data = self.AI_node.get_latest_data()
        return unity_data


    def reset(self,seed=None, options=None):
        
        self.AI_node.publish_to_unity_RESET() #  送結束訊後給unity
        self.AI_node.reset()
        unity_data_reset_state = self._wait_for_data()
        
        while unity_data_reset_state == None: # waiting for unity state
            unity_data_reset_state = self.AI_node.get_latest_data()
        unity_data_reset_state = data_dict_pop(unity_data_reset_state)
            
        self.state = self._process_data(unity_data_reset_state) 

        self.start_time = time.time()
        self.obs_for_avoidance = None

        print("Reset Game")
        self.AI_node.reset()
        time.sleep(1)
        return self.state, {}
    
#  將沒用到的feature拔掉
def data_dict_pop(data_dict):
    data_dict.pop('car_quaternion', None)
    data_dict.pop('car_pos', None)
    data_dict.pop('target_pos', None)
    return data_dict

def rule_action(obs_for_avoidance):
    action = refined_obstacle_avoidance_with_target_orientation(
        obs_for_avoidance['lidar_data'],
        obs_for_avoidance['car_quaternion'][0],
        obs_for_avoidance['car_quaternion'][1],
        obs_for_avoidance['car_pos'],
        obs_for_avoidance['target_pos']
    )
    return action

def set_mode():
    return "rule"    