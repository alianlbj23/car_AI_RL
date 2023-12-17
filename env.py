import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"
    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        self.action_space = spaces.Box(low=np.array([-10, -10]), high=np.array([10, 10]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(70,), dtype=np.float32)

        # 初始化状态
        self.state = None

        # ROS 和 Unity 接口的节点
        self.AI_node = AI_node

        self.update_flag = 0

        self.last_car_target_distance = np.inf
    
    def _process_data(self, unity_data):
        while unity_data is None:
            time.sleep(0.1)
            unity_data = self.AI_node.get_latest_data()

        # 现在处理数据
        flat_list = []
        for value in unity_data.values():
            if isinstance(value, list):
                flat_list.extend(value)
            else:
                flat_list.append(value)
        return np.array(flat_list, dtype=float)

    def reward_calculate(self, state_dict):
        if state_dict is None:
            return -1  # 或者其他默认奖励值
        reward = -1
        car_target_distance = state_dict['car_target_distance']
        lidar_data = state_dict['lidar_data']
        if min(lidar_data) < 0.5:  # 假设0.5是接近墙壁的阈值
            reward += 100
        elif min(lidar_data) < 0.1:  # 假设0.1是撞击墙壁的阈值
            reward += 500  # 撞击墙壁时给予更大的奖励


        
        return reward

    def step(self, action):
        self.AI_node.publish_to_unity(action)

        unity_data = self.AI_node.get_latest_data()
        
        if self.state is None:
            return None, 0, True, {}
        while unity_data is None:
            time.sleep(0.1)
            unity_data = self.AI_node.get_latest_data()

        self.state = self._process_data(unity_data)
        reward = self.reward_calculate(unity_data)

        terminated = False
        truncated = False
        
        if unity_data['car_target_distance'] < 1 or min(unity_data['lidar_data']) < 0.5:
            terminated = True
        else:
            terminated = False
        
        info = {} 

        return self.state, reward, terminated, truncated, info


    def reset(self,seed=None, options=None):
        self.AI_node.publish_to_unity_RESET()
        unity_data = self.AI_node.get_latest_data()
        self.state = self._process_data(unity_data)
        print(len(self.state))

        return self.state, {}

