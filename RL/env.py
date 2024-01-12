import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

from avoidance import refined_obstacle_avoidance_with_target_orientation
from RL.reward_cal import reward_calculate
from RL.utils import process_data, wait_for_data

class CustomCarEnv(gym.Env):
    print("hello")
    ENV_NAME = "CustomCarEnv-v0"
    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

        # state初始化
        self.state = None
        self.AI_node = AI_node
        self.start_time = time.time() 

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退

        elapsed_time = time.time() - self.start_time #  計時
        
        self.AI_node.publish_to_unity(action) #  送出後會等到unity做完動作後
        self.AI_node.reset()  

        unity_data, unity_data_for_reward = wait_for_data(self.AI_node)

        reward = reward_calculate(unity_data_for_reward)

        if elapsed_time > 180: #  3分鐘限制
            print("time up")

        self.state = process_data(unity_data)
        
        terminated = (
            unity_data['car_target_distance'] < 1 or 
            min(unity_data['lidar_data']) < 0.2 or 
            elapsed_time > 180
        )
        
        
        return self.state, reward, terminated, False, {}

    def reset(self,seed=None, options=None):
        self.AI_node.publish_to_unity_RESET() #  送結束訊後給unity
        self.AI_node.reset()
        unity_data_reset_state, _ = wait_for_data(self.AI_node)
        self.state = process_data(unity_data_reset_state) 

        self.start_time = time.time()

        print("Reset Game")
        self.AI_node.reset()
        time.sleep(1)
        return self.state, {}
    
def rule_action(obs_for_avoidance):
    action = refined_obstacle_avoidance_with_target_orientation(
        obs_for_avoidance['lidar_data'],
        obs_for_avoidance['car_quaternion'][0],
        obs_for_avoidance['car_quaternion'][1],
        obs_for_avoidance['car_pos'],
        obs_for_avoidance['target_pos']
    )
    return action