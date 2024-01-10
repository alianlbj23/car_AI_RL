import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

from avoidance import refined_obstacle_avoidance_with_target_orientation
from reward_cal import reward_calculate
from utils import process_data, wait_for_data

# 要將rule和RL分開，這個檔案因該要專心做RL
class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"
    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

        # state初始化
        self.state = None

        # ROS和Unity的資料處理
        self.AI_node = AI_node
        # self.obs_for_avoidance = None #  使用rule的時候用來記錄第一次的obs
        self.start_time = time.time() 
        # self.mode = "rule1"


    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退

        elapsed_time = time.time() - self.start_time #  計時
        
        # #  設定為rule模式，如果要用reward跑action的話self.mode射程其他的字眼
        # if self.mode == "rule":
        #     if self.obs_for_avoidance != None:
        #         action = rule_action(self.obs_for_avoidance)
        #     else:
        #         pass
        
        self.AI_node.publish_to_unity(action) #  送出後會等到unity做完動作後
        self.AI_node.reset()  

        unity_data, unity_data_for_reward = wait_for_data(self.AI_node)

        reward = reward_calculate(unity_data_for_reward)
        # self.obs_for_avoidance = unity_data_for_reward #  給rule用的

        if elapsed_time > 180: #  3分鐘限制
            print("time up")

        terminated = (
            unity_data['car_target_distance'] < 1 or 
            min(unity_data['lidar_data']) < 0.2 or 
            elapsed_time > 180
        )
        
        self.state = process_data(unity_data)
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