import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from tools import *
from collections import deque
from UnityAdaptor import transfer_obs
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import threading


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

        self.reset_done_flag = 0

        self.last_car_target_distance = np.inf
        self.last_car_position = np.inf
        self.previous_steering_angle = np.inf
        self.step_count = 0

        self.action_history = deque(maxlen=10) 

        self.previous_action = None

    def _process_data(self, unity_data):  #  將data轉成numpy
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
        return np.array(flat_list, dtype=np.float32)

    def reward_calculate(self, state_dict):

        reward = 0
        car_pos = state_dict['car_pos']
        car_quaternion = state_dict['car_quaternion']
        target_pos = state_dict['target_pos']
        current_distance = state_dict['car_target_distance']
        lidar_data = state_dict['lidar_data']
        lidar_data = state_dict['lidar_data']
        current_steering_angle = state_dict['car_steering_angle'][0]
        left_wheel_speed = state_dict['wheel_angular_vel'][0]
        right_wheel_speed = state_dict['wheel_angular_vel'][1]
        
        max_reward = 50
        
        #  與目標距離
        reward += calculate_distance_change(current_distance, self.last_car_target_distance)
        
        #紀錄上一次的距離
        self.last_car_target_distance = current_distance

        #  lidar
        reward += calculate_lidar_based_reward(lidar_data, 0.5)

        #  利用偏行角算分
        car_yaw = get_yaw_from_quaternion(car_quaternion[0], car_quaternion[1])
        direction_vector = get_direction_vector(car_pos, target_pos)
        angle_to_target = get_angle_to_target(car_yaw, direction_vector)
        angle_diff = np.abs(angle_to_target - 180)
        angle_diff = min(angle_diff, 180)
        reward += max_reward - (max_reward / 180) * angle_diff

        #  平穩駕駛獎勵
        reward += calculate_drive_reward(current_steering_angle, self.previous_steering_angle)
        self.previous_steering_angle = current_steering_angle

        #  輪速控制
        reward += calculate_wheel_speed_reward(left_wheel_speed, right_wheel_speed)

        replace_flag = False
        replace_flag += check_spinning(self.action_history, 3)
        if replace_flag == True:
            reward -= 50
            self.action_history = deque(maxlen=10) 

        return reward

    def step(self, action):
        #  不設定sleep，ros會因為訊息過大而崩潰
        time.sleep(0.7)
        action_flag = self.AI_node.start_work()
        self.AI_node.publish_to_unity(action)

        if self.previous_action is not None:
            action_diff = np.linalg.norm(np.array(action) - np.array(self.previous_action))
        self.previous_action = action

        self.action_history.append(action)
        while action_flag == 0:
            action_flag = self.AI_node.get_action_flag()
        unity_data = self._wait_for_data()
        unity_data_for_reward = unity_data.copy()
        unity_data.pop('car_quaternion', None)
        self.state = self._process_data(unity_data)
        
        reward = self.reward_calculate(unity_data_for_reward)

        self.step_count += 1
        # print("self.step_count : ", self.step_count)
        terminated = unity_data['car_target_distance'] < 1 or min(unity_data['lidar_data']) < 0.2 or self.step_count == 500
        return self.state, reward, terminated, False, {}
    
    def _wait_for_data(self): #  等待最新的data
        unity_data = self.AI_node.get_latest_data()
        while unity_data is None:
            time.sleep(0.1)  
            unity_data = self.AI_node.get_latest_data()
        return unity_data


    def reset(self,seed=None, options=None):
        self.AI_node.publish_to_unity_RESET()
        unity_data = self.AI_node.get_latest_data()
        while unity_data == None:
            unity_data = self.AI_node.get_latest_data()
        unity_data.pop('car_quaternion', None)
        self.state = self._process_data(unity_data)

        self.last_car_target_distance = np.inf
        self.last_car_position = np.inf
        self.previous_steering_angle = np.inf
        self.step_count = 0
        self.action_history = deque(maxlen=10) 

        self.AI_node.not_work()
        print("Reset Game")
        return self.state, {}


class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")#ros2Ai #unity2Ros

        self.buffer_lock = threading.Lock()
        self.data_buffer = deque(maxlen=10) # 這個要討論
        
        self.reset_done_flag = 0
        self.action_flag = 0

        #  如果駐列不為1，會無法收到最新資料拿去判斷
        self.subscriber_fromUnity_thu_ROSbridge_ = self.create_subscription(
            String, 
            "/Unity_2_AI", 
            self.callback_from_Unity, 
            1
        )

        self.subscriber_fromUnity_thu_ROSbridge_action_finish = self.create_subscription(
            String, 
            "/Unity_2_AI", 
            self.callback_from_Unity_action_finish, 
            1
        )
        
        self.publisher_AINode_2_unity_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity', 
            1
        )

        self.publisher_AINode_2_unity_RESET_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity_RESET_flag', 
            1
        )

        self.publisher_AINode_2_unity_action_reset_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity_ACTION_RESET_flag', 
            1
        )

        self.active_communication = True
        
        self.latest_data = None
        self.state_detect = 0
    
    def start_work(self):
        self.latest_data = None
        self.active_communication = True
        self.action_flag = 0
        return self.action_flag
    
    def not_work(self):
        self.get_logger().info("waiting for reset")
        msg = Float32MultiArray()
        msg.data = [1.0]
        self.publisher_AINode_2_unity_action_reset_thu_ROSbridge.publish(msg)
        self.active_communication = False
        self.action_flag = 0

    def collect_unity_data(self, unityState): #處理收來的資料
        self.state_detect, data_dict = transfer_obs(unityState)
        if self.state_detect == 1:
            self.latest_data = data_dict #最後一部
            # self.data_buffer.append(data_dict)
        else:
            print("erro")
            pass
    
    def resume_communication(self):
        self.active_communication = True
        self.latest_data = None 

    def publish_to_unity(self, action):
        if self.active_communication == True:
            msg = Float32MultiArray()
            action = [float(value) for value in action]
            msg.data = action
            self.publisher_AINode_2_unity_thu_ROSbridge.publish(msg)
    
    def publish_to_unity_RESET(self):
        if self.active_communication == True:
            msg = Float32MultiArray()
            msg.data = [1.0] # 送個訊號過去觸發unity的重製，裡面數字沒有意義
            self.publisher_AINode_2_unity_RESET_thu_ROSbridge.publish(msg)

    def callback_from_Unity(self, msg):
        if self.active_communication:
            self.collect_unity_data(msg.data)
        else:
            pass
    
    def get_action_flag(self):
        return self.action_flag
    
    def callback_from_Unity_action_finish(self, msg):
        # print("action_flag : ", self.action_flag)
        self.action_flag = 1
        

    
    def reset(self):
        self.latest_data = None
        self.state_detect = 0
        self.data_buffer.clear()
        self.active_communication = True
        print("resset")

    def get_latest_data(self):
        return self.latest_data
        # return self.data_buffer[-1] if self.data_buffer else None
    
    #觀察buffer
    def get_all_buffered_data(self):
        return list(self.data_buffer)
    
    def print_buffer_contents(self):
        with self.buffer_lock:
            buffer_size = len(self.data_buffer)
            print(f"Buffer contains {buffer_size} items.")

