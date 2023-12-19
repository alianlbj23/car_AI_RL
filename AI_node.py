import numpy as np
from datetime import datetime
from UnityAdaptor import transfer_obs
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from collections import deque
import threading


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