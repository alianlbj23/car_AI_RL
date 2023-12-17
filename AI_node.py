import numpy as np
from datetime import datetime
from UnityAdaptor import transfer_obs
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")#ros2Ai #unity2Ros

        self.subscriber_fromUnity_thu_ROSbridge_ = self.create_subscription(
            String, 
            "/Unity_2_AI", 
            self.callback_from_Unity, 
            100
        )
        
        self.publisher_AINode_2_unity_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity', 
            100
        )

        self.publisher_AINode_2_unity_RESET_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity_RESET_flag', 
            100
        )

        self.active_communication = True
        
        self.latest_data = None
        self.state_detect = 0
        self.tokens = list()
    
    def collect_unity_data(self, unityState):
        self.state_detect, data_dict = transfer_obs(unityState)
        if self.state_detect == 1:
            self.latest_data = data_dict
        else:
            print("Unity lidar no signal.....")

    def pause_communication(self):
        self.active_communication = False
    
    def resume_communication(self):
        self.active_communication = True
        self.latest_data = None  # 可能需要清除旧数据

    def publish_to_unity(self, action):
        msg = Float32MultiArray()
        action = [float(value) for value in action]
        msg.data = action
        self.publisher_AINode_2_unity_thu_ROSbridge.publish(msg)
    
    def publish_to_unity_RESET(self):
        msg = Float32MultiArray()
        msg.data = [1.0] # 送個訊號過去觸發unity的重製，裡面數字沒有意義
        self.publisher_AINode_2_unity_RESET_thu_ROSbridge.publish(msg)

    def callback_from_Unity(self, msg):
        if self.active_communication:
            self.collect_unity_data(msg.data)
        else:
            pass
    
    def get_latest_data(self):
        return self.latest_data