import numpy as np
from datetime import datetime
import os
from UnityAdaptor import transfer_obs
import threading
import sys
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import csv
from datetime import datetime

class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")#ros2Ai #unity2Ros

        self.subscriber_fromUnity_thu_ROSbridge_ = self.create_subscription(
            String, 
            "Unity_2_AI", 
            self.callback_from_Unity, 
            10
        )

        self.subscriber_fromUnity_thu_ROSbridge_stopFlag = self.create_subscription(
            String, 
            "Unity_2_AI_stop_flag", 
            self.callback_from_Unity_stop_flag, 
            10
        )

        self.state_detect = 0
        self.tokens = list()
    
    def collect_unity_data(self, unityState):
        self.state_detect, token = transfer_obs(unityState)
        if self.state_detect == 1:
            print(len(token))
        else:
            print("Unity lidar no signal.....")
            

    def callback_from_Unity(self, msg):
        self.collect_unity_data(msg.data)


def spin_pros(node):
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(node)
    exe.spin()
    rclpy.shutdown()
    sys.exit(0)


def main():
    rclpy.init()
    node = AiNode()
    pros = threading.Thread(target=spin_pros, args=(node,))
    pros.start()  

if __name__ == '__main__':
    
    main()