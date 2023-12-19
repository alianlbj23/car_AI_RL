# 跟車子互動用的
import numpy as np
from datetime import datetime
import os
from UnityAdaptor import transfer_obs
import threading
import sys
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
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

        self.publisher_AINode_2_unity_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            'AI_2_Unity', 
            10
        )

        self.publisher_AINode_2_unity_RESET_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            'AI_2_Unity_RESET_flag', 
            10
        )

        self.state_detect = 0
        self.tokens = list()

    def _process_data(self, unity_data):
        # while unity_data is None:
        #     time.sleep(0.1)
        #     unity_data = self.AI_node.get_latest_data()

        # 现在处理数据
        flat_list = []
        for value in unity_data.values():
            if isinstance(value, list):
                flat_list.extend(value)
            else:
                flat_list.append(value)
        return np.array(flat_list, dtype=float)
    
    def get_yaw_from_quaternion(self, z, w):
        """从四元数的 z 和 w 分量中提取偏航角（Y 轴旋转）"""
        return np.degrees(2 * np.arctan2(z, w))
    
    def get_direction_vector(self, current_position, target_position):
        """计算从当前位置指向目标位置的向量"""
        return np.array(target_position) - np.array(current_position)
    
    def get_angle_to_target(self, car_yaw, direction_vector):
        # 计算车辆朝向与目标方向之间的角度差
        target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
        angle_diff = target_yaw - np.radians(car_yaw)
        return np.abs(np.degrees(angle_diff)) % 360
    
    def collect_unity_data(self, unityState):
        self.state_detect, token = transfer_obs(unityState)
        if self.state_detect == 1: 
            
            car_pos = token['car_pos']
            target_pos = token['target_pos']
            car_quaternion = token['car_quaternion']
            car_yaw = self.get_yaw_from_quaternion(car_quaternion[0], car_quaternion[1])
            direction_vector = self.get_direction_vector(car_pos, target_pos)
            angle_to_target = self.get_angle_to_target(car_yaw, direction_vector)
            print(angle_to_target)
            del token['car_quaternion']
            # self._process_data(token)

            # new_frame = eval(token)
            # print("frame length : ", len(new_frame))
            # action = []
            # action.append(float(new_frame[-1]))
            # data = Float32MultiArray()
            # data.data = action
            # if(new_frame[-1] == 1):
            #     self.publisher_AINode_2_unity_RESET_thu_ROSbridge.publish(data)
            # else:
            #     self.publisher_AINode_2_unity_thu_ROSbridge.publish(data)
        else:
            print("Unity lidar no signal.....")
            

    def callback_from_Unity(self, msg):
        self.collect_unity_data(msg.data)

    def callback_from_Unity_stop_flag(self, msg):
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        csv_directory = os.path.join('.', 'dataFile')
        csv_file_path = os.path.join(csv_directory, f'lstm_training_{timestamp}.csv')

        os.makedirs(csv_directory, exist_ok=True)
        
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['token'])
            for item in self.tokens:
                csv_writer.writerow([item])

        self.tokens = list()
        self.get_logger().info("Generate data")        

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