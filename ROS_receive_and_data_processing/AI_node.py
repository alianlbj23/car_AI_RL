#  主要跟Unity收送資料的地方
from ROS_receive_and_data_processing.UnityAdaptor import transfer_obs
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from ROS_receive_and_data_processing.config import ACTION_MAPPINGS

class AI_node(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")#ros2Ai #unity2Ros        
        self.reset_done_flag = 0

        #  如果駐列不為1，會無法收到最新資料拿去判斷
        
        #  從Unity接收資料
        self.subscriber_fromUnity_thu_ROSbridge_ = self.create_subscription(
            String, 
            "/Unity_2_AI", 
            self.callback_from_Unity, 
            1
        )
        
        #  給予Unity action
        self.publisher_AINode_2_unity_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity', 
            1
        )

        #  傳送終止flag，讓unity reset這場
        self.publisher_AINode_2_unity_RESET_thu_ROSbridge = self.create_publisher(
            Float32MultiArray, 
            '/AI_2_Unity_RESET_flag', 
            1
        )
        
        self.latest_data = None
        self.publisher_2_unity_action_flag = 0

    def publish_to_unity(self, action_code): #  送動作用
        '''
        0是前進 1是左轉 2是右轉 3是後退 4是停止
        '''
        if self.publisher_2_unity_action_flag == 1:
            msg = Float32MultiArray()
            action = ACTION_MAPPINGS.get(action_code, "invalid")
            msg.data = action
            self.publisher_AINode_2_unity_thu_ROSbridge.publish(msg)
    
    def publish_to_unity_RESET(self):
        self.get_logger().info("Send reload signal")
        msg = Float32MultiArray()
        msg.data = [1.0] # 送個訊號過去觸發unity的重製，裡面數字沒有意義
        self.publisher_AINode_2_unity_RESET_thu_ROSbridge.publish(msg)
        #  停止傳送動作給unity
        self.publisher_2_unity_action_flag = 0 
        
    def callback_from_Unity(self, msg):
        self.state_detect, data_dict = transfer_obs(msg.data)
        if self.state_detect == 1:
            self.latest_data = data_dict 
        else:
            print("erro")
            pass

    def reset(self):
        self.latest_data = None
        self.publisher_2_unity_action_flag = 1
        
    def get_latest_data(self):
        return self.latest_data
