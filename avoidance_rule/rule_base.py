from avoidance_rule.Simulated_Annealing import ObstacleAvoidanceController
import rclpy
from utils.obs_utils import *
from csv_store_and_file.csv_store import save_data_to_csv, set_csv_format
# from ROS_receive_and_data_processing.config import 
import time

class RuleBasedController:
    def __init__(self, node, save_to_csv=False):
        self.node = node
        self.data = []
        self.controller = ObstacleAvoidanceController()  
        self.save_to_csv = save_to_csv #  是否寫入csv
        
    def rule_action(self, obs_for_avoidance):
        '''
        跑自己的避障rule
        '''
        action = self.controller.refined_obstacle_avoidance_with_target_orientation(
            obs_for_avoidance['lidar_data'],
            obs_for_avoidance['car_quaternion'][0],
            obs_for_avoidance['car_quaternion'][1],
            obs_for_avoidance['car_pos'],
            obs_for_avoidance['target_pos'],
        )
        return action

    def reset_controller(self):
        '''
        用於結束時清空資料和給Unity端結束訊號
        '''
        time.sleep(1)
        self.data = []
        self.node.publish_to_unity_RESET() #  傳送結束訊號

    def store_data(self, unity_data):
        if self.save_to_csv:
            self.data.append(unity_data)
            
    def run(self):
        while rclpy.ok():
            self.node.reset()
            _, unity_data = wait_for_data(self.node)
            
            #  這邊放你的cool algorithm
            action = self.rule_action(unity_data)
            
            unity_data = set_csv_format(action, unity_data) #  新增action到目前的state, 後續用於寫入csv
            self.store_data(unity_data)
            self.node.publish_to_unity(action)
            
            #  先檢查是否到達目標
            if unity_data['car_target_distance'] < 1:
                if self.save_to_csv:
                    save_data_to_csv(self.data)
                self.reset_controller()
                
            #  檢查是否撞到牆壁
            elif min(unity_data['lidar_data']) < 0.2:
                self.reset_controller()
    