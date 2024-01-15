from avoidance_rule.avoidance import refined_obstacle_avoidance_with_target_orientation
# from avoidance_rule.avoidance_90 import refined_obstacle_avoidance_with_target_orientation
import rclpy
from utils.obs_utils import *
from csv_store_and_file.csv_store import save_data_to_csv, set_csv_format
import time

class RuleBasedController:
    def __init__(self, node):
        self.node = node
        self.data = []
        self.last_turn_direction = 0  
        
    def rule_action(self, obs_for_avoidance):
        action = refined_obstacle_avoidance_with_target_orientation(
            obs_for_avoidance['lidar_data'],
            obs_for_avoidance['car_quaternion'][0],
            obs_for_avoidance['car_quaternion'][1],
            obs_for_avoidance['car_pos'],
            obs_for_avoidance['target_pos'],
        )
        return action

    def run(self):
        while rclpy.ok():
            self.node.reset()
            _, unity_data = wait_for_data(self.node)
            action = self.rule_action(unity_data)
            self.last_turn_direction = action
            unity_data = set_csv_format(action, unity_data)
            self.data.append(unity_data)
            
            self.node.publish_to_unity(action)
            terminated = (
                unity_data['car_target_distance'] < 1 or 
                min(unity_data['lidar_data']) < 0.2
            )
            if terminated:
                # save_data_to_csv(self.data) # 將資料存入csv
                time.sleep(1)
                self.data = []
                self.node.publish_to_unity_RESET()
    # def apply_rules(self):
    