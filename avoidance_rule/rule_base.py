from avoidance_rule.avoidance import refined_obstacle_avoidance_with_target_orientation
import rclpy
from utils.obs_utils import *

class RuleBasedController:
    def __init__(self, node):
        self.node = node
        self.data = []
        
    def rule_action(self, obs_for_avoidance):
        action = refined_obstacle_avoidance_with_target_orientation(
            obs_for_avoidance['lidar_data'],
            obs_for_avoidance['car_quaternion'][0],
            obs_for_avoidance['car_quaternion'][1],
            obs_for_avoidance['car_pos'],
            obs_for_avoidance['target_pos']
        )
        return action

    def run(self):
        while rclpy.ok():
            self.node.reset()
            _, unity_data = wait_for_data(self.node)
            action = self.rule_action(unity_data)
            self.data.append(unity_data)
            self.node.publish_to_unity(action)
            terminated = (
                unity_data['car_target_distance'] < 1 or 
                min(unity_data['lidar_data']) < 0.2
            )
            if terminated:
                self.node.publish_to_unity_RESET()
    # def apply_rules(self):
    