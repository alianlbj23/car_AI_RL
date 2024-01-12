#  計算reward用
from RL.reward_tools import *
def reward_calculate(state_dict):

    reward = 0
    car_pos = state_dict['car_pos']
    car_quaternion = state_dict['car_quaternion']
    target_pos = state_dict['target_pos']
    current_distance = state_dict['car_target_distance']
    lidar_data = state_dict['lidar_data']
    
    #  與目標距離
    reward += calculate_distance_change(current_distance, 1.5)

    #  lidar
    reward += calculate_lidar_based_reward(lidar_data, 0.45)*100

    #  利用偏行角算分
    reward += calculate_angle_point(car_quaternion[0], car_quaternion[1], car_pos, target_pos)*10
    
    return reward