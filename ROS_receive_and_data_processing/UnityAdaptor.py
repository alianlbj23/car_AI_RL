import math
from utils.adaptor_utils import *
from ROS_receive_and_data_processing.lidar_processing import lidar_processing

def transfer_obs(obs):
    """
    Process observation data from Unity and return a token and a flag indicating lidar detection.
    """
    obs = parse_json_to_dict(obs)

    lidar_data, lidar_no_element_detect = lidar_processing(obs)
    

    wheel_angular_vel = round_to_decimal_places([
        obs['ROS2WheelAngularVelocityLeftBack'][1],
        obs['ROS2WheelAngularVelocityRightBack'][1]
    ])

    car_quaternion = round_to_decimal_places(obs['ROS2CarQuaternion'][2:4]) #只取z w
    car_pos, target_pos = obs['ROS2CarPosition'], obs['ROS2TargetPosition']
    car_target_distance = (car_pos[0] - target_pos[0])**2 + (car_pos[1] - target_pos[1])**2
    car_target_distance = round_to_decimal_places([math.sqrt(car_target_distance)])[0]
    
    
    if len(wheel_angular_vel) != 2:
        print("error")
        
    '''
    這邊定義state, reward計算也會基於以下這些state去計算, 要改RL收到的obs可以在
    utils的obs_utils裡面data_dict_pop做修改
    '''
    state_dict = {
        "car_pos": trans_to_float(car_pos),
        "target_pos": trans_to_float(target_pos),
        "car_target_distance": float(car_target_distance),
        "car_quaternion": trans_to_float(car_quaternion),
        "lidar_data": trans_to_float(lidar_data),
        "relative_coordinates": trans_to_float(round_to_decimal_places(
                                                [target_pos[0]-car_pos[0],target_pos[1]-car_pos[0]])
                                               )
    }
    
    return lidar_no_element_detect, state_dict
