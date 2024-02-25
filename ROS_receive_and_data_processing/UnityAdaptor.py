import math
from utils.adaptor_utils import *
from ROS_receive_and_data_processing.lidar_processing import lidar_processing
from utils.rotate_angle import calculate_angle_point

def calculate_distance(car_pos, target_pos):
    """
    計算車輛與目標之間的距離和車輛的朝向。
    """
    car_target_distance = math.sqrt((car_pos[0] - target_pos[0])**2 + (car_pos[1] - target_pos[1])**2)
    return car_target_distance

def collect_features(obs):
    """
    蒐集所有能用的features
    """
    lidar_data, lidar_no_element_detect = lidar_processing(obs)

    wheel_angular_vel = round_to_decimal_places([
        obs['ROS2WheelAngularVelocityLeftBack'][1],
        obs['ROS2WheelAngularVelocityRightBack'][1]
    ])

    car_pos, target_pos = obs['ROS2CarPosition'], obs['ROS2TargetPosition']
    car_quaternion = obs['ROS2CarQuaternion'][2:4]  # 只取 z, w
    car_target_distance = calculate_distance(car_pos, target_pos)

    # 計算相對座標
    relative_coordinates = round_to_decimal_places([target_pos[0] - car_pos[0], target_pos[1] - car_pos[1]])

    if len(wheel_angular_vel) != 2:
        print("error")

    # 計算車頭面向目標角度
    angle_diff = calculate_angle_point(
        car_quaternion[0],
        car_quaternion[1],
        car_pos,
        target_pos
    )

    # 將所有可能會用到的features放入
    features = {
        "car_pos": car_pos,
        "target_pos": target_pos,
        "car_target_distance": car_target_distance,
        "car_quaternion": car_quaternion,
        "lidar_data": lidar_data,
        "relative_coordinates": relative_coordinates,
        "angle_diff": angle_diff,
        "lidar_no_element_detect": lidar_no_element_detect  # 用於判斷目前lidar有沒有數值
    }

    return features

def transfer_obs(obs):
    """
    將從Unity蒐集到的obs轉換成python這端要看的obs
    """
    obs = parse_json_to_dict(obs)
    features = collect_features(obs)

    '''
    定義rule和RL要收到的obs
    '''
    state_dict = {
        "car_pos": trans_to_float(features["car_pos"]),
        "target_pos": trans_to_float(features["target_pos"]),
        "car_target_distance": float(features["car_target_distance"]),
        "car_quaternion": trans_to_float(features["car_quaternion"]),
        "lidar_data": trans_to_float(features["lidar_data"]),
        "angle_diff": trans_to_float([features['angle_diff']])[0],
        "relative_coordinates": trans_to_float(features["relative_coordinates"]),
    }

    return features['lidar_no_element_detect'], state_dict
