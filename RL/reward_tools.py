#  reward_cal的計算工具
import numpy as np

#  處理偏行角計分 (180 +- 30 可以加分)
def get_yaw_from_quaternion(z, w):
    '''從四位數的z、w算出偏航角'''
    return np.degrees(2 * np.arctan2(z, w))
    
def get_direction_vector(current_position, target_position):
    '''計算現在位置指向目標的向量'''
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    '''計算car車頭與target之間的角度差'''
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return (np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    '''透過計算車頭和目標的角度差給予reward'''
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    point = 0
    
    angle_diff = (angle_to_target - 180) # 調整角度
    if 0 <= angle_diff <= 20: #  若角度差距在20以內就線性加分，越接近0給分越多
        point = 15 * (1 - (angle_diff / 30))

    return point

def calculate_distance_change(current_distance, threshold):
    '''計算與目標之間的距離和上次的距離比較，
    但因為傳輸過快的關係, 距離差幾乎趨近0, 因此這個自行斟酌使用
    '''
    point = 0
    if current_distance > threshold:
        point = -(current_distance - threshold)*15 

    if 2 < current_distance <= threshold:
        point = 20 * (threshold - current_distance)
    elif current_distance < 2:
        point = 50 * (3 - current_distance) ** 2
    elif current_distance < 1:
        point = 10000
    return point

def calculate_lidar_based_reward(lidar_data, safe_distance):
    '''lidar偵測障礙物後計算reward'''
    
    min_distance = min(lidar_data)  # 抓lidar最靠近物體的距離

    # 如果距離小於安全距離，要給懲罰
    if min_distance < safe_distance:
        distance_diff = safe_distance - min_distance
        # 用指數型給負分
        point = -np.exp(distance_diff * 10)*50  #  50的倍率可以自行調整
    else:
        point = 0
    return point