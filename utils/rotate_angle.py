import numpy as np
def get_yaw_from_quaternion(z, w):
    '''四位數的z、w取得偏行角'''
    return np.degrees(2 * np.arctan2(z, w))
    
def get_direction_vector(current_position, target_position):
    '''計算目前車體位置指向目標的vector'''
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    '''計算car與target之間的角度差'''
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return (np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    '''回傳車頭面對目標的角度, 左轉是0~-180, 右轉是0~180, 越接近0代表車頭越正面於目標'''
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    angle_diff = (angle_to_target - 180) % 360
    if angle_diff > 180:
        angle_diff -= 360   
    return angle_diff