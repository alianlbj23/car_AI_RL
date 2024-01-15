import math
import numpy as np

def get_yaw_from_quaternion(z, w):
    """从四元数的 z 和 w 分量中提取偏航角（Y 轴旋转）"""
    return np.degrees(2 * np.arctan2(z, w))

def get_direction_vector(current_position, target_position):
    """计算从当前位置指向目标位置的向量"""
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    """计算汽车与目标之间的角度差"""
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)
    return (np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    """计算汽车面向目标的角度差"""
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    angle_diff = angle_to_target - 180
    return angle_diff

def refined_obstacle_avoidance_with_target_orientation(lidar_data, car_quaternion_z, car_quaternion_w, car_pos, target_pos):
    """根据 LiDAR 数据和车辆朝向决定避障动作"""
    safety_threshold = 0.7  # 安全距离阈值

    # 计算汽车面对目标的角度差
    angle_to_target = calculate_angle_point(car_quaternion_z, car_quaternion_w, car_pos, target_pos)
    
    if min(lidar_data) >= safety_threshold:
        if abs(angle_to_target) > 10:  # 假设超过10度需要调整方向
            return 1 if angle_to_target > 0 else 2  # 1表示左转，2表示右转
        else:
            return 0  # 前进
    
    # 如果存在障碍物，执行避障动作
    else:
        return decide_turn_direction(lidar_data, angle_to_target)

def decide_turn_direction(lidar_data, angle_to_target):
    """基于LiDAR数据和目标方向决定转向"""
    safety_threshold = 0.7
    left_region = lidar_data[16:31]  # 左侧区域
    front_region = lidar_data[15:-15]  # 前方区域
    right_region = lidar_data[60:75]  # 右侧区域
    min_left = min(left_region)
    min_front = min(front_region)
    min_right = min(right_region)
    
    # 如果前方有障碍物
    if min_front < safety_threshold: 
        if min_left < min_right:
            return 2  # 右转
        else:
            return 1  # 左转
    # 如果没有直接前方的障碍物，但需要调整方向
    elif abs(angle_to_target) > 10:
        return 1 if angle_to_target > 0 else 2  # 根据角度差调整方向
    else:
        return 0  # 前进