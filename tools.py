#  計算reward的工具
import numpy as np

#  處理偏行角計分 (180 +- 30 可以加分)
def get_yaw_from_quaternion(z, w):
        """从四元数的 z 和 w 分量中提取偏航角（Y 轴旋转）"""
        return np.degrees(2 * np.arctan2(z, w))
    
def get_direction_vector(current_position, target_position):
    """计算从当前位置指向目标位置的向量"""
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    #  計算car與target之間的角度差
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return np.abs(np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    point = 0

    angle_diff = np.abs(angle_to_target - 180)
    if 0 <= angle_diff <= 20:
        point = 15 * (1 - (angle_diff / 30))
    # else:
    #     point =  -10 * (1 + (angle_diff / 30))
    return point


#  計算與目標之間的距離和上次的距離比較
def calculate_distance_change(current_distance, threshold):
    #  因為傳輸過快的關係，距離差幾乎趨近0，不要用距離差
    point = 0
    if current_distance > threshold:
        point = -(current_distance - threshold)*15 

    if 2 < current_distance <= threshold:
        # 距离在3到4之间，给予小量奖励，奖励随着距离减小而增加
        point = 20 * (threshold - current_distance)
    elif current_distance < 2:
        # 距离小于3时，给予较大奖励，特别是在距离接近1.5时，奖励更大
        point = 50 * (3 - current_distance) ** 2
    elif current_distance < 1:
        point = 10000
    return point


def calculate_lidar_based_reward(lidar_data, safe_distance):
    # 获取 LiDAR 数据中最近障碍物的距离
    min_distance = min(lidar_data)

    # 如果距离小于安全距离，则进行指数型惩罚
    if min_distance < safe_distance:
        # 计算距离差
        distance_diff = safe_distance - min_distance
        # 应用指数型惩罚
        point = -np.exp(distance_diff * 10)*50  # 惩罚系数可以根据需要调整
    else:
        # 距离大于安全距离时，没有惩罚
        point = 0
    return point

#  駕車穩定性
def calculate_steering_change(current_steering_angle, previous_steering_angle):
    angle_change = current_steering_angle - previous_steering_angle

    # 处理角度的跳变
    if angle_change > 180:
        angle_change -= 360
    elif angle_change < -180:
        angle_change += 360

    return angle_change

#  防左右擺頭 
def calculate_drive_reward(current_direction, previous_direction):
    point = 0
    if previous_direction == None:
        point = 0
    elif current_direction == previous_direction:
        # 如果连续向同一方向转向，给予奖励
        point = 10
    else:
        # 如果频繁改变转向方向，给予惩罚
        point = -30

    return point

#  輪速評分
def calculate_wheel_speed_reward(left_wheel_speed, right_wheel_speed):
    speed_difference = abs(left_wheel_speed - right_wheel_speed)
    stable_speed_threshold = 5  # 穩定數值

    if speed_difference < stable_speed_threshold:
        # 輪速差異小，平穩駕駛
        reward = 3
    else:
        # 不穩駕駛
        reward = -10

    return reward

#  計算是否一直重複
def check_spinning(action_history, threshold):
    point = 0
    action_history = list(action_history)
    if len(action_history) >= threshold:
        last_actions = action_history[-threshold:]
        if len(set(last_actions)) == 1:
            # 如果最后 'threshold' 个动作都相同，则扣分
            point = -30
        elif len(set(action_history)) == 1:
            # 如果所有动作都相同，则扣更多分
            point = -100
    return point