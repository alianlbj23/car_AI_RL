#  計算reward的工具
import numpy as np

#  處理偏行角計分 (180 +- 60 可以加分)
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

#  計算與目標之間的距離和上次的距離比較
def calculate_distance_change(current_distance, last_car_target_distance):
    point = 0
    if current_distance < 1:
        point += 1000
    elif current_distance < 1.5:
        point += 30
    elif current_distance < 2:
        point += 20
    elif current_distance < 2.5:
        point += 5
    elif current_distance > 3:
        point -= 20
    if current_distance < last_car_target_distance:
        point += 5
    elif current_distance > last_car_target_distance:
        point -= 30
    return point

def calculate_lidar_based_reward(lidar_data, safe_distance):
    point = 0
    for lidar_distance in lidar_data:
        if lidar_distance < safe_distance:
            point -= 30

    if min(lidar_data) < 0.4:
        point -= 1000

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

def calculate_drive_reward(current_steering_angle, previous_steering_angle, stable_threshold=10):
    angle_change = calculate_steering_change(current_steering_angle, previous_steering_angle)
    # 平穩駕駛獎勵
    if abs(angle_change) < stable_threshold:
        reward = 5  # 平穩
    else:
        reward = -10  # 不平穩

    return reward

#  輪速評分
def calculate_wheel_speed_reward(left_wheel_speed, right_wheel_speed):
    speed_difference = abs(left_wheel_speed - right_wheel_speed)
    stable_speed_threshold = 5  # 穩定數值

    if speed_difference < stable_speed_threshold:
        # 輪速差異小，平穩駕駛
        reward = 10
    else:
        # 不穩駕駛
        reward = -10

    return reward

def check_spinning(action_history, threshold):
    if len(action_history) < 10:
        return False  # 如果动作历史不足20步，直接返回False

    # 检查动作是否足够相似
    # 这里以简单的欧几里得距离作为示例，您可以根据需要调整相似性的判断标准
    first_action = np.array(action_history[0])
    for action in action_history:
        if np.linalg.norm(np.array(action) - first_action) < threshold:
            print("重複動作!!!!")
            return True
    return False