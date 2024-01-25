#  基於8個數值的lidar做的rule
import numpy as np
import random
last_turn_direction = None  # 上一次转向方向
turn_persistence = 3  # 转向持续性计数器
bias_counter = 0
def refined_obstacle_avoidance_with_target_orientation(lidars, car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    global last_turn_direction, turn_persistence, bias_counter

    safe_distance = 0.35  # 米，或根据激光雷达单位
    angle_tolerance = 10  # 度，角度对齐的容忍度

    # 计算与目标的最小角度差
    angle_diff = calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos)

    # 检查激光雷达是否指示附近有障碍物
    obstacle_near = any(lidar < safe_distance for lidar in lidars)

    if obstacle_near:
        front_clear = lidars[0] > safe_distance and lidars[7] > safe_distance
        left_clear = all(lidar > safe_distance for lidar in lidars[1:4])
        right_clear = all(lidar > safe_distance for lidar in lidars[4:7])

        clear_directions = []
        if front_clear:
            clear_directions.append(0)  # 前进
        if left_clear:
            clear_directions.append(1)  # 左转
        if right_clear:
            clear_directions.append(2)  # 右转

        if len(clear_directions) > 1:
            if bias_counter % 2 == 0:  # 偶数次，增加向右转的概率
                turn_choices = [direction for direction in clear_directions if direction != 1]  # 减少左转
            else:  # 奇数次，增加向左转的概率
                turn_choices = [direction for direction in clear_directions if direction != 2]  # 减少右转

            # 如果 turn_choices 为空，则使用原来的 clear_directions
            if not turn_choices:
                turn_choices = clear_directions

            return random.choice(turn_choices)
        elif len(clear_directions) == 1:
            return clear_directions[0]
        else:
            # 如果没有清晰的方向，随机选择旋转方向
            return random.choice([1, 2])
    
    else:
        if np.abs(angle_diff) > angle_tolerance:
            if last_turn_direction is None or turn_persistence == 0:
                if angle_diff > 0:
                    turn_direction = 1  # 向左转
                else:
                    turn_direction = 2  # 向右转
                last_turn_direction = turn_direction
                turn_persistence = 3
            else:
                turn_direction = last_turn_direction
                turn_persistence -= 1
            return turn_direction
        else:
            return 0  # 直行
    bias_counter += 1

        
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

    return (np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    angle_diff = (angle_to_target - 180) % 360
    if angle_diff > 180:
        angle_diff -= 360   
    return angle_diff
    

