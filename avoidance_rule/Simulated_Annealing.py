temperature = 1.0  # 初始温度
cooling_rate = 0.95  # 冷却率
last_turn_direction = None
turn_persistence = 3
bias_counter = 0
from utils.rotate_angle import *
import random

def refined_obstacle_avoidance_with_target_orientation(lidars, car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    global last_turn_direction, turn_persistence, temperature
    print("temperature : ", temperature)
    safe_distance = 0.35
    angle_tolerance = 10  # 度，角度对齐的容忍度
    angle_diff = calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos)
    # [现有代码不变]
    obstacle_near = any(lidar < safe_distance for lidar in lidars)
    if obstacle_near:
        # [现有代码不变]
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

        # 根据当前温度调整左右转概率
        if len(clear_directions) > 1:
            # 使用温度影响转向决策
            if random.random() < temperature:
                # 更高的温度意味着更大的随机性
                temperature *= cooling_rate
                return random.choice(clear_directions)
            else:
                # 低温下倾向于选择直行或最后一次转向
                temperature *= cooling_rate
                return last_turn_direction if last_turn_direction in clear_directions else 0
        elif len(clear_directions) == 1:
            temperature *= cooling_rate
            return clear_directions[0]
        else:
            # [现有代码不变]
            temperature *= cooling_rate
            return random.choice([1, 2])
    else:
        # [现有代码不变]
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
            temperature *= cooling_rate
            return turn_direction
        else:
            temperature *= cooling_rate
            return 0  # 直行

    # 更新温度

# [其他函数不变]
