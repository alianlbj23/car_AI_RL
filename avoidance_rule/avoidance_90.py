#  基於8個數值的lidar做的rule
import numpy as np
import random

def refined_obstacle_avoidance_with_target_orientation(lidars,
                                                        car_quaternion_1, 
                                                        car_quaternion_2,
                                                        car_pos,
                                                        target_pos,
                                                        ):
    safe_distance = 0.3  # meters or as per lidar unit
    angle_tolerance = 10  # degrees, the tolerance for angle alignment
    turn_persistence = 3
    last_turn_direction = None
    # Calculate the smallest angle difference to the target, considering the circular nature of angles
    angle_diff = calculate_angle_point(car_quaternion_1,
                                       car_quaternion_2,
                                       car_pos,
                                       target_pos)

    # Check if any lidar reading indicates a close obstacle
    obstacle_near = any(lidar < safe_distance for lidar in lidars)
    # If an obstacle is detected, switch to obstacle avoidance mode
    if obstacle_near:
        front_clear = lidars[0] > safe_distance and lidars[7] > safe_distance
        left_clear = all(lidar > safe_distance for lidar in lidars[1:4])
        right_clear = all(lidar > safe_distance for lidar in lidars[4:7])        
        # Decide on movement based on clear path
        clear_directions = []
        if front_clear:
            clear_directions.append(0) 
        if left_clear:
            clear_directions.append(1)  # Turn left
        if right_clear:
            clear_directions.append(2)  # Turn right
            
        if len(clear_directions) > 1:
            return random.choice(clear_directions)
        elif len(clear_directions) == 1:
            return clear_directions[0]
        else:
            return random.choice([1, 2])
        #     return 3  # No clear path, reverse
    else:
        # No obstacle near, align and move towards the target
        if np.abs(angle_diff) > angle_tolerance:
            if np.abs(angle_diff) > 0.3:
                # 检查是否需要改变方向
                if last_turn_direction is None or turn_persistence == 0:
                    # 决定新的转向方向
                    if angle_diff > 0:
                        turn_direction = 1  # 向左转
                    else:
                        turn_direction = 2  # 向右转
                    last_turn_direction = turn_direction
                    turn_persistence = 3  # 重置转向持续性计数器
                else:
                    turn_direction = last_turn_direction
                    turn_persistence -= 1  # 减少转向持续性计数器
                return turn_direction
            else:
                return 0  # 直行
        else:
            return 0  # 直行
        
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
    

