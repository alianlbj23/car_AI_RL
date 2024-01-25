import random
from utils.rotate_angle import calculate_angle_point
import pickle
from ROS_receive_and_data_processing.config import FRONT_LIDAR_INDICES, LEFT_LIDAR_INDICES, RIGHT_LIDAR_INDICES
import os
class ObstacleAvoidanceController:
    def __init__(self, initial_temperature=1.0, cooling_rate=0.95):
        self.temperature = initial_temperature
        self.cooling_rate = cooling_rate
        self.last_turn_direction = None
        self.turn_persistence = 3
        self.bias_counter = 0
    
    def load_parameters(self, filename):
        with open(filename, 'rb') as file:
            parameters = pickle.load(file)
            self.temperature = parameters['temperature']
            self.last_turn_direction = parameters['last_turn_direction']
            self.turn_persistence = parameters['turn_persistence']
            self.bias_counter = parameters['bias_counter']
            
    def check_file(self, filename):
        directory = os.path.dirname(filename)
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            
    def save_parameters(self, filename):
        self.check_file(filename)
        with open(filename, 'wb') as file:
            pickle.dump({
                'temperature': self.temperature,
                'last_turn_direction': self.last_turn_direction,
                'turn_persistence': self.turn_persistence,
                'bias_counter': self.bias_counter
            }, file)
            
    def refined_obstacle_avoidance_with_target_orientation(self, lidars, car_quaternion_1, car_quaternion_2, car_pos, target_pos):
        # print("temperature : ", self.temperature)
        safe_distance = 0.7
        angle_tolerance = 10  # degrees, tolerance for angle alignment

        angle_diff = calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos)
        obstacle_near = any(lidar < safe_distance for lidar in lidars)
        
        if obstacle_near:
            # 找安全方向
            #  8個lidar的版本
            # front_clear = lidars[0] > safe_distance and lidars[7] > safe_distance
            # left_clear = all(lidar > safe_distance for lidar in lidars[1:4])
            # right_clear = all(lidar > safe_distance for lidar in lidars[4:7])
            
            #  90個lidar
            front_clear = all(lidars[i] > safe_distance for i in FRONT_LIDAR_INDICES)
            left_clear = all(lidars[i] > safe_distance for i in LEFT_LIDAR_INDICES)
            right_clear = all(lidars[i] > safe_distance for i in RIGHT_LIDAR_INDICES)
            clear_directions = []
            if front_clear:
                clear_directions.append(0)  # 前進
            if left_clear:
                clear_directions.append(1)  # 左
            if right_clear:
                clear_directions.append(2)  # 右

            # 用溫度影響決策
            if len(clear_directions) > 1:
                if random.random() < self.temperature:
                    self.temperature *= self.cooling_rate
                    return random.choice(clear_directions)
                else:
                    self.temperature *= self.cooling_rate
                    return self.last_turn_direction if self.last_turn_direction in clear_directions else 0
            elif len(clear_directions) == 1:
                self.temperature *= self.cooling_rate
                return clear_directions[0]
            else:
                self.temperature *= self.cooling_rate
                return random.choice([1, 2])
        else:
            if abs(angle_diff) > angle_tolerance:
                if self.last_turn_direction is None or self.turn_persistence == 0:
                    self.last_turn_direction = 1 if angle_diff > 0 else 2
                    self.turn_persistence = 3
                else:
                    self.turn_persistence -= 1
                self.temperature *= self.cooling_rate
                return self.last_turn_direction
            else:
                self.temperature *= self.cooling_rate
                return 0  # forward
