import json
import math
import numpy as np

def parse_json_to_dict(json_str):
    """
    Convert a JSON string to a dictionary, processing special string formats into coordinate lists.
    """
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError:
        return {}

    for key, value in data.items():
        if isinstance(value, str) and value.startswith('(') and value.endswith(')'):
            coordinates = list(map(float, value.strip('()').split(',')))  
            data[key] = coordinates
    return data

def get_90_smallest_lidar_values(lidar_data):
    """
    Divide the lidar data into 12 chunks and return the smallest value from each chunk.
    """
    chunk_size = len(lidar_data) // 15
    return [min(lidar_data[i:i + chunk_size]) for i in range(0, len(lidar_data), chunk_size)]

#  將lidar vector做字串處理
def parse_direction_string(direction_str):
    stripped_str = direction_str.strip("()")
    return [float(num) for num in stripped_str.split(",")]

#  將lidar distance和vector壓縮成12等份，並挑出12等份最小的數值
def get_smallest_lidar_values_with_direction(lidar_data, lidar_directions):
    """
    Divide the lidar data into 15 chunks and return the smallest value from each chunk
    along with its direction.
    """
    chunk_size = len(lidar_data) // 8
    result = []
    min_value_list = []
    min_direction_list = []

    for i in range(0, len(lidar_data), chunk_size):
        chunk = lidar_data[i:i + chunk_size]
        min_value = min(chunk)
        min_index = i + chunk.index(min_value)
        min_direction = lidar_directions[min_index]
        
        min_value_list.append(min_value)
        min_direction_list.append(min_direction)

    return min_value_list, min_direction_list

def normalize_lidar_values(lidar_data):
    """
    Normalize the lidar data to a range of 0 to 1.
    """
    min_val, max_val = min(lidar_data), max(lidar_data)
    return [(x - min_val) / (max_val - min_val) if max_val - min_val else 0 for x in lidar_data]

def round_to_decimal_places(data_list, decimal_places=3):
    """
    Round the elements of a list to a specified number of decimal places.
    """
    return [round(num, decimal_places) for num in data_list]

def clamp_number(number, min_val=-10, max_val=10):
    """
    Clamp the number to a specified range.
    """
    return max(min_val, min(max_val, number))

def quaternion_to_car_orientation(x, y, z, w):
    # 左邊0~180 右邊 0~-180
    length = math.sqrt(x*x + y*y + z*z + w*w)
    x /= length
    y /= length
    z /= length
    w /= length
    
    # 計算角度（弧度）
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    yaw_degrees = math.degrees(yaw)
    
    return yaw_degrees

def calculate_angle(coord1, coord2):
    vector1 = np.array(coord1)
    vector2 = np.array(coord2)

    dot_product = np.dot(vector1, vector2)

    norm1 = np.linalg.norm(vector1)
    norm2 = np.linalg.norm(vector2)

    # 算夾角
    angle = np.arccos(dot_product / (norm1 * norm2))

    # rad轉成度
    angle_deg = np.degrees(angle)

    return angle_deg

def trans_to_float(data_list):
    return [float(i) for i in data_list]

def transfer_obs(obs):
    """
    Process observation data from Unity and return a token and a flag indicating lidar detection.
    """
    try:
        obs = parse_json_to_dict(obs)
    except json.JSONDecodeError:
        return 0, {}

    lidar_data = obs.get('ROS2Range', [])
    lidar_no_element_detect = int(bool(lidar_data))

    #這邊要做實驗
    # if lidar_data:
    #     lidar_data = normalize_lidar_values(lidar_data)
    lidar_data_direction = obs['ROS2RangePosition']
    lidar_data, lidar_data_direction = get_smallest_lidar_values_with_direction(lidar_data, lidar_data_direction)
    lidar_data = round_to_decimal_places(lidar_data)
    #  將選出來的lidar vector轉換成1維list
    converted_min_directions = [parse_direction_string(direction) for direction in lidar_data_direction]
    flattened_directions = [num for sublist in converted_min_directions for num in sublist]
    flattened_directions = round_to_decimal_places(flattened_directions)

    wheel_angular_vel = [
        obs['ROS2WheelAngularVelocityLeftBack'][1],
        obs['ROS2WheelAngularVelocityRightBack'][1]
    ]
    wheel_angular_vel = round_to_decimal_places(wheel_angular_vel)

    car_quaternion = round_to_decimal_places(obs['ROS2CarQuaternion'][2:4]) #只取z w
    car_steering_angle = [quaternion_to_car_orientation(0,0,car_quaternion[0], car_quaternion[1])]
    car_steering_angle = round_to_decimal_places(car_steering_angle)

    car_pos, target_pos = obs['ROS2CarPosition'], obs['ROS2TargetPosition']
    car_target_distance = (car_pos[0] - target_pos[0])**2 + (car_pos[1] - target_pos[1])**2
    car_target_distance = round_to_decimal_places([math.sqrt(car_target_distance)])[0]
    
    
    if len(wheel_angular_vel) != 2:
        print("error")

    state_dict = {
        "car_pos": trans_to_float(car_pos),
        "target_pos": trans_to_float(target_pos),
        "car_target_distance": float(car_target_distance),
        "car_quaternion": trans_to_float(car_quaternion),
        "lidar_data": trans_to_float(lidar_data),
        "relative_coordinates": trans_to_float([target_pos[0]-car_pos[0],target_pos[1]-car_pos[0]])
        # "flattened_directions": trans_to_float(flattened_directions), #  lidar direction
    }
    return lidar_no_element_detect, state_dict
