#  處理lidar的資料
from ROS_receive_and_data_processing.utils import *

def get_smallest_lidar_values(lidar_data):
    """
    Divide the lidar data into 8 chunks and return the smallest value from each chunk.
    """
    chunk_size = len(lidar_data) // 8
    min_value_list = []

    for i in range(0, len(lidar_data), chunk_size):
        chunk = lidar_data[i:i + chunk_size]
        min_value = min(chunk)
        min_value_list.append(min_value)

    return min_value_list   

def lidar_processing(obs):
    lidar_data = obs.get('ROS2Range', [])
    lidar_no_element_detect = int(bool(lidar_data)) #  判斷有沒有收到lidar資料
    lidar_data_direction = obs['ROS2RangePosition'] #  因為都沒用到lidar的vector，若有需要可自行使用
    lidar_data = get_smallest_lidar_values_with_direction(lidar_data, lidar_data_direction)
    lidar_data = round_to_decimal_places(lidar_data)
    return lidar_data, lidar_no_element_detect
    
    