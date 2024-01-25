#  處理lidar的資料
from utils.adaptor_utils import *
from ROS_receive_and_data_processing.config import LIDAR_RANGE


def get_smallest_lidar_values(lidar_data):
    """
    將lidar原本360個距離數值區分成n個區域, 每個區域只抓最小的數值
    """
    n = LIDAR_RANGE
    chunk_size = len(lidar_data) // n
    min_value_list = []

    for i in range(0, len(lidar_data), chunk_size):
        chunk = lidar_data[i:i + chunk_size]
        min_value = min(chunk)
        min_value_list.append(min_value)

    return min_value_list   

def lidar_processing(obs):
    '''將lidar資料過濾後並取到小數第3位'''
    lidar_data = obs.get('ROS2Range', [])
    lidar_no_element_detect = int(bool(lidar_data)) #  判斷有沒有收到lidar資料
    lidar_data_direction = obs['ROS2RangePosition'] #  因為都沒用到lidar的vector，若有需要可自行使用
    lidar_data = get_smallest_lidar_values(lidar_data)
    lidar_data = round_to_decimal_places(lidar_data)
    return lidar_data, lidar_no_element_detect
    
    