#  給env用的tools
#  主要給RL內的env用的tools
import numpy as np

def data_dict_pop(data_dict):
    '''拿掉自己沒有要丟入obs的features'''
    data_dict.pop('car_quaternion', None)
    data_dict.pop('car_pos', None)
    data_dict.pop('target_pos', None)
    return data_dict

def process_data(unity_data):  
    '''data轉成np後給env的observation'''
    flat_list = []
    for value in unity_data.values():
        if isinstance(value, list):
            flat_list.extend(value)
        else:
            flat_list.append(value)
    return np.array(flat_list, dtype=np.float32)

def wait_for_data(AI_node): 
    '''等待最新的data, 並回傳給reward計算用的state和給obs的state'''
    unity_data = AI_node.get_latest_data()
    
    while unity_data is None:
        unity_data = AI_node.get_latest_data()
    unity_data_for_reward = unity_data.copy()
    unity_data = data_dict_pop(unity_data)
    return unity_data, unity_data_for_reward