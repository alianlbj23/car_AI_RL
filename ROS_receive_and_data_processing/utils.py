#  將unity的資料轉換成python可以處理的格式
import json

def round_to_decimal_places(data_list, decimal_places=3):
    """
    Round the elements of a list to a specified number of decimal places.
    """
    return [round(num, decimal_places) for num in data_list]

def trans_to_float(data_list):
    return [float(i) for i in data_list]

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
