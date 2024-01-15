import csv
from datetime import datetime
import os

def set_csv_format(action, data_dict):
    data_dict["action"] = action
    return data_dict

def save_data_to_csv(data):
    # 确保有数据要保存
    if not data:
        return

    # 使用当前时间戳创建文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'collected_data_{timestamp}.csv'

    # 指定保存路径
    current_dir = os.getcwd()  # 获取当前目录
    path = os.path.join(current_dir, 'training_data')  # 创建目标文件夹路径

    # 检查文件夹是否存在，如果不存在则创建
    if not os.path.exists(path):
        os.makedirs(path)

    full_path = os.path.join(path, filename)

    keys = data[0].keys()  # 获取字典的键作为列标题

    with open(full_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=keys)
        writer.writeheader()
        for i in data:
            writer.writerow(i)
    print("store csv file")