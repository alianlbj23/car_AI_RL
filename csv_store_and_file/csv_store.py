import csv
from datetime import datetime
import os

def set_csv_format(action, data_dict):
    data_dict["action"] = action
    return data_dict

def save_data_to_csv(data):
    # 確定有數據
    if not data:
        return

    # 用時間創文件名稱
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'collected_data_{timestamp}.csv'

    # 指定路徑
    current_dir = os.getcwd()  # 抓現在目錄
    path = os.path.join(current_dir, 'training_data')  # 創文件名稱

    if not os.path.exists(path):
        os.makedirs(path)

    full_path = os.path.join(path, filename)

    keys = data[0].keys()  # 取字典的key當作title

    with open(full_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=keys)
        writer.writeheader()
        for i in data:
            writer.writerow(i)
    print("store csv file")