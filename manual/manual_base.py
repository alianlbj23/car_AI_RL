import rclpy
from utils.obs_utils import *
import tkinter as tk
from utils.rotate_angle import calculate_angle_point
import time
from csv_store_and_file.csv_store import save_data_to_csv, set_csv_format
import sys

class ManualBasedController:
    def __init__(self, node, save_to_csv=True):
        self.node = node
        self.action = -1  # 4 表示沒動作
        self.data = []
        
        # 創 Tkinter 窗口
        self.root = tk.Tk()
        self.root.title("Manual Control")
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.save_to_csv = save_to_csv

    def on_key_press(self, event):
        if event.keysym == 'w':
            self.action = 0  # 前進
        elif event.keysym == 'a':
            self.action = 1  # 左轉
        elif event.keysym == 'd':
            self.action = 2  # 右轉
        elif event.keysym == 's':
            self.action = 3  # 後退
        elif event.keysym == 'q':
            self.quit_program()
            
    def quit_program(self):
        self.root.destroy() 
        sys.exit()  
        
    def on_key_release(self, event):
        self.action = 4  # 停止

    def reset_controller(self):
        time.sleep(1)
        self.data = []
        self.node.publish_to_unity_RESET()
        
    def run(self):
        while rclpy.ok():
            if self.action != -1:
                self.node.reset()
                self.node.publish_to_unity(self.action)
                _, unity_data = wait_for_data(self.node)
                unity_data = set_csv_format(self.action, unity_data) #  存csv用
                self.data.append(unity_data)
                angle_diff = calculate_angle_point(
                    unity_data['car_quaternion'][0], 
                    unity_data['car_quaternion'][1], 
                    unity_data['car_pos'], 
                    unity_data['target_pos']
                )
                if unity_data['car_target_distance'] < 1:
                    if self.save_to_csv:
                        save_data_to_csv(self.data)
                    self.reset_controller()
                    
                elif min(unity_data['lidar_data']) < 0.2:
                    self.reset_controller()
            self.root.update()  # 更新 Tkinter 窗口