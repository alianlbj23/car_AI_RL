import rclpy
from utils.obs_utils import *
import tkinter as tk

class ManualBasedController:
    def __init__(self, node):
        self.node = node
        self.action = 4  # -1 表示沒動作

        # 创建 Tkinter 窗口
        self.root = tk.Tk()
        self.root.title("Manual Control")
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def on_key_press(self, event):
        if event.keysym == 'w':
            self.action = 0  # 前進
        elif event.keysym == 'a':
            self.action = 1  # 左轉
        elif event.keysym == 'd':
            self.action = 2  # 右轉
        elif event.keysym == 's':
            self.action = 3  # 後退

    def on_key_release(self, event):
        self.action = 4  # 停止

    def run(self):
        while rclpy.ok():
            if self.action != -1:
                self.node.reset()
                self.node.publish_to_unity(self.action)
                _, unity_data = wait_for_data(self.node)
                print(unity_data['lidar_data'])
                terminated = (
                    unity_data['car_target_distance'] < 1 or 
                    min(unity_data['lidar_data']) < 0.2
                )
                # if terminated:
                #     self.node.publish_to_unity_RESET()
            self.root.update()  # 更新 Tkinter 窗口


