# config.py

# vel和rotate_vel是mapping到Unity輪子轉動單位(度/秒)
vel = 0.5
rotate_vel = 5.0
#  這邊可以自行定義動作
ACTION_MAPPINGS = {
    0: [vel, vel, vel, vel],  # 前進
    1: [-rotate_vel, rotate_vel,-rotate_vel,rotate_vel],    # 左轉
    2: [rotate_vel, -rotate_vel,rotate_vel,-rotate_vel],     # 右轉
    3: [-vel, -vel, -vel, -vel], # 後退
    4: [0.0,0.0,0.0,0.0]      # 停止
}

LIDAR_RANGE = 90
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(range(-15, 0)) #0~16個和-15到0的LIDAR RANGE
LEFT_LIDAR_INDICES = list(range(16, 46))
RIGHT_LIDAR_INDICES = list(range(-45, -15)) 
