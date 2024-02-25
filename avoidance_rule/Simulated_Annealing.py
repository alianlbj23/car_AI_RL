import random
from utils.rotate_angle import calculate_angle_point
from ROS_receive_and_data_processing.config import (
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
    OBSTACLE_DISTANCE,
)


class ObstacleAvoidanceController:

    def refined_obstacle_avoidance_with_target_orientation(self, lidars, angle_diff):
        safe_distance = OBSTACLE_DISTANCE
        angle_tolerance = 10  # degrees, tolerance for angle alignment

        angle_diff = angle_diff  #  計算面向目標角度
        obstacle_near = any(lidar < safe_distance for lidar in lidars)

        if obstacle_near:
            #  90個lidar
            front_clear = all(lidars[i] > safe_distance for i in FRONT_LIDAR_INDICES)
            left_clear = all(lidars[i] > safe_distance for i in LEFT_LIDAR_INDICES)
            right_clear = all(lidars[i] > safe_distance for i in RIGHT_LIDAR_INDICES)
            if front_clear:
                return 0
            if right_clear:
                return 2
            if left_clear:
                return 1
        else:
            if abs(angle_diff) > angle_tolerance:
                if angle_diff < 0:
                    return 2
                else:
                    return 1
            else:
                return 0  # forward
