def obstacle_avoidance_with_target_orientation(lidars, current_angle, target_angle):
    """
    Determine movement based on 8 lidar distance readings to avoid obstacles,
    while orienting the vehicle towards a target.
    
    :param lidars: List of 8 distance readings from lidars placed around the vehicle.
    :param current_angle: The current orientation angle of the vehicle (degrees).
    :param target_angle: The desired orientation angle towards the target (degrees).
    :returns: Movement direction (0: Forward, 1: Turn Left, 2: Turn Right, 3: Backward)
    """
    # Define thresholds for obstacle distance and angle difference
    safe_distance = 1.0  # meters or as per lidar unit
    critical_distance = 0.5  # meters or as per lidar unit
    angle_tolerance = 10  # degrees, the tolerance for angle alignment

    # Calculate the smallest angle difference to the target, considering the circular nature of angles
    angle_diff = (target_angle - current_angle + 180) % 360 - 180
    angle_diff = abs(angle_diff)  # Make it absolute for comparison

    # Check for critical immediate action
    if min(lidars) < critical_distance:
        return 3  # Reverse immediately if something is too close

    # Decision making based on lidar readings
    front_clear = lidars[0] > safe_distance and lidars[7] > safe_distance
    left_clear = all(lidar > safe_distance for lidar in lidars[1:4])
    right_clear = all(lidar > safe_distance for lidar in lidars[4:7])

    # Orient towards target if not aligned and path is clear
    if angle_diff > angle_tolerance:
        if front_clear:
            # Choose to rotate left or right based on the shortest path to target angle
            return 1 if (target_angle - current_angle + 360) % 360 < 180 else 2
        else:
            # If not clear, but need to orient towards target, prefer safer side
            if left_clear:
                return 1  # Turn left if left is clearer
            elif right_clear:
                return 2  # Turn right if right is clearer
            else:
                return 3  # No clear path, reverse
    else:
        # If aligned with target, proceed with obstacle avoidance
        if front_clear:
            return 0  # Forward
        elif left_clear:
            return 1  # Turn left
        elif right_clear:
            return 2  # Turn right
        else:
            return 3  # No clear path, reverse

# Example use case
lidar_readings = [1.5, 1.2, 1.0, 0.8, 1.5, 1.2, 1.0, 1.5]  # example lidar readings
current_angle = 0  # example current angle
target_angle = 5  # example target angle
movement = obstacle_avoidance_with_target_orientation(lidar_readings, current_angle, target_angle)
movement
