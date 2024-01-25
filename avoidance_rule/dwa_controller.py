import numpy as np

class Vehicle:
    # Vehicle parameters
    def __init__(self):
        self.max_speed = 1.0  # maximum speed [m/s]
        self.min_speed = -0.5  # minimum speed [m/s]
        self.max_yaw_rate = 40.0 * np.pi / 180.0  # maximum yaw rate [rad/s]
        self.max_accel = 0.2  # maximum acceleration [m/ss]
        self.max_delta_yaw_rate = 40.0 * np.pi / 180.0  # maximum delta yaw rate [rad/ss]
        self.v_resolution = 0.01  # velocity resolution [m/s]
        self.yaw_rate_resolution = 0.1 * np.pi / 180.0  # yaw rate resolution [rad/s]
        self.dt = 0.1  # time tick [s]
        self.predict_time = 3.0  # predict time [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 1.0  # robot radius [m]

def calculate_dynamic_window(vehicle, state):
    Vs = [vehicle.min_speed, vehicle.max_speed, -vehicle.max_yaw_rate, vehicle.max_yaw_rate]
    Vd = [state[3] - vehicle.max_accel * vehicle.dt, state[3] + vehicle.max_accel * vehicle.dt,
          state[4] - vehicle.max_delta_yaw_rate * vehicle.dt, state[4] + vehicle.max_delta_yaw_rate * vehicle.dt]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def calculate_trajectory(state, v, y, eval_dt, vehicle):
    x = np.array(state)
    trajectory = np.array(x)
    time = 0
    while time <= eval_dt:
        x[2] += y * vehicle.dt
        x[0] += v * np.cos(x[2]) * vehicle.dt
        x[1] += v * np.sin(x[2]) * vehicle.dt
        trajectory = np.vstack((trajectory, x))
        time += vehicle.dt
    return trajectory

def calculate_cost(vehicle, trajectory, goal, obstacles):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    goal_cost = vehicle.to_goal_cost_gain * np.hypot(dx, dy)
    speed_cost = vehicle.speed_cost_gain * (vehicle.max_speed - trajectory[-1, 3])
    obstacle_cost = 0.0
    for ob in obstacles:
        d = np.hypot(ob[0] - trajectory[:, 0], ob[1] - trajectory[:, 1])
        if np.amin(d) <= vehicle.robot_radius:
            obstacle_cost = float("inf")
            break
        else:
            obstacle_cost += vehicle.obstacle_cost_gain / np.amin(d)
    total_cost = goal_cost + speed_cost + obstacle_cost
    return total_cost

class DWAController:
    def __init__(self):
        self.vehicle = Vehicle()

    def calculate_action(self, car_pos, target_pos, lidar_data, last_action):
        obstacle_positions = self.lidar_to_obstacles(lidar_data, car_pos)
        target_direction = np.arctan2(target_pos[1] - car_pos[1], target_pos[0] - car_pos[0])
        state = [car_pos[0], car_pos[1], target_direction, self.vehicle.max_speed, 0]
        trajectory, control = self.dwa_control(state, target_pos, obstacle_positions)
        action = self.control_to_action(control)
        return action

    def lidar_to_obstacles(self, lidar_data, car_pos):
        obstacles = []
        for i, distance in enumerate(lidar_data):
            angle = np.deg2rad(i * (360 / len(lidar_data)))
            x = car_pos[0] + distance * np.cos(angle)
            y = car_pos[1] + distance * np.sin(angle)
            obstacles.append([x, y])
        return obstacles

    def control_to_action(self, control):
        v, y = control
        if y > 0:
            return 1  # Left turn
        elif y < 0:
            return 2  # Right turn
        else:
            return 0 if v > 0 else 3  # Forward or Reverse

    def dwa_control(self, state, goal,obstacles):
        dw = calculate_dynamic_window(self.vehicle, state)
        best_trajectory = np.array([state])
        best_cost = float("inf")
        best_v = state[3]
        best_y = state[4]
        for v in np.arange(dw[0], dw[1], self.vehicle.v_resolution):
            for y in np.arange(dw[2], dw[3], self.vehicle.yaw_rate_resolution):
                trajectory = calculate_trajectory(state, v, y, self.vehicle.predict_time, self.vehicle)
                cost = calculate_cost(self.vehicle, trajectory, goal, obstacles)
                if cost < best_cost:
                    best_cost = cost
                    best_trajectory = trajectory
                    best_v = v
                    best_y = y

        return best_trajectory, [best_v, best_y]
