# main.py

import numpy as np
from robot import Robot
from grid_map import GridMap
from lidar import LidarSensor
from filter import Filter

GRID_SIZE = 10
motion_kernel = [0.6, 0.3, 0.1]
sensor_noise=1
# Initialize grid, robot, lidar, and filter
grid_map = GridMap(GRID_SIZE)
robot = Robot(grid_map, motion_kernel)
lidar = LidarSensor(grid_map, sensor_noise)
bayes_filter = Filter(grid_map, lidar, motion_kernel, sensor_noise)

# Control loop
while True:
    print("Enter direction (N, S, W, E) or 'q' to quit:")
    user_input = input().strip().upper()
    if user_input == "Q":
        break

    # Move the robot and get noisy sensor readings
    robot.move(user_input)
    measurements = lidar.get_noisy_measurement(robot.x, robot.y)

    # Update belief using Bayesian filter
    belief = bayes_filter.step(user_input, measurements)

    print("Current belief:")
    grid_map.display(belief)
