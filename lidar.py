# lidar.py

import numpy as np


class LidarSensor:
    def __init__(self, grid_map, sensor_std_dev=1.0):
        self.grid_map = grid_map
        self.sensor_std_dev = sensor_std_dev

    def get_noisy_measurement(self, x, y):
        measurements=self.grid_map.get_all_measurements(x,y)
        for direction in measurements:
            measurements[direction]+=np.floor(np.random.normal(0,self.sensor_std_dev))
        return measurements
