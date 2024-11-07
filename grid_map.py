# gridmap.py

import numpy as np


class GridMap:
    def __init__(self, file_name):

        self.obstacle_map = np.loadtxt(file_name)
        self.nrows = self.obstacle_map.shape[0]
        self.ncols = self.obstacle_map.shape[1]

    # Function to calculate distance from a cell to obstacles/edge of map in a given direction
    def get_directional_measurement(self, x, y, direction):
        dist = 0
        while True:
            if direction == "N":
                y -= 1
            elif direction == "S":
                y += 1
            elif direction == "W":
                x -= 1
            elif direction == "E":
                x += 1

            # If out of bounds or hits obstacle, return the distance
            if x < 0 or x >= self.ncols or y < 0 or y >= self.nrows or self.obstacle_map[y, x] == 1:
                return dist
            dist += 1

    # Function to return distances from cell to obstacles/edge of map in all 4 directions
    def get_all_measurements(self, x, y):
        measurements = {direction: self.get_directional_measurement(x, y, direction) for direction in
                        ["N", "S", "W", "E"]}
        return measurements

    def display(self, belief):
        for y in range(self.nrows):
            row = ""
            for x in range(self.ncols):
                if self.obstacle_map[y, x] == 1:
                    row += " X "
                else:
                    row += f"{belief[y, x]:.2f} "
            print(row)
        print("=" * 40)
