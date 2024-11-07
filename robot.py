# robot.py

import numpy as np


class Robot:
    def __init__(self, x, y, map, kernel):
        self.x = x
        self.y = y
        self.obstacle_map = map.obstacle_map
        self.map_nrows = map.nrows
        self.map_ncols = map.ncols
        self.kernel = kernel

    # This robot has access to the map information to do implicit obstacle checking
    def move(self, direction):
        steps = np.random.choice([1, 2, 3], p=self.kernel)  # Probabilities for 1, 2, or 3 steps
        for _ in range(steps):
            if direction == "N" and self.y > 0 and self.obstacle_map[self.y - 1] != 1:
                self.y -= 1
            elif direction == "S" and self.y < self.map_nrows - 1 and self.obstacle_map[self.y + 1] != 1:
                self.y += 1
            elif direction == "W" and self.x > 0 and self.obstacle_map[self.x - 1] != 1:
                self.x -= 1
            elif direction == "E" and self.x < self.map_ncols - 1 and self.obstacle_map[self.x + 1] != 1:
                self.x += 1

    def get_position(self):
        return self.x, self.y
