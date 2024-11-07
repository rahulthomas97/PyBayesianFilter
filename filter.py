import numpy as np


class Filter:
    def __init__(self, gridmap, lidar, motion_kernel, sensor_noise):
        """
        Initializes the filter.

        :param gridmap: 2D numpy array representing the grid map (obstacles = 1, free space = 0).
        :param lidar: Lidar sensor object for measurements.
        :param movement_noise: Array of movement noise for the four directions (e.g., [0.6, 0.3, 0.1]).
        :param sensor_noise: The sensor noise (standard deviation for measurement uncertainty).
        """
        self.gridmap = gridmap
        self.lidar = lidar
        self.motion_kernel = motion_kernel  # Movement noise array: [P(move 1 step), P(move 2 steps), P(move 3 steps)]
        self.sensor_noise = sensor_noise  # Sensor noise, assumed to be Gaussian for simplicity.

        # Grid dimensions
        self.nrows, self.ncols = self.gridmap.nrows,self.gridmap.ncols

        # Probability grid initialized uniformly
        self.belief = np.ones((self.nrows, self.ncols))
        self.belief[self.gridmap.obstacle_map==1]=0
        self.belief*=1/((self.nrows * self.ncols)-np.sum(self.gridmap.obstacle_map))

    def predict(self, direction):
        new_belief = np.zeros((self.nrows, self.ncols))

        for y in range(self.nrows):
            for x in range(self.ncols):
                prob = self.belief[y, x]

                for steps, step_prob in enumerate(self.motion_kernel, start=1):
                    if direction == "N":
                        new_y, new_x = max(y - steps, 0), x
                    elif direction == "S":
                        new_y, new_x = min(y + steps, self.nrows - 1), x
                    elif direction == "W":
                        new_y, new_x = y, max(x - steps, 0)
                    elif direction == "E":
                        new_y, new_x = y, min(x + steps, self.nrows - 1)

                    new_belief[new_y, new_x] += prob * step_prob

        # Normalize the belief
        self.belief = new_belief / new_belief.sum()

    def update(self, measurements):
        """
        Updates the belief about the robot's position based on noisy lidar measurements.

        :param measurements: A list of lidar measurements for the four directions (north, east, south, west).
        """
        # Loop over each possible cell in the grid and compute the likelihood of the measurements
        for i in range(self.nrows):
            for j in range(self.ncols):
                # Compute the expected measurement for each direction
                expected_measurements = self.lidar.get_expected_measurements(i, j, self.gridmap)

                # Calculate likelihood for the four directions based on observed measurements
                likelihood = 1.0
                for direction in measurements:
                    # Compute Gaussian likelihood (assuming Gaussian noise)
                    likelihood *= self.gaussian_likelihood(measurements[direction], expected_measurements[direction], self.sensor_noise)

                # Update probability using likelihood and prior
                self.belief[i, j] *= likelihood

        # Normalize the probabilities
        self.belief /= np.sum(self.belief)

    @staticmethod
    def gaussian_likelihood(observed, expected, sigma):
        """
        Computes the Gaussian likelihood for a noisy sensor measurement.

        :param observed: The actual measurement from the sensor.
        :param expected: The expected measurement (based on the robot's current position).
        :param sigma: The standard deviation (sensor noise).
        :return: The likelihood of the observed measurement given the expected value.
        """
        return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((observed - expected) ** 2) / (sigma ** 2))

    def get_belief(self):
        """
        Returns the current belief (probability distribution) of the robot's position.
        """
        return self.belief

    def step(self,direction,measurements):
        self.predict(direction)
        self.update(measurements)


