import threading
import time
import random
import numpy as np

from random import random, gauss

class LaneSensor(threading.Thread):
    def __init__(self, car, lane, do_add_noise=True, do_add_disturbances=True):
        super().__init__()
        self.car = car
        self.lane = lane
        self.car_screen_pos = self.car.screen_pos
        self.do_add_noise = do_add_noise
        self.do_add_disturbances = do_add_disturbances

        self.num_measurement_points = 30
        self.distance_between_points = 5

        self.running = True
        self.distances = []

    def run(self):
        while self.running:
            self.distances = self.get_sensor_values()
            time.sleep(0.02)

    def get_distance_to_lane(self):
        _, road_center_x, right_x = self.lane.get_lane_markings(self.car.position.y + self.car_screen_pos - self.car.length / 2)
        left_distance = self.car.position.x - road_center_x
        right_distance = right_x - self.car.position.x
        return left_distance, right_distance

    def get_distances_to_lane(self):
        distances = []
        for i in range(self.num_measurement_points):
            y = self.car.position.y + self.car_screen_pos - self.car.length / 2 - i * self.distance_between_points
            _, road_center_x, right_x = self.lane.get_lane_markings(y)
            left_distance = self.car.position.x - road_center_x
            right_distance = right_x - self.car.position.x
            distances.append((left_distance, right_distance))

        return distances

    def add_noise(self, distances, noise=2):
        return [(left_distance + gauss(0, noise), right_distance + gauss(0, noise)) for left_distance, right_distance in distances]

    def get_sensor_values(self):
        distances = self.get_distances_to_lane()
        
        if self.do_add_noise:
            distances = self.add_noise(distances)

        # TODO: Remove some of the distances

        return distances

    def fit_lane_curve(self):
        y_values = np.arange(len(self.distances)) * self.distance_between_points
        left_distances = [d[0] for d in self.distances]
        right_distances = [d[1] for d in self.distances]

        # Fit a second-degree polynomial (quadratic) to the lane markings
        left_fit = np.polyfit(y_values, left_distances, 2)
        right_fit = np.polyfit(y_values, right_distances, 2)

        return left_fit, right_fit

    def get_curve_points(self, fit, y_values):
        x_values = np.polyval(fit, y_values)
        points = [(x, y) for x, y in zip(x_values, y_values)]
        return points

    def stop(self):
        self.running = False