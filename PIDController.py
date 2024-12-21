import threading
import time
import numpy as np

class PIDController(threading.Thread):
    def __init__(self, car, sensor, steering_controller, kp=0.2, ki=0.0, kd=0.1, min_error=3, max_steering_angle=20):
        super().__init__()
        self.car = car
        self.sensor = sensor
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_error = min_error
        self.max_steering_angle = max_steering_angle
        self.running = True
        self.inactive = False
        self.error_sum = 0
        self.last_error = 0
        self.steering_controller = steering_controller
        self.last_lane_width = 0

    def run(self):
        while self.running:
            if self.car.lane_assist_on and self.sensor.distances:

                error = self.calculate_error()

                if error is None:
                    self.inactive = True
                    self.steering_controller.reset_controller()
                    time.sleep(0.02)
                    continue
                else:
                    self.inactive = False

                if abs(error) > self.min_error:
                    self.error_sum += error
                    d_error = error - self.last_error

                    control = self.kp * error + self.ki * self.error_sum + self.kd * d_error

                    target_angle = self.car.steering_angle - control
                    target_angle = max(-self.max_steering_angle, min(target_angle, self.max_steering_angle))
                    self.steering_controller.set_target_angle(target_angle, from_controller=True)

                    self.last_error = error
                else:
                    self.steering_controller.set_target_angle(0, from_controller=True)
            else:
                self.reset_controller()
            time.sleep(0.02)

    def calculate_error(self):
        if self.sensor.right_detected and self.sensor.left_detected:
            left_distance, right_distance = self.sensor.distances[0]
            self.last_lane_width = left_distance + right_distance
        elif self.sensor.right_detected:
            _, right_distance = self.sensor.distances[0]
            left_distance = self.last_lane_width - right_distance
        elif self.sensor.left_detected:
            left_distance, _ = self.sensor.distances[0]
            right_distance = self.last_lane_width - left_distance
        else:
            return None

        return (left_distance - right_distance) / 2

    def calculate_steering_angle(self):
        # Calculate the center of the lane at the car's position
        y = self.car.screen_pos - self.car.length / 2
        left_fit, right_fit = self.sensor.fit_lane_curve()
        left_x = np.polyval(left_fit, y)
        right_x = np.polyval(right_fit, y)
        lane_center_x = (left_x + right_x) / 2

        # Calculate the error (distance from the car to the lane center)
        error = self.car.position.x - lane_center_x

        return error
    
    def reset_controller(self):
        self.error_sum = 0
        self.last_error = 0

    def stop(self):
        self.running = False
        self.steering_controller.reset_controller()