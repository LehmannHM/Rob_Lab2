import threading
import time
import numpy as np

class LTASystem(threading.Thread):
    def __init__(self, car, sensor, steering_controller, MAX_STEERING_ANGLE=20):
        super().__init__()
        self.car = car
        self.sensor = sensor
        self.steering_controller = steering_controller

        self.running = True

        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.1

        self.MIN_ERROR = 3
        self.MAX_STEERING_ANGLE = 20
        self.LOOKAHEAD_DISTANCE = 5

        self.inactive = False
        self.error_sum = 0
        self.last_error = 0
        self.last_lane_width = 0

    def run(self):
        while self.running:
            if self.car.lane_assist_on and self.sensor.distances:

                error = self.calculate_error()

                # Reset if no lane detected
                if error is None:
                    self.inactive = True
                    self.steering_controller.reset_controller()
                    time.sleep(0.02)
                    continue
                else:
                    self.inactive = False

                if abs(error) > self.MIN_ERROR:
                    self.error_sum += error
                    d_error = error - self.last_error

                    control = self.Kp * error + self.Ki * self.error_sum + self.Kd * d_error

                    target_angle = self.car.steering_angle - control
                    target_angle = max(-self.MAX_STEERING_ANGLE, min(target_angle, self.MAX_STEERING_ANGLE))
                    self.steering_controller.set_target_angle(target_angle, from_controller=True)

                    self.last_error = error
                else:
                    # Reset controller if the error is too small
                    self.steering_controller.set_target_angle(0, from_controller=True)
            else:
                self.reset_controller()
            time.sleep(0.02)

    def calculate_error(self):
        if self.sensor.right_detected and self.sensor.left_detected:
            left_distance, right_distance = self.get_lookahead_distance()
            self.last_lane_width = left_distance + right_distance
        elif self.sensor.right_detected:
            _, right_distance = self.get_lookahead_distance()
            left_distance = self.last_lane_width - right_distance
        elif self.sensor.left_detected:
            left_distance, _ = self.get_lookahead_distance ()
            right_distance = self.last_lane_width - left_distance
        else:
            return None

        return (left_distance - right_distance) / 2

    def get_lookahead_distance(self):
        n = min(len(self.sensor.distances), self.LOOKAHEAD_DISTANCE)
        avg_left_distance  = sum(d[0] for d in self.sensor.distances[:n]) / n if self.sensor.left_detected else None
        avg_right_distance = sum(d[1] for d in self.sensor.distances[:n]) / n if self.sensor.right_detected else None
        return avg_left_distance, avg_right_distance

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