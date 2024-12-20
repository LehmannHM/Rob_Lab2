import threading
import time

class PIDController(threading.Thread):
    def __init__(self, car, sensor, kp=0.7, ki=0.01, kd=0.2, min_error=0):
        super().__init__()
        self.car = car
        self.sensor = sensor
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_error = min_error
        self.running = True
        self.error_sum = 0
        self.last_error = 0

    def run(self):
        while self.running:
            if self.sensor.distances:
                left_distance, right_distance = self.sensor.distances[0]
                error = (left_distance - right_distance) / 2

                if (abs(error) > self.min_error):
                    self.error_sum += error
                    d_error = error - self.last_error

                    control = self.kp * error + self.ki * self.error_sum + self.kd * d_error

                    self.car.steering_angle -= control
                    self.car.steering_angle = max(-self.car.max_steering, min(self.car.steering_angle, self.car.max_steering))

                    self.last_error = error

            time.sleep(0.02)

    def stop(self):
        self.running = False