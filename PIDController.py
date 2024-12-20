import threading
import time

class PIDController(threading.Thread):
    def __init__(self, car, sensor, steering_controller, kp=0.6, ki=0.01, kd=0.2, min_error=0):
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
        self.steering_controller = steering_controller

    def run(self):
        while self.running:
            if self.car.lane_assist_on and self.sensor.distances:
                left_distance, right_distance = self.sensor.distances[0]
                error = (left_distance - right_distance) / 2

                if abs(error) > self.min_error:
                    self.error_sum += error
                    d_error = error - self.last_error

                    control = self.kp * error + self.ki * self.error_sum + self.kd * d_error

                    target_angle = self.car.steering_angle - control
                    self.steering_controller.set_target_angle(target_angle, from_controller=True)

                    self.last_error = error
            else:
                self.reset_controller()
            time.sleep(0.02)
    
    def reset_controller(self):
        self.error_sum = 0
        self.last_error = 0

    def stop(self):
        self.running = False
        self.steering_controller.reset_controller()