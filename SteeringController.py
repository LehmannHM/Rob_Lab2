class SteeringController:
    def __init__(self, car, max_steering_speed=120.0):
        self.car = car
        self.target_angle = 0.0
        self.max_steering_speed = max_steering_speed
        self.controller_active = False  # Flag to indicate if the PID controller is active

    def update(self, dt):
        angle_diff = self.target_angle - self.car.steering_angle
        max_angle_change = self.max_steering_speed * dt

        if abs(angle_diff) > max_angle_change:
            angle_diff = max_angle_change if angle_diff > 0 else -max_angle_change

        self.car.steering_angle += angle_diff

        # Gradually return to center if no controller is active
        if not self.controller_active and self.target_angle == 0:
            self.car.steering_angle *= 0.92

    def set_target_angle(self, angle, from_controller=False):
        self.target_angle = max(-self.car.max_steering, min(angle, self.car.max_steering))
        self.controller_active = from_controller

    def reset_controller(self):
        self.controller_active = False