import os
import math
import pygame
import threading
import time

from SteeringController import SteeringController
from pygame.math import Vector2

        
class Car:
    def __init__(self, screen_pos, max_steering=30, max_acceleration=50.0):
        self.position = Vector2(0, 0)  # Initial position
        self.screen_pos = screen_pos

        self.velocity = 100.0  # Initial forward velocity
        self.max_velocity = 200.0  # Maximum velocity
        self.angle = 0.0  # Heading angle in degrees
        self.max_steering = max_steering  # Maximum steering angle in degrees
        self.max_acceleration = max_acceleration
        self.brake_deceleration = 100.0
        self.free_deceleration = 20.0
        
        self.acceleration = 0.0  # Acceleration input
        self.steering_angle = 0.0  # Current steering angle

        self.lane_assist_on = True

        # Load image
        current_dir = os.path.dirname(os.path.abspath(__file__))        
        image_path = os.path.join(current_dir, "car.png")        
        self.image = pygame.image.load(image_path)
        self.length = self.image.get_height()

        # Bicycle model parameters
        self.l_f = self.length / 3  # Length from center to front axle
        self.l_r = self.length / 3 * 2  # Length from center to rear axle

    def init_position(self, x, y):
        self.position = Vector2(x, y)

    def update(self, dt):
        # Update velocity based on acceleration (affects forward speed)
        if self.acceleration != 0:
            self.velocity += self.acceleration * dt
            self.velocity = max(0, min(self.velocity, self.max_velocity))  # Prevent negative speed

        # Calculate beta (side slip angle)
        if abs(self.steering_angle) > 0:
            beta = math.atan(self.l_r / (self.l_f + self.l_r) * math.tan(math.radians(self.steering_angle)))
        else:
            beta = 0

        # Update position based on kinematic bicycle model equations
        dx = self.velocity * dt * math.cos(math.radians(self.angle + math.degrees(beta)))
        dy = self.velocity * dt * math.sin(math.radians(self.angle + math.degrees(beta)))

        # Update position; keep y constant since we are simulating a road
        self.position.y += dx
        self.position.x += dy
        
        # Update heading angle (psi)
        dpsi = (self.velocity / self.l_r) * math.sin(beta) * dt
        self.angle += math.degrees(dpsi)

class Lane:
    def __init__(self, game_width, game_height):
        self.width = 250  # Width of the lane
        self.curve_amplitude = 50  # Amplitude of the curve
        self.curve_frequency = 0.005  # Frequency of the curve
        self.game_width = game_width
        self.game_height = game_height
        self.current_amplitude = self.curve_amplitude
        self.current_frequency = self.curve_frequency
        self.target_amplitude = self.curve_amplitude
        self.target_frequency = self.curve_frequency
        self.transition_speed = 0.01 

    def calculate_road_x(self, y):
        return self.game_width // 2 + self.current_amplitude * math.sin(self.current_frequency * y)

    def get_lane_markings(self, y):
        road_center_x = self.calculate_road_x(y)
        left_x = road_center_x - self.width // 2
        right_x = road_center_x + self.width // 2
        return left_x, road_center_x, right_x

    def update_curve_parameters(self, scroll_offset):
        # Change amplitude and frequency dynamically based on scroll offset
        if scroll_offset // 1 % 100 == 0:  # Change every 1000 pixels
            self.target_amplitude = 50 + 50 * math.sin(scroll_offset * 0.001)
            self.target_frequency = 0.004 + 0.002 * math.cos(scroll_offset * 0.001)

        self.current_amplitude += (self.target_amplitude - self.current_amplitude) * self.transition_speed
        self.current_frequency += (self.target_frequency - self.current_frequency) * self.transition_speed

    def draw(self, screen, scroll_offset):
        # self.update_curve_parameters(scroll_offset)
        for y in range(0, self.game_height + int(scroll_offset), 5):  # Draw road in segments with scroll offset
            road_center_x = self.calculate_road_x(y + scroll_offset)
            left_x = road_center_x - self.width // 2
            right_x = road_center_x + self.width // 2

            # Draw the grey road area
            pygame.draw.polygon(screen, (200, 200, 200), [
                (left_x, y),
                (right_x, y),
                (right_x, y + 20),
                (left_x, y + 20)
            ])

            # Draw the black road boundaries
            pygame.draw.line(screen, (0, 0, 0), (left_x, y), (left_x, y + 20), 5)
            pygame.draw.line(screen, (0, 0, 0), (right_x, y), (right_x, y + 20), 5)

            # Draw the yellow dashed center line
            if (((y - scroll_offset) // 40) % 2 == 0):  
                pygame.draw.line(screen, (255, 255, 0), (road_center_x, y), (road_center_x, y +  20), 5)
            
            if (y % (20 * 5) == 0):
                # TODO: This value is not correct and should be moving with the road bit
                road_offset = pygame.font.SysFont('Arial', 24).render(f'{y + scroll_offset:.0f}', True, (0, 0, 0))
                screen.blit(road_offset, (left_x - 80, y))


class Simulator:
    def __init__(self, width, height, car, lane, sensor):
        pygame.init()
        pygame.display.set_caption("Car simulator")
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.running = True
        self.car_screen_pos = self.height * (3 / 4)
        self.car = car
        self.lane = lane
        self.sensor = sensor

        self.keep_velocity = True
        self.show_sensor_dots = False

        self.steering_controller = SteeringController(self.car)

        self.left_distances = []
        self.right_distances = []
        self.times = []
        self.start_time = time.time()

    def run(self):
        self.car.init_position(self.width // 2 + self.car.image.get_width(), 0)

        while self.running:
            dt = self.clock.get_time() / 1000

            # Handle events and user input
            self.handle_events()
            self.handle_user_input(dt)

            # Update logic for the car's movement and position.
            self.car.update(dt)
            self.steering_controller.update(dt)

            self.draw()

            self.clock.tick(self.ticks)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_k:
                    self.keep_velocity = not self.keep_velocity
                if event.key == pygame.K_s:
                    self.show_sensor_dots = not self.show_sensor_dots
                if event.key == pygame.K_a:
                    self.car.lane_assist_on = not self.car.lane_assist_on
    
    def handle_user_input(self, dt):
        pressed = pygame.key.get_pressed()

        # Acceleration and braking
        if pressed[pygame.K_UP]:
            if self.car.velocity < 0:
                self.car.acceleration = self.car.brake_deceleration
            else:
                self.car.acceleration += dt * self.car.max_acceleration 
        elif pressed[pygame.K_DOWN]:
            if self.car.velocity > 0:
                self.car.acceleration = -self.car.brake_deceleration 
            else:
                self.car.acceleration -= dt * self.car.max_acceleration 
        elif pressed[pygame.K_SPACE]:
            if abs(self.car.velocity) > dt * self.car.brake_deceleration:
                self.car.acceleration = -math.copysign(self.car.brake_deceleration, self.car.velocity)
            else:
                self.car.acceleration = -self.car.velocity / dt 
        else:
            if self.keep_velocity:
                self.car.acceleration = 0
            else:
                if abs(self.car.velocity) > dt * self.car.free_deceleration:
                    self.car.acceleration = -math.copysign(self.car.free_deceleration, self.car.velocity)
                else:
                    if dt != 0:
                        self.car.acceleration = -self.car.velocity / dt
        
        # Steering
        if pressed[pygame.K_RIGHT]:
            target_angle = self.car.steering_angle + dt * self.car.max_steering * 2
            self.steering_controller.set_target_angle(target_angle)            
            self.steering_controller.reset_controller()
        elif pressed[pygame.K_LEFT]:
            target_angle = self.car.steering_angle - dt * self.car.max_steering * 2
            self.steering_controller.set_target_angle(target_angle)
            self.steering_controller.reset_controller()
        elif not self.steering_controller.controller_active:
            self.steering_controller.set_target_angle(0)  # Gradually return to center

    def draw(self):
        self.screen.fill((255,255,255))   # Clear screen with white background

        # Lanes
        self.lane.draw(self.screen, self.car.position.y)

        # Car
        rotated_car_image = pygame.transform.rotate(self.car.image, -self.car.angle)
        rect_car_image = rotated_car_image.get_rect(center=(self.car.position.x , self.car_screen_pos))
        self.screen.blit(rotated_car_image , rect_car_image.topleft) 

        self.draw_text_overlay()

        if self.show_sensor_dots:
            self.draw_sensor_dots()

        pygame.display.flip()              # Refresh display

    def draw_text_overlay(self):
        # Display velocity and steering angle
        velocity = pygame.font.SysFont('Arial', 24).render(f'Velocity: {self.car.velocity:.0f}', True, (0, 0, 0))
        self.screen.blit(velocity, (self.width / 4 * 3, self.height / 4 * 3))

        steering_angle = pygame.font.SysFont('Arial', 24).render(f'Steering angle: {self.car.steering_angle:.0f}', True, (0, 0, 0))
        self.screen.blit(steering_angle, (self.width / 4 * 3, self.height / 4 * 3 + 20))

        # Display distance to lane markings
        left_distance = self.sensor.distances[0][0]
        right_distance = self.sensor.distances[0][1]
        left_distance_text = pygame.font.SysFont('Arial', 24).render(f'Left Distance: {left_distance:.0f}', True, (0, 0, 0))
        self.screen.blit(left_distance_text, (self.width / 9, self.height / 4 * 3))
        right_distance_text = pygame.font.SysFont('Arial', 24).render(f'Right Distance: {right_distance:.0f}', True, (0, 0, 0))
        self.screen.blit(right_distance_text, (self.width / 9, self.height / 4 * 3 + 20))

    def draw_sensor_dots(self):
        car_top = self.car_screen_pos - self.car.length / 2
        for i, (left_distance, right_distance) in enumerate(self.sensor.distances):
            pygame.draw.circle(self.screen, (255, 0, 0), (self.car.position.x - left_distance, car_top - i * 10), 5, 5)
            pygame.draw.circle(self.screen, (255, 0, 0), (self.car.position.x + right_distance, car_top - i * 10), 5, 5)

    def plot_lane_detection(self):
        plt.figure()
        plt.plot(self.times, self.left_distances, label='Left Distance')
        plt.plot(self.times, self.right_distances, label='Right Distance')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance')
        plt.title('Lane Detection Over Time')
        plt.legend()
        plt.show()