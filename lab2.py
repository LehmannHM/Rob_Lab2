import time
from Simulation import Car, Lane, Simulator
from LaneSensor import LaneSensor
from PIDController import PIDController

WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 1000
CAR_SCREEN_POS = WINDOW_HEIGHT * (3 / 4)

if __name__ == '__main__':
    car = Car(CAR_SCREEN_POS)
    lane = Lane(WINDOW_WIDTH, WINDOW_HEIGHT)
    sensor = LaneSensor(car, lane, do_add_noise=True)
    simulator = Simulator(WINDOW_WIDTH, WINDOW_HEIGHT, car, lane, sensor)
    pid_controller = PIDController(car, sensor, simulator.steering_controller)

    sensor.start()
    pid_controller.start()

    try:
        simulator.run()
    except KeyboardInterrupt:
        sensor.stop()
        pid_controller.stop()

        sensor.join()
        pid_controller.join()

    sensor.stop()
    pid_controller.stop()

    sensor.join()
    pid_controller.join()