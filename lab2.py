import time
from Simulation import Car, Lane, Simulator
from LaneSensor import LaneSensor
from LTASystem import LTASystem

WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 1000
CAR_SCREEN_POS = WINDOW_HEIGHT * (3 / 4)

if __name__ == '__main__':
    car = Car(CAR_SCREEN_POS)
    lane = Lane(WINDOW_WIDTH, WINDOW_HEIGHT)
    sensor = LaneSensor(car, lane, do_add_noise=True, do_add_disturbances=True)
    simulator = Simulator(WINDOW_WIDTH, WINDOW_HEIGHT, car, lane, sensor)
    lta_system = LTASystem(car, sensor, simulator.steering_controller)

    sensor.start()
    lta_system.start()

    try:
        simulator.run()
    except KeyboardInterrupt:
        sensor.stop()
        lta_system.stop()

        sensor.join()
        lta_system.join()

    sensor.stop()
    lta_system.stop()

    sensor.join()
    lta_system.join()