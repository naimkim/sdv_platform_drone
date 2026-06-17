import math
import time


class SensorDriverBase:

    def read_obstacle(self):
        raise NotImplementedError

    def calibrate(self):
        raise NotImplementedError


class SimSensorDriver(SensorDriverBase):

    def __init__(self):
        self.start_time = time.monotonic()
        self.calibrated = True

    def read_obstacle(self):
        elapsed_sec = time.monotonic() - self.start_time

        # Simulate an obstacle every 12 seconds for 4 seconds.
        cycle_sec = elapsed_sec % 12.0
        detected = cycle_sec >= 8.0

        if detected:
            distance = 0.4 + 0.2 * math.sin(elapsed_sec * 2.0)
            angle = 10.0 * math.sin(elapsed_sec)
        else:
            distance = 3.0
            angle = 0.0

        return {
            'detected': detected,
            'distance': max(0.0, float(distance)),
            'angle': float(angle),
        }

    def calibrate(self):
        self.calibrated = True
        self.start_time = time.monotonic()
        return True


class HwSensorDriver(SensorDriverBase):

    def __init__(self):
        self.calibrated = False

    def read_obstacle(self):
        # Hardware integration point. Keep the vehicle safe until a concrete
        # sensor backend is connected.
        return {
            'detected': False,
            'distance': 0.0,
            'angle': 0.0,
        }

    def calibrate(self):
        self.calibrated = True
        return True
