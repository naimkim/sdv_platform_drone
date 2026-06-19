import math
import time


class SensorDriverBase:

    def read_obstacle(self):
        raise NotImplementedError

    def calibrate(self):
        raise NotImplementedError

    def set_mission_active(self, active):
        return None


class SimSensorDriver(SensorDriverBase):

    def __init__(self):
        self.start_time = time.monotonic()
        self.calibrated = True
        self.mission_active = False
        self.mission_started_at = None

    def read_obstacle(self):
        elapsed_sec = time.monotonic() - self.start_time
        mission_elapsed = 0.0
        if self.mission_started_at is not None:
            mission_elapsed = time.monotonic() - self.mission_started_at

        # During each mission, expose an obstacle after four seconds. This
        # makes Demo #1 deterministic regardless of launch timing.
        detected = self.mission_active and 4.0 <= mission_elapsed < 7.0

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

    def set_mission_active(self, active):
        active = bool(active)
        if active and not self.mission_active:
            self.mission_started_at = time.monotonic()
        elif not active:
            self.mission_started_at = None
        self.mission_active = active


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
