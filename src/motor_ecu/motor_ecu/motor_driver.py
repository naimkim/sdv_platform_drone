class MotorDriverBase:

    def set_velocity(self, linear, angular):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def emergency_stop(self):
        raise NotImplementedError

    def update(self, dt_sec):
        raise NotImplementedError

    def get_status(self):
        raise NotImplementedError


class SimMotorDriver(MotorDriverBase):

    def __init__(self):
        self.target_linear = 0.0
        self.current_linear = 0.0
        self.target_angular = 0.0
        self.current_angular = 0.0
        self.enabled = False
        self.max_linear_accel = 0.6
        self.max_angular_accel = 1.2

    def set_velocity(self, linear, angular):
        self.enabled = True
        self.target_linear = float(linear)
        self.target_angular = float(angular)

    def stop(self):
        self.enabled = False
        self.target_linear = 0.0
        self.target_angular = 0.0

    def emergency_stop(self):
        self.enabled = False
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

    def update(self, dt_sec):
        self.current_linear = self.step_toward(
            self.current_linear,
            self.target_linear,
            self.max_linear_accel * dt_sec
        )
        self.current_angular = self.step_toward(
            self.current_angular,
            self.target_angular,
            self.max_angular_accel * dt_sec
        )

    def get_status(self):
        return {
            'target_linear': self.target_linear,
            'current_linear': self.current_linear,
            'target_angular': self.target_angular,
            'current_angular': self.current_angular,
            'enabled': self.enabled,
        }

    def step_toward(self, current, target, max_delta):
        if current < target:
            return min(current + max_delta, target)
        if current > target:
            return max(current - max_delta, target)
        return current
