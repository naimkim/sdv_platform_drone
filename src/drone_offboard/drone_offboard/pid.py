"""A small, dependency-free PID controller with anti-windup and output limits.

Used by the offboard waypoint follower to turn position error into a velocity
setpoint. Keeping it standalone makes the gains unit-testable without a flight
stack, which is the point of the "control tuning" deliverable in Phase 1.
"""


class PID:
    def __init__(self, kp, ki, kd,
                 output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Symmetric saturation on the output and the integral term.
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def update(self, error, dt):
        """Return the control output for the given error over timestep dt."""
        if dt <= 0.0:
            return self._clamp_output(self.kp * error)

        # Integral with anti-windup clamp.
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = _clamp(
                self._integral, -self.integral_limit, self.integral_limit)

        # Derivative on the error (skip the first sample to avoid a kick).
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)
        return self._clamp_output(output)

    def _clamp_output(self, output):
        if self.output_limit is None:
            return output
        return _clamp(output, -self.output_limit, self.output_limit)


def _clamp(value, low, high):
    return max(low, min(high, value))
