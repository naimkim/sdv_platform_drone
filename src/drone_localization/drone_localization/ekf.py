"""Constant-velocity Kalman filter for GPS / VIO position fusion.

State is [px, py, pz, vx, vy, vz] in a local ENU frame. The prediction uses a
constant-velocity model with a discrete white-noise-acceleration process model;
measurements are position fixes from VIO and (when trusted) GPS.

This is the linear core that ROS's robot_localization EKF generalizes. Keeping
it standalone and numpy-only makes the fusion math unit-testable without a
flight stack — the point of the Phase 2 "VIO / GPS-denied" deliverable.
"""

import numpy as np

STATE_DIM = 6
POS_SLICE = slice(0, 3)
VEL_SLICE = slice(3, 6)


class KalmanFilter:
    def __init__(self, process_noise=0.5,
                 initial_position=(0.0, 0.0, 0.0),
                 initial_velocity=(0.0, 0.0, 0.0),
                 initial_covariance=1.0):
        self.q = float(process_noise)
        self.x = np.zeros(STATE_DIM)
        self.x[POS_SLICE] = np.asarray(initial_position, dtype=float)
        self.x[VEL_SLICE] = np.asarray(initial_velocity, dtype=float)
        self.P = np.eye(STATE_DIM) * float(initial_covariance)

        # Position-observation model (GPS and VIO both observe position).
        self.H = np.zeros((3, STATE_DIM))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

    # -- prediction -----------------------------------------------------

    def predict(self, dt):
        if dt <= 0.0:
            return
        F = np.eye(STATE_DIM)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._process_noise(dt)

    def _process_noise(self, dt):
        Q = np.zeros((STATE_DIM, STATE_DIM))
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        for i in range(3):
            p, v = i, i + 3
            Q[p, p] = dt4 / 4.0 * self.q
            Q[p, v] = dt3 / 2.0 * self.q
            Q[v, p] = dt3 / 2.0 * self.q
            Q[v, v] = dt2 * self.q
        return Q

    # -- correction -----------------------------------------------------

    def innovation(self, position):
        """Measurement residual z - Hx for a position fix (no state change)."""
        z = np.asarray(position, dtype=float)
        return z - self.H @ self.x

    def update_position(self, position, measurement_noise):
        """Fuse a position fix; return the innovation vector used."""
        z = np.asarray(position, dtype=float)
        R = self._as_cov(measurement_noise)

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(STATE_DIM) - K @ self.H) @ self.P
        return y

    @staticmethod
    def _as_cov(measurement_noise):
        m = np.asarray(measurement_noise, dtype=float)
        if m.ndim == 0:
            return np.eye(3) * float(m)
        if m.ndim == 1:
            return np.diag(m)
        return m

    # -- accessors ------------------------------------------------------

    @property
    def position(self):
        return self.x[POS_SLICE].copy()

    @property
    def velocity(self):
        return self.x[VEL_SLICE].copy()

    @property
    def position_covariance(self):
        return self.P[POS_SLICE, POS_SLICE].copy()


def innovation_distance(innovation):
    """Euclidean magnitude of an innovation vector."""
    return float(np.linalg.norm(innovation))
