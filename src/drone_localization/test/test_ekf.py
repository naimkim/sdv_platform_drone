"""Unit tests for the constant-velocity Kalman filter."""

import numpy as np

from drone_localization.ekf import KalmanFilter, innovation_distance


def test_predict_integrates_velocity():
    kf = KalmanFilter(initial_position=(0.0, 0.0, 0.0),
                      initial_velocity=(1.0, 2.0, 0.0))
    kf.predict(2.0)
    pos = kf.position
    assert abs(pos[0] - 2.0) < 1e-9
    assert abs(pos[1] - 4.0) < 1e-9
    assert abs(pos[2] - 0.0) < 1e-9


def test_zero_dt_predict_is_noop():
    kf = KalmanFilter(initial_velocity=(1.0, 0.0, 0.0))
    kf.predict(0.0)
    assert np.allclose(kf.position, [0.0, 0.0, 0.0])


def test_update_pulls_toward_measurement():
    kf = KalmanFilter(initial_position=(0.0, 0.0, 0.0),
                      initial_covariance=1.0)
    kf.update_position((10.0, 0.0, 0.0), measurement_noise=0.01)
    # Tight measurement noise -> estimate jumps close to the fix.
    assert kf.position[0] > 9.0


def test_innovation_is_measurement_minus_estimate():
    kf = KalmanFilter(initial_position=(1.0, 2.0, 3.0))
    y = kf.innovation((4.0, 2.0, 3.0))
    assert abs(y[0] - 3.0) < 1e-9
    assert abs(y[1]) < 1e-9
    assert innovation_distance(y) - 3.0 < 1e-9


def test_repeated_updates_reduce_covariance():
    kf = KalmanFilter(initial_covariance=10.0)
    start = np.trace(kf.position_covariance)
    for _ in range(20):
        kf.predict(0.05)
        kf.update_position((5.0, 5.0, 2.0), measurement_noise=0.1)
    end = np.trace(kf.position_covariance)
    assert end < start
    # And it should track the measured point.
    assert np.linalg.norm(kf.position - np.array([5.0, 5.0, 2.0])) < 0.5


def test_diagonal_measurement_noise_accepted():
    kf = KalmanFilter()
    # Per-axis noise vector should be accepted as a diagonal R.
    y = kf.update_position((1.0, 1.0, 1.0), measurement_noise=[0.1, 0.1, 0.1])
    assert y.shape == (3,)
