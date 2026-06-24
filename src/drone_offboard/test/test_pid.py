"""Unit tests for the standalone PID controller."""

from drone_offboard.pid import PID


def test_proportional_only():
    pid = PID(2.0, 0.0, 0.0)
    assert pid.update(1.0, 0.1) == 2.0
    assert pid.update(-0.5, 0.1) == -1.0


def test_output_limit_clamps():
    pid = PID(10.0, 0.0, 0.0, output_limit=2.0)
    assert pid.update(5.0, 0.1) == 2.0
    assert pid.update(-5.0, 0.1) == -2.0


def test_integral_accumulates():
    pid = PID(0.0, 1.0, 0.0)
    # Constant error of 1.0 over two 0.5 s steps -> integral 1.0.
    pid.update(1.0, 0.5)
    out = pid.update(1.0, 0.5)
    assert abs(out - 1.0) < 1e-9


def test_integral_limit_prevents_windup():
    pid = PID(0.0, 1.0, 0.0, integral_limit=0.5)
    for _ in range(20):
        out = pid.update(1.0, 0.5)
    assert out <= 0.5 + 1e-9


def test_derivative_responds_to_change():
    pid = PID(0.0, 0.0, 1.0)
    # First sample seeds the derivative (no kick).
    assert pid.update(1.0, 0.1) == 0.0
    # Error grows by 1.0 over 0.1 s -> derivative 10.0.
    assert abs(pid.update(2.0, 0.1) - 10.0) < 1e-9


def test_reset_clears_state():
    pid = PID(0.0, 1.0, 0.0)
    pid.update(1.0, 1.0)
    pid.reset()
    # After reset the integral is gone, so a fresh step starts from zero.
    assert abs(pid.update(1.0, 1.0) - 1.0) < 1e-9


def test_zero_dt_is_safe():
    pid = PID(1.0, 1.0, 1.0, output_limit=5.0)
    # Should not divide by zero; falls back to proportional.
    assert pid.update(3.0, 0.0) == 3.0
