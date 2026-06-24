"""Unit tests for the detection -> obstacle geometry bridge."""

import math

from drone_perception.detection_geometry import (
    bearing_rad,
    estimate_distance,
    focal_length_px,
    obstacle_offset,
)


def test_focal_length_for_90deg_fov():
    # 90 deg HFOV, 640 px wide -> f = 320 / tan(45) = 320.
    f = focal_length_px(640, math.radians(90.0))
    assert abs(f - 320.0) < 1e-6


def test_centered_detection_has_zero_bearing():
    f = focal_length_px(640, math.radians(90.0))
    assert abs(bearing_rad(320.0, 640, f)) < 1e-9


def test_right_edge_bearing_is_positive_45deg():
    f = focal_length_px(640, math.radians(90.0))
    assert abs(bearing_rad(640.0, 640, f) - math.radians(45.0)) < 1e-6


def test_estimate_distance_pinhole():
    # 1 m tall object, 100 px high, f = 320 -> 3.2 m.
    assert abs(estimate_distance(1.0, 100.0, 320.0) - 3.2) < 1e-9


def test_estimate_distance_zero_size_is_inf():
    assert estimate_distance(1.0, 0.0, 320.0) == float('inf')


def test_centered_obstacle_is_straight_ahead():
    x, y = obstacle_offset(320.0, 640, math.radians(90.0), distance=5.0)
    assert abs(x - 5.0) < 1e-9
    assert abs(y) < 1e-9


def test_right_detection_maps_to_negative_y():
    # Object on the right of the image -> to the right of the drone (-y).
    x, y = obstacle_offset(640.0, 640, math.radians(90.0), distance=5.0)
    assert y < 0.0
    assert x > 0.0
    # Range is preserved: sqrt(x^2 + y^2) == distance.
    assert abs(math.hypot(x, y) - 5.0) < 1e-6
