"""Unit tests for the potential-field avoidance core."""

import numpy as np

from drone_avoidance.potential_field import (
    attractive_velocity,
    avoidance_velocity,
    clamp_magnitude,
    repulsive_velocity,
)


def test_no_obstacle_heads_to_goal():
    v = avoidance_velocity((10.0, 0.0), [], max_speed=1.5)
    assert v[0] > 1.0           # moving toward +x goal
    assert abs(v[1]) < 1e-6
    assert np.linalg.norm(v) <= 1.5 + 1e-9


def test_goal_reached_stops():
    v = attractive_velocity((0.0, 0.0), gain=1.0, max_speed=1.5,
                            slow_radius=1.5)
    assert np.allclose(v, [0.0, 0.0])


def test_speed_is_clamped():
    v = avoidance_velocity((100.0, 0.0), [], max_speed=1.5)
    assert np.linalg.norm(v) <= 1.5 + 1e-9


def test_obstacle_out_of_range_has_no_effect():
    v = repulsive_velocity((5.0, 0.0), influence_radius=2.5, gain=1.2,
                           min_clearance=0.3)
    assert np.allclose(v, [0.0, 0.0])


def test_close_obstacle_pushes_away():
    # Obstacle just ahead on +x -> repulsion points along -x.
    v = repulsive_velocity((0.5, 0.0), influence_radius=2.5, gain=1.2,
                           min_clearance=0.3)
    assert v[0] < 0.0


def test_obstacle_ahead_deflects_path():
    goal = (10.0, 0.0)
    # Obstacle between drone and goal -> goalward speed is reduced vs clear.
    clear = avoidance_velocity(goal, [], max_speed=1.5)
    blocked = avoidance_velocity(goal, [(0.6, 0.0)], max_speed=1.5)
    assert blocked[0] < clear[0]


def test_clamp_magnitude():
    assert np.allclose(clamp_magnitude([3.0, 4.0], 5.0), [3.0, 4.0])
    out = clamp_magnitude([3.0, 4.0], 2.5)
    assert abs(np.linalg.norm(out) - 2.5) < 1e-9
