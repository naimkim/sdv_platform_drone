"""Artificial potential field for reactive obstacle avoidance (2D, horizontal).

The desired velocity is the sum of an attractive pull toward the goal and a
repulsive push away from each nearby obstacle. Obstacles are given as relative
position vectors (drone -> obstacle) in the same planar frame as the goal
vector. Altitude is handled separately by the controller.

Pure numpy so the avoidance behaviour is unit-testable without a flight stack.
"""

import numpy as np


def attractive_velocity(goal_vec, gain, max_speed, slow_radius):
    """Velocity pulling toward the goal, easing off inside slow_radius."""
    goal = np.asarray(goal_vec, dtype=float)[:2]
    dist = float(np.linalg.norm(goal))
    if dist < 1e-6:
        return np.zeros(2)
    speed = max_speed if dist > slow_radius else max_speed * (dist / slow_radius)
    return goal / dist * min(gain * dist, speed)


def repulsive_velocity(obstacle_vec, influence_radius, gain, min_clearance):
    """Velocity pushing away from a single obstacle (zero if out of range)."""
    obs = np.asarray(obstacle_vec, dtype=float)[:2]
    dist = float(np.linalg.norm(obs))
    if dist >= influence_radius or dist < 1e-9:
        if dist < 1e-9:
            # Sitting on the obstacle: push along +x as a safe default.
            return np.array([gain, 0.0])
        return np.zeros(2)

    clamped = max(dist, min_clearance)
    # Classic 1/d falloff, scaled so it vanishes at the influence boundary.
    magnitude = gain * (1.0 / clamped - 1.0 / influence_radius)
    away = -obs / dist
    return away * magnitude


def avoidance_velocity(goal_vec, obstacles,
                       max_speed=1.5,
                       attract_gain=1.0,
                       slow_radius=1.5,
                       influence_radius=2.5,
                       repulse_gain=1.2,
                       min_clearance=0.3):
    """Blend attraction and repulsion into a clamped velocity command."""
    velocity = attractive_velocity(
        goal_vec, attract_gain, max_speed, slow_radius)

    for obs in obstacles:
        velocity = velocity + repulsive_velocity(
            obs, influence_radius, repulse_gain, min_clearance)

    return clamp_magnitude(velocity, max_speed)


def clamp_magnitude(vec, max_magnitude):
    vec = np.asarray(vec, dtype=float)
    mag = float(np.linalg.norm(vec))
    if mag > max_magnitude and mag > 1e-9:
        return vec / mag * max_magnitude
    return vec
