"""Inter-drone collision avoidance (planar, position-based).

Each drone adds a repulsive velocity away from neighbours that come within a
separation radius, then re-clamps to its speed limit. Because every drone runs
the same rule symmetrically, the avoidance is effectively reciprocal — each
pair backs off together rather than one chasing the other.

Pure functions, unit-testable without ROS.
"""

import math


def separation_velocity(own_pos, neighbors, radius, gain):
    """Repulsive velocity pushing away from neighbours inside `radius`."""
    ox, oy = own_pos
    ax = ay = 0.0
    for nx, ny in neighbors:
        dx, dy = ox - nx, oy - ny
        dist = math.hypot(dx, dy)
        if 0.0 < dist < radius:
            weight = (radius - dist) / radius
            ax += (dx / dist) * weight * gain
            ay += (dy / dist) * weight * gain
    return ax, ay


def avoid(desired_velocity, own_pos, neighbors, radius, gain, max_speed):
    """Blend a desired velocity with neighbour repulsion, clamped to max_speed."""
    rx, ry = separation_velocity(own_pos, neighbors, radius, gain)
    vx = desired_velocity[0] + rx
    vy = desired_velocity[1] + ry
    speed = math.hypot(vx, vy)
    if speed > max_speed and speed > 1e-9:
        scale = max_speed / speed
        vx, vy = vx * scale, vy * scale
    return vx, vy
