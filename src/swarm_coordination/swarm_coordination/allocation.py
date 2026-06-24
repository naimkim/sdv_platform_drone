"""Search-area allocation across the active swarm members.

Given the set of members in good standing (the trusted set from the consensus
guard), each drone deterministically derives its own sub-region with no extra
coordination round — every honest agent runs the same function on the same set
and arrives at a consistent, disjoint partition. When a member drops or is
quarantined, the set shrinks and the area is re-partitioned automatically.

Pure functions so the partitioning is unit-testable without ROS.
"""

import math


def _ordered_members(drone_id, members):
    full = set(members)
    full.add(drone_id)
    return sorted(full)


def vertical_strip_bounds(drone_id, members, area_size):
    """Return this drone's (x_min, y_min, x_max, y_max) vertical strip.

    The square area is split into one full-height strip per member, ordered by
    id. Simple, deterministic and matches the RViz sector overlay.
    """
    ordered = _ordered_members(drone_id, members)
    n = max(1, len(ordered))
    idx = ordered.index(drone_id)
    strip = area_size / n
    return (strip * idx, 0.0, strip * (idx + 1), area_size)


def sector_center(drone_id, members, area_size):
    x_min, y_min, x_max, y_max = vertical_strip_bounds(
        drone_id, members, area_size)
    return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)


def grid_dimensions(n):
    """Near-square (rows, cols) able to hold n cells, cols >= rows."""
    n = max(1, n)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    return rows, cols


def assign_grid(members, area_size):
    """Assign each member a cell of a near-square grid over the area.

    Returns {drone_id: (x_min, y_min, x_max, y_max)}. A generalization of the
    vertical-strip split for denser swarms.
    """
    ordered = sorted(set(members))
    rows, cols = grid_dimensions(len(ordered))
    cell_w = area_size / cols
    cell_h = area_size / rows

    assignment = {}
    for idx, drone_id in enumerate(ordered):
        r = idx // cols
        c = idx % cols
        assignment[drone_id] = (
            c * cell_w, r * cell_h, (c + 1) * cell_w, (r + 1) * cell_h)
    return assignment
