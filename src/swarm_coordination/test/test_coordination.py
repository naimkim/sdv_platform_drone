"""Unit tests for the swarm coordination algorithms."""

import math

from swarm_coordination import allocation, collision, coverage


# -- allocation ---------------------------------------------------------

def test_strips_partition_the_area():
    members = ['drone_1', 'drone_2', 'drone_3']
    spans = []
    for d in members:
        x_min, _, x_max, _ = allocation.vertical_strip_bounds(d, members, 9.0)
        spans.append((x_min, x_max))
    spans.sort()
    # Disjoint, contiguous, covering [0, 9].
    assert abs(spans[0][0] - 0.0) < 1e-9
    assert abs(spans[-1][1] - 9.0) < 1e-9
    for (a_lo, a_hi), (b_lo, b_hi) in zip(spans, spans[1:]):
        assert abs(a_hi - b_lo) < 1e-9


def test_self_included_even_if_not_in_members():
    # An agent not yet in the trusted list still gets a valid strip.
    c = allocation.sector_center('drone_X', ['drone_A'], 10.0)
    assert 0.0 <= c[0] <= 10.0


def test_strip_shrinks_as_members_grow():
    one = allocation.vertical_strip_bounds('d1', ['d1'], 10.0)
    many = allocation.vertical_strip_bounds('d1', ['d1', 'd2', 'd3', 'd4'], 10.0)
    assert (one[2] - one[0]) > (many[2] - many[0])


def test_grid_dimensions_near_square():
    assert allocation.grid_dimensions(1) == (1, 1)
    assert allocation.grid_dimensions(4) == (2, 2)
    rows, cols = allocation.grid_dimensions(5)
    assert rows * cols >= 5 and cols >= rows


def test_assign_grid_covers_all_members():
    members = ['a', 'b', 'c', 'd']
    cells = allocation.assign_grid(members, 10.0)
    assert set(cells) == set(members)


# -- coverage -----------------------------------------------------------

def test_lawnmower_waypoints_within_bounds():
    bounds = (0.0, 0.0, 4.0, 4.0)
    wps = coverage.lawnmower(bounds, spacing=1.0, margin=0.5)
    assert len(wps) >= 2
    for x, y in wps:
        assert 0.0 <= x <= 4.0
        assert 0.0 <= y <= 4.0


def test_lawnmower_alternates_direction():
    bounds = (0.0, 0.0, 4.0, 4.0)
    wps = coverage.lawnmower(bounds, spacing=1.0, margin=0.5)
    # First pass goes bottom->top, second top->bottom.
    assert wps[0][1] < wps[1][1]
    assert wps[2][1] > wps[3][1]


def test_lawnmower_tiny_cell_returns_center():
    wps = coverage.lawnmower((0.0, 0.0, 0.2, 0.2), spacing=1.0, margin=0.5)
    assert len(wps) == 1
    assert abs(wps[0][0] - 0.1) < 1e-9


# -- collision ----------------------------------------------------------

def test_no_neighbors_keeps_desired():
    v = collision.avoid((1.0, 0.0), (0.0, 0.0), [], radius=1.0, gain=0.5,
                        max_speed=2.0)
    assert abs(v[0] - 1.0) < 1e-9
    assert abs(v[1]) < 1e-9


def test_close_neighbor_pushes_away():
    # Neighbor on +x -> repulsion has a -x component.
    rx, ry = collision.separation_velocity(
        (0.0, 0.0), [(0.5, 0.0)], radius=1.0, gain=1.0)
    assert rx < 0.0


def test_neighbor_outside_radius_ignored():
    rx, ry = collision.separation_velocity(
        (0.0, 0.0), [(5.0, 0.0)], radius=1.0, gain=1.0)
    assert rx == 0.0 and ry == 0.0


def test_avoid_clamps_to_max_speed():
    v = collision.avoid((10.0, 0.0), (0.0, 0.0), [(0.2, 0.0)],
                        radius=1.0, gain=5.0, max_speed=2.0)
    assert math.hypot(*v) <= 2.0 + 1e-9
