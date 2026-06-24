"""Coverage-path generation within an assigned sub-region.

A boustrophedon (lawnmower) sweep visits a rectangular cell with parallel
passes, so a drone actually searches its allocated area rather than hovering at
the centre. Pure functions, unit-testable.
"""


def lawnmower(bounds, spacing, margin=0.5):
    """Boustrophedon waypoints covering the bounds rectangle.

    bounds  : (x_min, y_min, x_max, y_max)
    spacing : lateral gap between passes [m]
    margin  : inset from the cell edge [m]

    Passes run along y; successive passes alternate direction and step along x.
    """
    x_min, y_min, x_max, y_max = bounds
    x0, x1 = x_min + margin, x_max - margin
    y0, y1 = y_min + margin, y_max - margin

    if x1 < x0 or y1 < y0:
        # Cell too small for the margin: just cover its centre.
        return [((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)]

    spacing = max(spacing, 1e-3)
    n_passes = max(1, int((x1 - x0) / spacing) + 1)

    waypoints = []
    for i in range(n_passes):
        x = x0 + i * spacing if n_passes > 1 else (x0 + x1) / 2.0
        x = min(x, x1)
        if i % 2 == 0:
            waypoints.append((x, y0))
            waypoints.append((x, y1))
        else:
            waypoints.append((x, y1))
            waypoints.append((x, y0))
    return waypoints
