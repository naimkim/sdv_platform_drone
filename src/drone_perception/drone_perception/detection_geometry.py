"""Project 2D object detections into relative obstacle positions.

A pinhole camera model turns a YOLO bounding box (pixel column + box height)
into a bearing and a range, and hence a relative position the avoidance planner
can repel from. The detector itself (YOLO + TensorRT on Jetson) is upstream;
this is the detection -> navigation geometry bridge, which is unit-testable
without a camera.

Body frame is REP-103: x forward, y left, z up. A detection on the right side
of the image (pixel column > centre) yields a negative y (to the right).
"""

import math


def focal_length_px(image_width, hfov_rad):
    """Horizontal focal length in pixels from image width and horizontal FOV."""
    return (image_width / 2.0) / math.tan(hfov_rad / 2.0)


def bearing_rad(u_px, image_width, focal_px):
    """Angle of an image column off the optical axis (+ to the right)."""
    return math.atan2(u_px - image_width / 2.0, focal_px)


def estimate_distance(real_size_m, size_px, focal_px):
    """Range estimate from apparent size via the pinhole relation.

    distance = real_size * focal_length / apparent_size.
    """
    if size_px <= 0.0:
        return float('inf')
    return real_size_m * focal_px / size_px


def obstacle_offset(u_px, image_width, hfov_rad, distance):
    """Relative (x_forward, y_left) position of a detection at given range."""
    f = focal_length_px(image_width, hfov_rad)
    theta = bearing_rad(u_px, image_width, f)
    x_forward = distance * math.cos(theta)
    y_left = -distance * math.sin(theta)
    return x_forward, y_left
