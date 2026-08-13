"""Pure LaserScan-to-body preview conversion matching autorunlida."""

import math


def retained_preview_points(points, points_age, retention_seconds=2.0):
    """Return cached display points only inside the bounded stale window."""
    if points_age is None or points_age < 0.0 or points_age >= retention_seconds:
        return []
    return points


def preview_points(ranges, angle_min, angle_increment, range_min, range_max,
                   yaw_deg=177.0, offset_x=0.0, offset_y=0.035,
                   visible_x=(-0.55, 1.85), visible_y=(-2.55, 2.55),
                   max_range=6.0, max_points=720):
    minimum = max(float(range_min), 0.05)
    maximum = min(float(range_max), float(max_range))
    yaw = math.radians(float(yaw_deg))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    points = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance < minimum or distance > maximum:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        sensor_x = distance * math.cos(angle)
        sensor_y = distance * math.sin(angle)
        body_x = cos_yaw * sensor_x - sin_yaw * sensor_y + offset_x
        body_y = sin_yaw * sensor_x + cos_yaw * sensor_y + offset_y
        if (visible_x[0] <= body_x <= visible_x[1] and
                visible_y[0] <= body_y <= visible_y[1]):
            points.append((round(body_x, 4), round(body_y, 4)))
    if len(points) > max_points:
        step = max(1, math.ceil(len(points) / max_points))
        points = points[::step][:max_points]
    return points
