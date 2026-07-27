from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Velocity2D:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def distance_xy(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def circle_clearance(a: Pose2D, a_radius: float, b: Pose2D, b_radius: float) -> float:
    return distance_xy(a, b) - a_radius - b_radius


def polygon_circle_clearance(polygon: list[tuple[float, float]], center: Pose2D, radius: float) -> float:
    if len(polygon) < 3:
        raise ValueError("polygon must contain at least three points")
    if _point_in_polygon(center.x, center.y, polygon):
        return -radius
    min_distance = min(_point_segment_distance(center.x, center.y, a, b) for a, b in _segments(polygon))
    return min_distance - radius


def time_to_collision(
    robot_pose: Pose2D,
    robot_velocity: Velocity2D,
    person_pose: Pose2D,
    person_velocity: Velocity2D,
    combined_radius: float,
    horizon_s: float,
) -> float | None:
    px = person_pose.x - robot_pose.x
    py = person_pose.y - robot_pose.y
    vx = person_velocity.vx - robot_velocity.vx
    vy = person_velocity.vy - robot_velocity.vy
    vv = vx * vx + vy * vy
    if vv < 1e-9:
        return 0.0 if math.hypot(px, py) <= combined_radius else None
    t_star = clamp(-(px * vx + py * vy) / vv, 0.0, horizon_s)
    d_star = math.hypot(px + vx * t_star, py + vy * t_star)
    if d_star <= combined_radius:
        return t_star
    return None


def _segments(polygon: list[tuple[float, float]]):
    for index, point in enumerate(polygon):
        yield point, polygon[(index + 1) % len(polygon)]


def _point_segment_distance(px: float, py: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp(((px - ax) * dx + (py - ay) * dy) / length_sq, 0.0, 1.0)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _point_in_polygon(px: float, py: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        intersects = ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / max(yj - yi, 1e-12) + xi)
        if intersects:
            inside = not inside
        j = i
    return inside
