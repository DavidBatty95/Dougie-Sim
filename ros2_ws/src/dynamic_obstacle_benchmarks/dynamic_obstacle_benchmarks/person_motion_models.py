from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .config import Bounds
from .geometry import Pose2D, Velocity2D, clamp, distance_xy


@dataclass
class PersonState:
    person_id: int
    pose: Pose2D
    velocity: Velocity2D
    radius_m: float
    behaviour: str
    speed_mps: float = 0.5
    target_index: int = 0
    paused_until_s: float = 0.0
    waypoints: list[Pose2D] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "id": self.person_id,
            "pose": {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw},
            "velocity": {"x": self.velocity.vx, "y": self.velocity.vy, "yaw": self.velocity.wz},
            "radius_m": self.radius_m,
            "behaviour": self.behaviour,
            "speed_mps": self.speed_mps,
        }


def build_person(
    person_id: int,
    behaviour: str,
    rng: random.Random,
    bounds: Bounds,
    radius_m: float,
    speed_mps: float,
) -> PersonState:
    waypoints = _waypoints_for_behaviour(behaviour, rng, bounds)
    if len(waypoints) < 2:
        raise ValueError(f"behaviour {behaviour} did not produce enough waypoints")
    return PersonState(person_id, waypoints[0], Velocity2D(), radius_m, behaviour, speed_mps=speed_mps, waypoints=waypoints)


def update_person(person: PersonState, dt_s: float, now_s: float, speed_mps: float, bounds: Bounds) -> None:
    if person.behaviour == "stop_start" and now_s < person.paused_until_s:
        person.velocity = Velocity2D()
        return

    target = person.waypoints[person.target_index]
    dx = target.x - person.pose.x
    dy = target.y - person.pose.y
    distance = math.hypot(dx, dy)
    if distance < 0.10:
        person.target_index = (person.target_index + 1) % len(person.waypoints)
        target = person.waypoints[person.target_index]
        dx = target.x - person.pose.x
        dy = target.y - person.pose.y
        distance = max(math.hypot(dx, dy), 1e-9)
        if person.behaviour == "stop_start" and int(now_s * 10) % 17 == 0:
            person.paused_until_s = now_s + 0.8
            person.velocity = Velocity2D()
            return

    vx = speed_mps * dx / max(distance, 1e-9)
    vy = speed_mps * dy / max(distance, 1e-9)
    person.pose = Pose2D(
        clamp(person.pose.x + vx * dt_s, bounds.x_min, bounds.x_max),
        clamp(person.pose.y + vy * dt_s, bounds.y_min, bounds.y_max),
        math.atan2(vy, vx),
    )
    person.velocity = Velocity2D(vx, vy, 0.0)


def _waypoints_for_behaviour(behaviour: str, rng: random.Random, bounds: Bounds) -> list[Pose2D]:
    x0 = bounds.x_min
    x1 = bounds.x_max
    y0 = bounds.y_min
    y1 = bounds.y_max
    if behaviour == "straight_line_crossing":
        y = rng.uniform(y0 * 0.5, y1 * 0.5)
        return [Pose2D(x0 + 0.5, y), Pose2D(x1 - 0.5, -y), Pose2D(x0 + 0.5, y)]
    if behaviour == "random_waypoint":
        return [_random_pose(rng, bounds) for _ in range(4)]
    if behaviour == "corridor_bottleneck":
        x = rng.uniform(-0.4, 0.4)
        return [Pose2D(x, y0 + 0.5), Pose2D(x, y1 - 0.5), Pose2D(x, y0 + 0.5)]
    if behaviour == "opposing_flow":
        y = rng.uniform(-0.8, 0.8)
        return [Pose2D(x1 - 0.5, y), Pose2D(x0 + 0.5, y), Pose2D(x1 - 0.5, y)]
    if behaviour == "stop_start":
        y = rng.uniform(y0 * 0.6, y1 * 0.6)
        return [Pose2D(x0 + 0.6, y), Pose2D(x1 - 0.6, y), Pose2D(x0 + 0.6, y)]
    if behaviour == "occlusion_emergence":
        corner_x = x0 + 0.8 if rng.random() < 0.5 else x1 - 0.8
        return [Pose2D(corner_x, y0 + 0.8), Pose2D(0.0, 0.0), Pose2D(-corner_x, y1 - 0.8)]
    raise ValueError(f"unsupported person behaviour: {behaviour}")


def _random_pose(rng: random.Random, bounds: Bounds) -> Pose2D:
    return Pose2D(rng.uniform(bounds.x_min + 0.5, bounds.x_max - 0.5), rng.uniform(bounds.y_min + 0.5, bounds.y_max - 0.5), rng.uniform(-math.pi, math.pi))


def min_person_distance(pose: Pose2D, people: list[PersonState]) -> float:
    if not people:
        return math.inf
    return min(distance_xy(pose, person.pose) for person in people)
