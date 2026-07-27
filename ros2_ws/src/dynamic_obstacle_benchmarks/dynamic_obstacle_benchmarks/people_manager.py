from __future__ import annotations

import random

from .config import BenchmarkConfig
from .geometry import Pose2D, distance_xy
from .person_motion_models import PersonState, build_person, update_person


class PeopleManager:
    def __init__(self, config: BenchmarkConfig, rng: random.Random) -> None:
        self.config = config
        self.rng = rng
        self.people: list[PersonState] = []

    def reset(self, robot_start: Pose2D) -> list[PersonState]:
        self.people = []
        behaviours = self.config.behaviours or ["random_waypoint"]
        for person_id in range(self.config.people_count):
            for _ in range(self.config.max_sampling_attempts):
                behaviour = behaviours[person_id % len(behaviours)]
                speed = self.rng.uniform(self.config.people_speed_min_mps, self.config.people_speed_max_mps)
                person = build_person(person_id, behaviour, self.rng, self.config.bounds, self.config.person_radius_m, speed)
                if distance_xy(person.pose, robot_start) < self.config.spawn_min_distance_from_robot_m:
                    continue
                if any(distance_xy(person.pose, other.pose) < self.config.spawn_min_distance_between_people_m for other in self.people):
                    continue
                self.people.append(person)
                break
            else:
                raise RuntimeError("could not spawn non-colliding people; relax spawn distances or reduce people count")
        return self.people

    def update(self, dt_s: float, now_s: float) -> list[PersonState]:
        for person in self.people:
            update_person(person, dt_s, now_s, person.speed_mps, self.config.bounds)
        return self.people

    def as_scenario(self) -> list[dict]:
        return [person.as_dict() | {"waypoints": [{"x": p.x, "y": p.y, "yaw": p.yaw} for p in person.waypoints]} for person in self.people]
