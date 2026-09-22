from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable

from .config import BenchmarkConfig
from .geometry import Pose2D, distance_xy
from .person_motion_models import PersonState, min_person_distance


@dataclass(frozen=True)
class Goal:
    index: int
    pose: Pose2D

    def as_dict(self) -> dict:
        return {"index": self.index, "pose": {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw}}


class GoalSampler:
    def __init__(
        self,
        config: BenchmarkConfig,
        rng: random.Random,
        pose_validator: Callable[[Pose2D, Pose2D], bool] | None = None,
    ) -> None:
        self.config = config
        self.rng = rng
        self.pose_validator = pose_validator

    def sample(self, start: Pose2D, people: list[PersonState]) -> list[Goal]:
        if self.config.goal_mode == "fixed":
            return self._fixed_goals()
        if self.config.goal_mode == "curriculum":
            return self._curriculum_goals(start, people)
        if self.config.goal_mode == "crossing_challenge":
            return self._crossing_goals(start, people)
        return self._random_goals(start, people)

    def sample_random_goal(self, index: int, start: Pose2D, people: list[PersonState]) -> Goal:
        return Goal(index, self._sample_valid_pose(start, people))

    def _fixed_goals(self) -> list[Goal]:
        if len(self.config.fixed_goals) < self.config.goals_count:
            raise ValueError("fixed goal mode requires at least goals.count entries")
        return [Goal(i, Pose2D(float(g["x"]), float(g["y"]), float(g.get("yaw", 0.0)))) for i, g in enumerate(self.config.fixed_goals[: self.config.goals_count])]

    def _random_goals(self, start: Pose2D, people: list[PersonState]) -> list[Goal]:
        goals: list[Goal] = []
        previous = start
        for index in range(self.config.goals_count):
            goal = self.sample_random_goal(index, previous, people)
            goals.append(goal)
            previous = goal.pose
        return goals

    def _curriculum_goals(self, start: Pose2D, people: list[PersonState]) -> list[Goal]:
        goals: list[Goal] = []
        cx = (self.config.bounds.x_min + self.config.bounds.x_max) / 2.0
        cy = (self.config.bounds.y_min + self.config.bounds.y_max) / 2.0
        max_radius = min(self.config.bounds.x_max - self.config.bounds.x_min, self.config.bounds.y_max - self.config.bounds.y_min) / 2.0 - self.config.min_distance_from_walls_m
        for index in range(self.config.goals_count):
            angle = 2.0 * math.pi * index / max(1, self.config.goals_count)
            radius = max_radius * (0.35 + 0.65 * (index + 1) / self.config.goals_count)
            goals.append(Goal(index, Pose2D(cx + radius * math.cos(angle), cy + radius * math.sin(angle), angle)))
        return goals

    def _crossing_goals(self, start: Pose2D, people: list[PersonState]) -> list[Goal]:
        left = Pose2D(self.config.bounds.x_min + self.config.min_distance_from_walls_m, 0.0, 0.0)
        right = Pose2D(self.config.bounds.x_max - self.config.min_distance_from_walls_m, 0.0, math.pi)
        return [Goal(index, right if index % 2 == 0 else left) for index in range(self.config.goals_count)]

    def _sample_valid_pose(self, previous: Pose2D, people: list[PersonState]) -> Pose2D:
        b = self.config.goal_bounds or self.config.bounds
        margin = self.config.min_distance_from_walls_m
        for _ in range(self.config.max_sampling_attempts):
            x = self.rng.uniform(b.x_min + margin, b.x_max - margin)
            y = self.rng.uniform(b.y_min + margin, b.y_max - margin)
            if self.config.onboard_only:
                heading = previous.yaw + self.rng.uniform(-self.config.max_goal_heading_change_rad, self.config.max_goal_heading_change_rad)
                distance = self.rng.uniform(self.config.min_goal_distance_m, self.config.max_goal_distance_m or 2.5)
                x, y = previous.x + distance * math.cos(heading), previous.y + distance * math.sin(heading)
                if not (b.x_min + margin <= x <= b.x_max - margin and b.y_min + margin <= y <= b.y_max - margin):
                    continue
            pose = Pose2D(
                x,
                y,
                math.atan2(y - previous.y, x - previous.x),
            )
            if distance_xy(pose, previous) < self.config.min_goal_distance_m:
                continue
            inside_goal_bounds = b.x_min <= previous.x <= b.x_max and b.y_min <= previous.y <= b.y_max
            if inside_goal_bounds and self.config.max_goal_distance_m is not None and distance_xy(pose, previous) > self.config.max_goal_distance_m:
                continue
            heading_change = math.atan2(
                math.sin(pose.yaw - previous.yaw),
                math.cos(pose.yaw - previous.yaw),
            )
            if abs(heading_change) > self.config.max_goal_heading_change_rad:
                continue
            if min_person_distance(pose, people) < self.config.spawn_min_distance_from_goal_m:
                continue
            if self.pose_validator is not None and not self.pose_validator(previous, pose):
                continue
            return pose
        raise RuntimeError("could not sample enough valid goals; relax bounds/distances or reduce goal count")
