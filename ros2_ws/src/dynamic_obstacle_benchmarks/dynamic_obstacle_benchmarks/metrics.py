from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import BenchmarkConfig
from .geometry import Pose2D, Velocity2D, circle_clearance, distance_xy, time_to_collision
from .goal_sampler import Goal
from .person_motion_models import PersonState


@dataclass
class SafetyEvent:
    time_s: float
    type: str
    person_id: int | None = None
    distance_m: float | None = None
    ttc_s: float | None = None
    robot_pose: Pose2D | None = None
    center_distance_m: float | None = None
    body_clearance_m: float | None = None

    def as_dict(self) -> dict:
        data = {
            "time_s": self.time_s,
            "type": self.type,
            "person_id": self.person_id,
            # Retained as the legacy center-distance field for existing consumers.
            "distance_m": self.distance_m,
            "center_distance_m": self.center_distance_m if self.center_distance_m is not None else self.distance_m,
            "body_clearance_m": self.body_clearance_m,
            "ttc_s": self.ttc_s,
        }
        if self.robot_pose is not None:
            data["robot_pose"] = {"x": self.robot_pose.x, "y": self.robot_pose.y, "yaw": self.robot_pose.yaw}
        return data


@dataclass
class GoalMetrics:
    index: int
    goal: Goal
    reached: bool = False
    failure_reason: str | None = None
    time_s: float = 0.0
    path_length_m: float = 0.0
    minimum_separation_m: float = math.inf
    minimum_center_distance_m: float = math.inf
    minimum_body_clearance_m: float = math.inf
    near_misses: int = 0
    critical_near_misses: int = 0
    collisions: int = 0

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "goal_pose": {"x": self.goal.pose.x, "y": self.goal.pose.y, "yaw": self.goal.pose.yaw},
            "reached": self.reached,
            "failure_reason": self.failure_reason,
            "time_s": self.time_s,
            "path_length_m": self.path_length_m,
            "minimum_separation_m": None if math.isinf(self.minimum_separation_m) else self.minimum_separation_m,
            "minimum_center_distance_m": None if math.isinf(self.minimum_center_distance_m) else self.minimum_center_distance_m,
            "minimum_body_clearance_m": None if math.isinf(self.minimum_body_clearance_m) else self.minimum_body_clearance_m,
            "near_misses": self.near_misses,
            "critical_near_misses": self.critical_near_misses,
            "collisions": self.collisions,
        }


@dataclass
class BenchmarkMetrics:
    goals_requested: int
    goals_reached: int = 0
    total_time_s: float = 0.0
    actual_path_length_m: float = 0.0
    ideal_path_length_m: float = 0.0
    stop_time_s: float = 0.0
    avoidance_time_s: float = 0.0
    moving_avoidance_time_s: float = 0.0
    post_avoidance_progress_m: float = 0.0
    moving_pedestrian_time_s: float = 0.0
    max_abs_safe_lateral_cmd_mps: float = 0.0
    minimum_separation_m: float = math.inf
    mean_separation_m: float = 0.0
    minimum_center_distance_m: float = math.inf
    mean_center_distance_m: float = 0.0
    minimum_body_clearance_m: float = math.inf
    mean_body_clearance_m: float = 0.0
    minimum_ttc_s: float = math.inf
    collisions: int = 0
    near_misses: int = 0
    critical_near_misses: int = 0
    time_below_warning_distance_s: float = 0.0
    time_below_near_miss_distance_s: float = 0.0
    time_below_critical_distance_s: float = 0.0
    emergency_stops: int = 0
    avoidance_interventions: int = 0
    stale_events: int = 0
    speed_limit_violations: int = 0
    left_valid_area: bool = False
    no_movement: bool = False
    fallen: bool = False
    valid_trial: bool = True
    invalid_reason: str | None = None
    goal_metrics: list[GoalMetrics] = field(default_factory=list)
    events: list[SafetyEvent] = field(default_factory=list)
    _last_pose: Pose2D | None = None
    _sample_count: int = 0
    _separation_sum: float = 0.0
    _clearance_sum: float = 0.0
    _near_active: set[int] = field(default_factory=set)
    _critical_active: set[int] = field(default_factory=set)
    _collision_active: set[int] = field(default_factory=set)
    _critical_ttc_active: set[int] = field(default_factory=set)
    _near_duration_s: dict[int, float] = field(default_factory=dict)
    _critical_duration_s: dict[int, float] = field(default_factory=dict)
    _speed_violation_active: bool = False

    @property
    def average_speed_mps(self) -> float:
        return self.actual_path_length_m / self.total_time_s if self.total_time_s > 1e-9 else 0.0

    @property
    def efficiency(self) -> float:
        if self.actual_path_length_m < 1e-6:
            return 0.0
        return max(0.0, min(1.0, self.ideal_path_length_m / self.actual_path_length_m))


class MetricsMonitor:
    def __init__(self, config: BenchmarkConfig) -> None:
        self.config = config
        self.metrics = BenchmarkMetrics(config.goals_count)

    def start_goal(self, goal: Goal) -> GoalMetrics:
        gm = GoalMetrics(goal.index, goal)
        self.metrics.goal_metrics.append(gm)
        return gm

    def update(self, now_s: float, dt_s: float, robot_pose: Pose2D, robot_velocity: Velocity2D, people: list[PersonState]) -> None:
        self.metrics.total_time_s = now_s
        if self.metrics._last_pose is not None:
            step = distance_xy(robot_pose, self.metrics._last_pose)
            if step > 0.002:
                self.metrics.actual_path_length_m += step
                if self.metrics.goal_metrics:
                    self.metrics.goal_metrics[-1].path_length_m += step
            if math.hypot(robot_velocity.vx, robot_velocity.vy) < 0.03:
                self.metrics.stop_time_s += dt_s
        self.metrics._last_pose = robot_pose

        speed = math.hypot(robot_velocity.vx, robot_velocity.vy)
        violating = speed > self.config.max_speed_mps + 1e-3
        if violating and not self.metrics._speed_violation_active:
            self.metrics.speed_limit_violations += 1
            self.metrics.events.append(SafetyEvent(now_s, "speed_limit_violation", robot_pose=robot_pose))
        self.metrics._speed_violation_active = violating
        if not self.config.pose_in_valid_area(robot_pose):
            self.metrics.left_valid_area = True

        if not people:
            self._expire_missing_people(set())
            return
        if any(math.hypot(person.velocity.vx, person.velocity.vy) >= 0.10 for person in people):
            self.metrics.moving_pedestrian_time_s += dt_s
        present_ids = {person.person_id for person in people}
        min_center_distance = math.inf
        min_clearance = math.inf
        for person in people:
            center_distance = distance_xy(robot_pose, person.pose)
            clearance = circle_clearance(robot_pose, self.config.robot_radius_m, person.pose, person.radius_m)
            min_center_distance = min(min_center_distance, center_distance)
            min_clearance = min(min_clearance, clearance)
            self._update_distance_events(now_s, dt_s, person.person_id, center_distance, clearance, robot_pose)
            ttc = time_to_collision(
                robot_pose,
                robot_velocity,
                person.pose,
                person.velocity,
                self.config.collision_distance_m,
                self.config.ttc_horizon_s,
            )
            if ttc is not None:
                self.metrics.minimum_ttc_s = min(self.metrics.minimum_ttc_s, ttc)
                if ttc <= self.config.ttc_critical_s:
                    if person.person_id not in self.metrics._critical_ttc_active:
                        self.metrics._critical_ttc_active.add(person.person_id)
                        self.metrics.events.append(SafetyEvent(
                            now_s, "critical_ttc", person.person_id, center_distance, ttc, robot_pose,
                            center_distance, clearance,
                        ))
                else:
                    self.metrics._critical_ttc_active.discard(person.person_id)
            else:
                self.metrics._critical_ttc_active.discard(person.person_id)

        self._expire_missing_people(present_ids)

        self.metrics.minimum_separation_m = min(self.metrics.minimum_separation_m, min_center_distance)
        self.metrics.minimum_center_distance_m = min(self.metrics.minimum_center_distance_m, min_center_distance)
        self.metrics.minimum_body_clearance_m = min(self.metrics.minimum_body_clearance_m, min_clearance)
        self.metrics._sample_count += 1
        self.metrics._separation_sum += min_center_distance
        self.metrics._clearance_sum += min_clearance
        self.metrics.mean_separation_m = self.metrics._separation_sum / self.metrics._sample_count
        self.metrics.mean_center_distance_m = self.metrics.mean_separation_m
        self.metrics.mean_body_clearance_m = self.metrics._clearance_sum / self.metrics._sample_count
        if self.metrics.goal_metrics:
            goal = self.metrics.goal_metrics[-1]
            goal.minimum_separation_m = min(goal.minimum_separation_m, min_center_distance)
            goal.minimum_center_distance_m = min(goal.minimum_center_distance_m, min_center_distance)
            goal.minimum_body_clearance_m = min(goal.minimum_body_clearance_m, min_clearance)

    def complete_goal(self, goal_metric: GoalMetrics, reached: bool, reason: str | None, elapsed_s: float) -> None:
        goal_metric.reached = reached
        goal_metric.failure_reason = reason
        goal_metric.time_s = elapsed_s
        if reached:
            self.metrics.goals_reached += 1

    def _expire_missing_people(self, present_ids: set[int]) -> None:
        self.metrics._near_active.intersection_update(present_ids)
        self.metrics._critical_active.intersection_update(present_ids)
        self.metrics._collision_active.intersection_update(present_ids)
        self.metrics._critical_ttc_active.intersection_update(present_ids)
        self.metrics._near_duration_s = {
            person_id: duration
            for person_id, duration in self.metrics._near_duration_s.items()
            if person_id in present_ids
        }
        self.metrics._critical_duration_s = {
            person_id: duration
            for person_id, duration in self.metrics._critical_duration_s.items()
            if person_id in present_ids
        }

    def _update_distance_events(self, now_s: float, dt_s: float, person_id: int, center_distance: float, clearance: float, robot_pose: Pose2D) -> None:
        if center_distance < self.config.warning_distance_m:
            self.metrics.time_below_warning_distance_s += dt_s
        if center_distance < self.config.near_miss_distance_m:
            self.metrics.time_below_near_miss_distance_s += dt_s
            duration = self.metrics._near_duration_s.get(person_id, 0.0) + dt_s
            self.metrics._near_duration_s[person_id] = duration
            if person_id not in self.metrics._near_active and duration + 1e-9 >= self.config.near_miss_min_duration_s:
                self.metrics._near_active.add(person_id)
                self.metrics.near_misses += 1
                if self.metrics.goal_metrics:
                    self.metrics.goal_metrics[-1].near_misses += 1
                self.metrics.events.append(SafetyEvent(
                    now_s - duration + dt_s, "near_miss", person_id, center_distance, None, robot_pose,
                    center_distance, clearance,
                ))
        else:
            self.metrics._near_active.discard(person_id)
            self.metrics._near_duration_s.pop(person_id, None)

        if center_distance < self.config.critical_distance_m:
            self.metrics.time_below_critical_distance_s += dt_s
            duration = self.metrics._critical_duration_s.get(person_id, 0.0) + dt_s
            self.metrics._critical_duration_s[person_id] = duration
            if person_id not in self.metrics._critical_active and duration + 1e-9 >= self.config.near_miss_min_duration_s:
                self.metrics._critical_active.add(person_id)
                self.metrics.critical_near_misses += 1
                if self.metrics.goal_metrics:
                    self.metrics.goal_metrics[-1].critical_near_misses += 1
                self.metrics.events.append(SafetyEvent(
                    now_s - duration + dt_s, "critical_near_miss", person_id, center_distance, None, robot_pose,
                    center_distance, clearance,
                ))
        else:
            self.metrics._critical_active.discard(person_id)
            self.metrics._critical_duration_s.pop(person_id, None)

        if clearance <= 0.0:
            if person_id not in self.metrics._collision_active:
                self.metrics._collision_active.add(person_id)
                self.metrics.collisions += 1
                if self.metrics.goal_metrics:
                    self.metrics.goal_metrics[-1].collisions += 1
                self.metrics.events.append(SafetyEvent(
                    now_s, "collision", person_id, center_distance, None, robot_pose,
                    center_distance, clearance,
                ))
        elif clearance > 0.05:
            self.metrics._collision_active.discard(person_id)
