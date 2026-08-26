from __future__ import annotations

import csv
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .config import BenchmarkConfig
from .goal_sampler import Goal
from .metrics import BenchmarkMetrics
from .person_motion_models import PersonState
from .scoring import ScoreBreakdown


class ResultWriter:
    def __init__(self, config: BenchmarkConfig, output_path: str | None) -> None:
        output = Path(output_path) if output_path else Path(config.output_dir) / f"person_avoidance_seed_{config.seed}.json"
        self.output_path = output
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, config: BenchmarkConfig, metrics: BenchmarkMetrics, score: ScoreBreakdown, goals: list[Goal], people: list[PersonState], scenario_path: Path) -> Path:
        data = {
            "benchmark_name": config.name,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "seed": config.seed,
            "mode": config.mode,
            "backend": config.backend,
            "environment": "mock" if config.backend == "mock" else config.backend,
            "record_rosbag": config.record_rosbag,
            "people_count": config.people_count,
            "pedestrian_scenario": config.pedestrian_scenario,
            "goals_requested": metrics.goals_requested,
            "goals_reached": metrics.goals_reached,
            "valid_trial": metrics.valid_trial,
            "invalid_reason": metrics.invalid_reason,
            "success": metrics.valid_trial and metrics.goals_reached == metrics.goals_requested and metrics.collisions == 0 and not metrics.no_movement and not metrics.fallen,
            "failure_reason": _failure_reason(metrics),
            "total_time_s": round(metrics.total_time_s, 3),
            "actual_path_length_m": round(metrics.actual_path_length_m, 3),
            "ideal_path_length_m": round(metrics.ideal_path_length_m, 3),
            "efficiency": round(metrics.efficiency, 3),
            "average_speed_mps": round(metrics.average_speed_mps, 3),
            "avoidance_time_s": round(metrics.avoidance_time_s, 3),
            "moving_avoidance_time_s": round(metrics.moving_avoidance_time_s, 3),
            "post_avoidance_progress_m": round(metrics.post_avoidance_progress_m, 3),
            "moving_pedestrian_time_s": round(metrics.moving_pedestrian_time_s, 3),
            "max_abs_safe_lateral_cmd_mps": round(metrics.max_abs_safe_lateral_cmd_mps, 4),
            "collisions": metrics.collisions,
            "fallen": metrics.fallen,
            "no_movement": metrics.no_movement,
            "near_misses": metrics.near_misses,
            "critical_near_misses": metrics.critical_near_misses,
            "minimum_separation_m": None if metrics.minimum_separation_m == float("inf") else round(metrics.minimum_separation_m, 3),
            "minimum_center_distance_m": None if metrics.minimum_center_distance_m == float("inf") else round(metrics.minimum_center_distance_m, 3),
            "mean_center_distance_m": round(metrics.mean_center_distance_m, 3),
            "minimum_body_clearance_m": None if metrics.minimum_body_clearance_m == float("inf") else round(metrics.minimum_body_clearance_m, 3),
            "mean_body_clearance_m": round(metrics.mean_body_clearance_m, 3),
            "minimum_ttc_s": None if metrics.minimum_ttc_s == float("inf") else round(metrics.minimum_ttc_s, 3),
            "time_below_warning_distance_s": round(metrics.time_below_warning_distance_s, 3),
            "time_below_near_miss_distance_s": round(metrics.time_below_near_miss_distance_s, 3),
            "time_below_critical_distance_s": round(metrics.time_below_critical_distance_s, 3),
            "emergency_stops": metrics.emergency_stops,
            "avoidance_interventions": metrics.avoidance_interventions,
            "stale_events": metrics.stale_events,
            "speed_limit_violations": metrics.speed_limit_violations,
            "left_valid_area": metrics.left_valid_area,
            **score.as_dict(),
            "goals": [goal_metric.as_dict() for goal_metric in metrics.goal_metrics],
            "events": [event.as_dict() for event in metrics.events],
            "scenario_file": str(scenario_path),
        }
        self.output_path.write_text(json.dumps(data, indent=2))
        self._write_csvs(metrics)
        return self.output_path

    def write_scenario(self, config: BenchmarkConfig, goals: list[Goal], people: list[PersonState]) -> Path:
        scenario_path = self.output_path.with_name(self.output_path.stem + "_scenario.yaml")
        scenario = {
            "seed": config.seed,
            "mode": config.mode,
            "backend": config.backend,
            "pedestrian_scenario": config.pedestrian_scenario,
            "robot_start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "goals": [goal.as_dict() for goal in goals],
            "people": [person.as_dict() | {"waypoints": [{"x": p.x, "y": p.y, "yaw": p.yaw} for p in person.waypoints]} for person in people],
        }
        scenario_path.write_text(yaml.safe_dump(scenario, sort_keys=False))
        return scenario_path

    def _write_csvs(self, metrics: BenchmarkMetrics) -> None:
        goals_csv = self.output_path.with_name(self.output_path.stem + "_goals.csv")
        with goals_csv.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["index", "reached", "failure_reason", "time_s", "path_length_m", "minimum_separation_m", "minimum_center_distance_m", "minimum_body_clearance_m", "near_misses", "critical_near_misses", "collisions"])
            writer.writeheader()
            for goal in metrics.goal_metrics:
                row = goal.as_dict()
                row.pop("goal_pose", None)
                writer.writerow(row)
        events_csv = self.output_path.with_name(self.output_path.stem + "_events.csv")
        with events_csv.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["time_s", "type", "person_id", "distance_m", "center_distance_m", "body_clearance_m", "ttc_s"])
            writer.writeheader()
            for event in metrics.events:
                row = event.as_dict()
                row.pop("robot_pose", None)
                writer.writerow(row)


def _failure_reason(metrics: BenchmarkMetrics) -> str | None:
    if not metrics.valid_trial:
        return metrics.invalid_reason or "invalid_trial"
    if metrics.collisions:
        return "collision"
    if metrics.fallen:
        return "fallen"
    for goal in metrics.goal_metrics:
        if not goal.reached and goal.failure_reason:
            return goal.failure_reason
    if metrics.no_movement:
        return "no_movement"
    if metrics.goals_reached < metrics.goals_requested:
        return "one_or_more_goals_failed"
    return None
