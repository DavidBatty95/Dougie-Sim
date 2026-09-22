from __future__ import annotations

import csv
import hashlib
import io
import json
import os
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import yaml

from .config import BenchmarkConfig
from .goal_sampler import Goal
from .metrics import BenchmarkMetrics
from .person_motion_models import PersonState
from .scoring import ScoreBreakdown


RESULT_SCHEMA_VERSION = 2


class ResultWriter:
    def __init__(self, config: BenchmarkConfig, output_path: str | None) -> None:
        output = Path(output_path) if output_path else Path(config.output_dir) / f"person_avoidance_seed_{config.seed}.json"
        self.output_path = output
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.run_id = str(uuid4())

    def write(self, config: BenchmarkConfig, metrics: BenchmarkMetrics, score: ScoreBreakdown, goals: list[Goal], people: list[PersonState], scenario_path: Path) -> Path:
        data_sources = _resolved_data_sources(config)
        artifact_paths = self._write_csvs(metrics)
        artifacts = {
            "scenario": _artifact_reference(scenario_path),
            "goals_csv": _artifact_reference(artifact_paths["goals_csv"]),
            "events_csv": _artifact_reference(artifact_paths["events_csv"]),
        }
        data = {
            "schema_version": RESULT_SCHEMA_VERSION,
            "run_id": self.run_id,
            "benchmark_name": config.name,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "seed": config.seed,
            "mode": config.mode,
            "backend": config.backend,
            "onboard_only": config.onboard_only,
            "visual_session": config.visual_session,
            "session_end_reason": (
                metrics.invalid_reason or metrics.session_end_reason or ("fallen" if metrics.fallen else
                    "duration_elapsed" if metrics.total_time_s >= config.total_timeout_s else
                    "goal_budget_reached" if len(goals) >= config.goals_count else "stopped")
            ) if config.visual_session else None,
            "goal_frame": config.map_frame,
            "scoring_frame": "world" if config.onboard_only else config.map_frame,
            "environment": "mock" if config.backend == "mock" else config.backend,
            "record_rosbag": config.record_rosbag,
            "people_count": config.people_count,
            "people_requested": config.people_count,
            "people_observed": metrics.people_observed,
            "people_active_initial": metrics.people_active_initial,
            "people_active_minimum": metrics.people_active_minimum,
            "people_active_final": metrics.people_active_final,
            "data_sources": {
                "requested": data_sources,
                "observed": {
                    "control_people": metrics.control_people_source_observed,
                    "scoring_people": (
                        "synthetic_ground_truth" if config.backend == "mock"
                        else "mujoco_simulator_truth"
                    ),
                    "scan": metrics.scan_source_observed,
                },
            },
            "data_health": {
                "require_exact_people_count": config.require_exact_people_count,
                "odom_stale_timeout_s": config.odom_stale_timeout_s,
                "pedestrian_stale_timeout_s": config.pedestrian_stale_timeout_s,
                "scan_stale_timeout_s": config.scan_stale_timeout_s,
                "max_odom_age_s": round(metrics.max_odom_age_s, 3),
                "max_pedestrian_age_s": round(metrics.max_pedestrian_age_s, 3),
                "max_scan_age_s": round(metrics.max_scan_age_s, 3),
                "max_control_people_source_age_s": round(
                    metrics.max_control_people_source_age_s, 3
                ),
                "max_odom_sample_stall_s": round(metrics.max_odom_sample_stall_s, 3),
                "max_pedestrian_sample_stall_s": round(metrics.max_pedestrian_sample_stall_s, 3),
                "max_scan_sample_stall_s": round(metrics.max_scan_sample_stall_s, 3),
                "scan_messages_observed": metrics.scan_messages_observed,
            },
            "pedestrian_scenario": config.pedestrian_scenario,
            "goals_requested": metrics.goals_requested,
            "goals_reached": metrics.goals_reached,
            "valid_trial": metrics.valid_trial,
            "invalid_reason": metrics.invalid_reason,
            "success": not config.visual_session and metrics.valid_trial and metrics.goals_reached == metrics.goals_requested and metrics.collisions == 0 and not metrics.no_movement and not metrics.fallen,
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
            "minimum_dcpa_m": None if metrics.minimum_dcpa_m == float("inf") else round(metrics.minimum_dcpa_m, 3),
            "tcpa_at_minimum_dcpa_s": None if metrics.tcpa_at_minimum_dcpa_s == float("inf") else round(metrics.tcpa_at_minimum_dcpa_s, 3),
            "time_below_warning_distance_s": round(metrics.time_below_warning_distance_s, 3),
            "time_below_near_miss_distance_s": round(metrics.time_below_near_miss_distance_s, 3),
            "time_below_critical_distance_s": round(metrics.time_below_critical_distance_s, 3),
            "warning_distance_wall_time_s": round(metrics.warning_distance_wall_time_s, 3),
            "near_miss_distance_wall_time_s": round(metrics.near_miss_distance_wall_time_s, 3),
            "critical_distance_wall_time_s": round(metrics.critical_distance_wall_time_s, 3),
            "warning_distance_person_time_s": round(metrics.warning_distance_person_time_s, 3),
            "near_miss_distance_person_time_s": round(metrics.near_miss_distance_person_time_s, 3),
            "critical_distance_person_time_s": round(metrics.critical_distance_person_time_s, 3),
            "emergency_stops": metrics.emergency_stops,
            "avoidance_interventions": metrics.avoidance_interventions,
            "yield_episodes": metrics.yield_episodes,
            "resume_episodes": metrics.resume_episodes,
            "yield_time_s": round(metrics.yield_time_s, 3),
            "stale_events": metrics.stale_events,
            "speed_limit_violations": metrics.speed_limit_violations,
            "left_valid_area": metrics.left_valid_area,
            **score.as_dict(),
            "goals": [goal_metric.as_dict() for goal_metric in metrics.goal_metrics],
            "events": [event.as_dict() for event in metrics.events],
            "scenario_file": str(scenario_path),
            "artifacts": artifacts,
        }
        _atomic_write_text(self.output_path, json.dumps(data, indent=2))
        return self.output_path

    def write_scenario(self, config: BenchmarkConfig, goals: list[Goal], people: list[PersonState]) -> Path:
        scenario_path = self.output_path.with_name(self.output_path.stem + "_scenario.yaml")
        scenario = {
            "schema_version": RESULT_SCHEMA_VERSION,
            "run_id": self.run_id,
            "seed": config.seed,
            "mode": config.mode,
            "backend": config.backend,
            "pedestrian_scenario": config.pedestrian_scenario,
            "robot_start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "goals": [goal.as_dict() for goal in goals],
            "people": [person.as_dict() | {"waypoints": [{"x": p.x, "y": p.y, "yaw": p.yaw} for p in person.waypoints]} for person in people],
        }
        _atomic_write_text(scenario_path, yaml.safe_dump(scenario, sort_keys=False))
        return scenario_path

    def _write_csvs(self, metrics: BenchmarkMetrics) -> dict[str, Path]:
        goals_csv = self.output_path.with_name(self.output_path.stem + "_goals.csv")
        handle = io.StringIO(newline="")
        writer = csv.DictWriter(handle, fieldnames=["index", "reached", "failure_reason", "time_s", "path_length_m", "minimum_separation_m", "minimum_center_distance_m", "minimum_body_clearance_m", "near_misses", "critical_near_misses", "collisions"])
        writer.writeheader()
        for goal in metrics.goal_metrics:
            row = goal.as_dict()
            row.pop("goal_pose", None)
            writer.writerow(row)
        _atomic_write_text(goals_csv, handle.getvalue())
        events_csv = self.output_path.with_name(self.output_path.stem + "_events.csv")
        handle = io.StringIO(newline="")
        writer = csv.DictWriter(handle, fieldnames=["time_s", "type", "person_id", "distance_m", "center_distance_m", "body_clearance_m", "ttc_s", "tcpa_s", "dcpa_m"])
        writer.writeheader()
        for event in metrics.events:
            row = event.as_dict()
            row.pop("robot_pose", None)
            writer.writerow(row)
        _atomic_write_text(events_csv, handle.getvalue())
        return {"goals_csv": goals_csv, "events_csv": events_csv}


def _resolved_data_sources(config: BenchmarkConfig) -> dict[str, str]:
    mock = config.backend == "mock"
    return {
        "control_people": (
            "mock_goal_follower_no_people_input" if mock else "runtime_social_stack_configuration"
        ) if config.control_people_source == "auto" else config.control_people_source,
        "scoring_people": (
            "synthetic_ground_truth" if mock else "mujoco_simulator_truth"
        ) if config.scoring_people_source == "auto" else config.scoring_people_source,
        "scan": (
            "not_used" if mock else "runtime_scan_source_unverified"
        ) if config.scan_source == "auto" else config.scan_source,
    }


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


def _atomic_write_text(path: Path, content: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(content)
    temporary.replace(path)


def _artifact_reference(path: Path) -> dict[str, str]:
    return {
        "path": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
