#!/usr/bin/env python3
"""Validate safety and quality invariants in a benchmark result."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import yaml


SUPPORTED_SCHEMA_VERSION = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--minimum-score", type=float, default=95.0)
    parser.add_argument("--backend", choices=("mock", "mujoco"), default="mock")
    parser.add_argument("--people-free", action="store_true")
    parser.add_argument("--require-avoidance", action="store_true")
    return parser.parse_args()


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def is_integer(value) -> bool:
    return type(value) is int


def is_number(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def main() -> int:
    args = parse_args()
    if not args.result.is_file():
        raise SystemExit(f"benchmark result does not exist: {args.result}")

    data = json.loads(args.result.read_text())
    failures: list[str] = []
    require(data.get("visual_session", False) is False, "visual sessions are not scored benchmark results", failures)
    require(
        type(data.get("schema_version")) is int
        and data["schema_version"] == SUPPORTED_SCHEMA_VERSION,
        f"schema_version must be {SUPPORTED_SCHEMA_VERSION}",
        failures,
    )
    require(data.get("backend") == args.backend, f"backend must be {args.backend}", failures)
    require(data.get("valid_trial") is True, "trial must be valid", failures)
    require(data.get("invalid_reason") is None, "valid trial must not have an invalid_reason", failures)
    require(data.get("success") is True, "benchmark must report success", failures)
    require(is_integer(data.get("collisions")) and data["collisions"] == 0, "collisions must be zero", failures)
    require(data.get("fallen") is False, "robot must not fall", failures)
    require(data.get("no_movement") is False, "robot must move", failures)
    require(data.get("left_valid_area") is False, "robot must remain inside benchmark bounds", failures)
    require(
        is_integer(data.get("speed_limit_violations")) and data["speed_limit_violations"] == 0,
        "speed limit violations must be zero",
        failures,
    )
    if args.backend == "mujoco":
        require(
            is_integer(data.get("people_requested"))
            and is_integer(data.get("people_observed"))
            and data.get("people_observed") == data.get("people_requested"),
            "observed pedestrian count must exactly match the requested count",
            failures,
        )
        sources = data.get("data_sources")
        requested_sources = sources.get("requested") if isinstance(sources, dict) else None
        observed_sources = sources.get("observed") if isinstance(sources, dict) else None
        require(
            all(
                isinstance(group, dict)
                and all(isinstance(group.get(key), str) and group[key] for key in (
                    "control_people", "scoring_people", "scan"
                ))
                for group in (requested_sources, observed_sources)
            ),
            "live result must declare requested and observed control, scoring, and scan sources",
            failures,
        )
        if isinstance(requested_sources, dict) and isinstance(observed_sources, dict):
            require(
                observed_sources.get("scan") in {"mujoco_raycast", "fallback_geometry"},
                "live result must report an observed scan source",
                failures,
            )
            if requested_sources.get("scan") in {"mujoco_raycast", "fallback_geometry"}:
                require(
                    observed_sources.get("scan") == requested_sources.get("scan"),
                    "observed scan source must match the configured scan source",
                    failures,
                )
            if requested_sources.get("control_people") not in {"auto", "not_used"}:
                require(
                    observed_sources.get("control_people") == requested_sources.get("control_people"),
                    "observed control people source must match the configured source",
                    failures,
                )
            if requested_sources.get("scoring_people") not in {"auto", "not_used"}:
                require(
                    observed_sources.get("scoring_people") == requested_sources.get("scoring_people"),
                    "observed scoring people source must match the configured source",
                    failures,
                )
        require(
            is_integer(data.get("people_active_initial"))
            and data.get("people_active_initial") == data.get("people_requested"),
            "all requested pedestrians must be active at scoring start",
            failures,
        )
        require(
            is_integer(data.get("people_count"))
            and data.get("people_count") == data.get("people_requested"),
            "people_count must match people_requested",
            failures,
        )
        active_counts = [
            data.get("people_active_initial"),
            data.get("people_active_minimum"),
            data.get("people_active_final"),
        ]
        require(
            all(is_integer(value) for value in active_counts)
            and 0 <= active_counts[1] <= active_counts[0]
            and 0 <= active_counts[2] <= active_counts[0],
            "active pedestrian counts must be nonnegative and bounded by the initial count",
            failures,
        )
        health = data.get("data_health")
        require(
            isinstance(health, dict) and health.get("require_exact_people_count") is True,
            "live result must enforce exact pedestrian population",
            failures,
        )
        if isinstance(health, dict):
            for age_key, timeout_key in (
                ("max_odom_age_s", "odom_stale_timeout_s"),
                ("max_pedestrian_age_s", "pedestrian_stale_timeout_s"),
                ("max_scan_age_s", "scan_stale_timeout_s"),
                ("max_control_people_source_age_s", "pedestrian_stale_timeout_s"),
                ("max_odom_sample_stall_s", "odom_stale_timeout_s"),
                ("max_pedestrian_sample_stall_s", "pedestrian_stale_timeout_s"),
                ("max_scan_sample_stall_s", "scan_stale_timeout_s"),
            ):
                age = health.get(age_key)
                timeout = health.get(timeout_key)
                require(
                    is_number(age)
                    and is_number(timeout)
                    and 0.0 <= age <= timeout,
                    f"{age_key} must remain within {timeout_key}",
                    failures,
                )
            require(
                is_integer(health.get("scan_messages_observed"))
                and health["scan_messages_observed"] > 0,
                "live result must observe scan messages during scoring",
                failures,
            )
    if args.people_free:
        require(data.get("people_count") == 0, "people_count must be zero", failures)
    if args.require_avoidance:
        require(
            data.get("pedestrian_scenario") in {"demo_endurance", "demo_endurance_plaza"},
            "a deterministic endurance pedestrian scenario is required",
            failures,
        )
        require(data.get("people_count", 0) >= 4, "at least four pedestrians are required", failures)
        require(data.get("goals_requested", 0) >= 4, "at least four goals are required", failures)
        require(data.get("moving_pedestrian_time_s", 0.0) >= 20.0, "pedestrians must move for at least 20 seconds", failures)
        require(data.get("yield_episodes", 0) >= 3, "at least three yield episodes are required", failures)
        require(data.get("resume_episodes", 0) >= 3, "at least three returns to normal after yielding are required", failures)
        require(data.get("yield_time_s", 0.0) >= 1.0, "cumulative yield time must be at least one second", failures)
        require(data.get("moving_avoidance_time_s", 0.0) >= 2.0, "planner yield must overlap moving pedestrians for at least two seconds", failures)
        require(data.get("post_avoidance_progress_m", 0.0) >= 0.75, "robot must make at least 0.75 m progress after avoidance", failures)
        require(data.get("total_time_s", 0.0) >= 30.0, "avoidance trial must last at least 30 seconds", failures)
        require(
            isinstance(data.get("minimum_body_clearance_m"), (int, float))
            and data["minimum_body_clearance_m"] > 0.0,
            "minimum body clearance must remain positive",
            failures,
        )
        require(
            data.get("max_abs_safe_lateral_cmd_mps", math.inf) <= 0.01,
            "safe command must not rely on unsupported lateral motion",
            failures,
        )
    require(
        data.get("goals_reached") == data.get("goals_requested")
        and is_integer(data.get("goals_requested"))
        and is_integer(data.get("goals_reached"))
        and data["goals_requested"] > 0,
        "all requested goals must be reached",
        failures,
    )
    goal_rows = data.get("goals")
    require(
        isinstance(goal_rows, list)
        and len(goal_rows) == data.get("goals_requested")
        and all(isinstance(row, dict) for row in goal_rows),
        "goal records must match goals_requested",
        failures,
    )
    if isinstance(goal_rows, list) and all(isinstance(row, dict) for row in goal_rows):
        require(
            sum(row.get("reached") is True for row in goal_rows) == data.get("goals_reached"),
            "goal records must match goals_reached",
            failures,
        )

    for key in (
        "final_score",
        "safety_score",
        "task_score",
        "speed_score",
        "efficiency_score",
        "total_time_s",
        "actual_path_length_m",
    ):
        value = data.get(key)
        require(
            is_number(value),
            f"{key} must be finite",
            failures,
        )

    score = data.get("final_score")
    if is_number(score):
        require(score >= args.minimum_score, f"final_score {score} is below {args.minimum_score}", failures)
        require(score <= 100.0, f"final_score {score} exceeds 100", failures)
    for key in ("safety_score", "task_score", "speed_score", "efficiency_score"):
        value = data.get(key)
        if is_number(value):
            require(0.0 <= value <= 100.0, f"{key} must be within [0, 100]", failures)

    run_id = data.get("run_id")
    require(isinstance(run_id, str) and bool(run_id), "run_id must be a nonempty string", failures)
    artifacts = data.get("artifacts")
    expected_names = {
        "goals_csv": args.result.stem + "_goals.csv",
        "events_csv": args.result.stem + "_events.csv",
        "scenario": args.result.stem + "_scenario.yaml",
    }
    resolved_artifacts: dict[str, Path] = {}
    require(isinstance(artifacts, dict), "artifacts must declare the result bundle", failures)
    if isinstance(artifacts, dict):
        for key, expected_name in expected_names.items():
            reference = artifacts.get(key)
            valid_reference = (
                isinstance(reference, dict)
                and reference.get("path") == expected_name
                and isinstance(reference.get("sha256"), str)
                and len(reference["sha256"]) == 64
            )
            require(valid_reference, f"invalid artifact reference: {key}", failures)
            if not valid_reference:
                continue
            artifact = args.result.parent / expected_name
            resolved_artifacts[key] = artifact
            require(
                artifact.is_file() and artifact.stat().st_size > 0,
                f"missing artifact: {artifact}",
                failures,
            )
            if artifact.is_file():
                digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
                require(digest == reference["sha256"], f"artifact checksum mismatch: {key}", failures)

    scenario_path = resolved_artifacts.get("scenario")
    if scenario_path is not None and scenario_path.is_file():
        try:
            scenario = yaml.safe_load(scenario_path.read_text())
        except yaml.YAMLError:
            scenario = None
        require(isinstance(scenario, dict), "scenario artifact must contain a mapping", failures)
        if isinstance(scenario, dict):
            require(scenario.get("schema_version") == SUPPORTED_SCHEMA_VERSION, "scenario schema_version must match result", failures)
            require(scenario.get("run_id") == run_id, "scenario run_id must match result", failures)
            for key in ("seed", "mode", "backend", "pedestrian_scenario"):
                require(scenario.get(key) == data.get(key), f"scenario {key} must match result", failures)
            require(
                isinstance(scenario.get("goals"), list)
                and len(scenario["goals"]) == data.get("goals_requested"),
                "scenario goals must match goals_requested",
                failures,
            )
            require(
                isinstance(scenario.get("people"), list)
                and len(scenario["people"]) == data.get("people_requested"),
                "scenario people must match people_requested",
                failures,
            )
            require(
                Path(str(data.get("scenario_file", ""))).name == scenario_path.name,
                "scenario_file must identify the bundled scenario",
                failures,
            )

    goals_path = resolved_artifacts.get("goals_csv")
    if goals_path is not None and goals_path.is_file():
        try:
            with goals_path.open(newline="") as handle:
                csv_goal_rows = list(csv.DictReader(handle))
        except (csv.Error, UnicodeError):
            csv_goal_rows = []
        rows_match = len(csv_goal_rows) == len(goal_rows) if isinstance(goal_rows, list) else False
        if rows_match:
            rows_match = all(
                str(csv_row.get("index")) == str(result_row.get("index"))
                and str(csv_row.get("reached", "")).lower()
                == str(result_row.get("reached", "")).lower()
                for csv_row, result_row in zip(csv_goal_rows, goal_rows)
            )
        require(rows_match, "goals CSV rows must match result goals", failures)

    events_path = resolved_artifacts.get("events_csv")
    result_events = data.get("events")
    require(isinstance(result_events, list), "events must be a list", failures)
    if events_path is not None and events_path.is_file():
        try:
            with events_path.open(newline="") as handle:
                csv_event_rows = list(csv.DictReader(handle))
        except (csv.Error, UnicodeError):
            csv_event_rows = []
        require(
            isinstance(result_events, list) and len(csv_event_rows) == len(result_events),
            "events CSV rows must match result events",
            failures,
        )

    if failures:
        print("Benchmark regression validation failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print(f"Benchmark regression passed: score={score}, goals={data['goals_reached']}, collisions=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
