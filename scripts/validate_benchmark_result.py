#!/usr/bin/env python3
"""Validate safety and quality invariants in a benchmark result."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--minimum-score", type=float, default=95.0)
    parser.add_argument("--backend", choices=("mock", "mujoco"), default="mock")
    parser.add_argument("--people-free", action="store_true")
    return parser.parse_args()


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
    args = parse_args()
    if not args.result.is_file():
        raise SystemExit(f"benchmark result does not exist: {args.result}")

    data = json.loads(args.result.read_text())
    failures: list[str] = []
    require(data.get("backend") == args.backend, f"backend must be {args.backend}", failures)
    require(data.get("valid_trial") is True, "trial must be valid", failures)
    require(data.get("success") is True, "benchmark must report success", failures)
    require(data.get("collisions") == 0, "collisions must be zero", failures)
    require(data.get("fallen") is False, "robot must not fall", failures)
    require(data.get("no_movement") is False, "robot must move", failures)
    require(data.get("left_valid_area") is False, "robot must remain inside benchmark bounds", failures)
    require(data.get("speed_limit_violations") == 0, "speed limit violations must be zero", failures)
    if args.people_free:
        require(data.get("people_count") == 0, "people_count must be zero", failures)
    require(
        data.get("goals_reached") == data.get("goals_requested")
        and isinstance(data.get("goals_requested"), int)
        and data["goals_requested"] > 0,
        "all requested goals must be reached",
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
            isinstance(value, (int, float)) and math.isfinite(value),
            f"{key} must be finite",
            failures,
        )

    score = data.get("final_score")
    if isinstance(score, (int, float)) and math.isfinite(score):
        require(score >= args.minimum_score, f"final_score {score} is below {args.minimum_score}", failures)
        require(score <= 100.0, f"final_score {score} exceeds 100", failures)

    expected_artifacts = [
        args.result.with_name(args.result.stem + "_goals.csv"),
        args.result.with_name(args.result.stem + "_events.csv"),
        args.result.with_name(args.result.stem + "_scenario.yaml"),
    ]
    for artifact in expected_artifacts:
        require(artifact.is_file() and artifact.stat().st_size > 0, f"missing artifact: {artifact}", failures)

    if failures:
        print("Benchmark regression validation failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print(f"Benchmark regression passed: score={score}, goals={data['goals_reached']}, collisions=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
