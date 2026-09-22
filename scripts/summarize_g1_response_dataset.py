#!/usr/bin/env python3
"""Summarize repeated G1 response trials by requested command."""

from __future__ import annotations

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

from assemble_g1_response_dataset import DEFAULT_SCHEMA, validate_dataset


SUMMARY_METRICS = (
    "latency_s",
    "steady_response",
    "steady_relative_tracking_error",
    "steady_response_p90_p10",
    "effective_acceleration",
    "effective_deceleration",
    "stop_time_s",
    "stopping_distance_planar_m",
    "post_stop_drift_5s_m",
    "post_stop_yaw_drift_5s_rad",
    "max_tilt_rad",
    "minimum_base_height_m",
)


def command_key(trial: dict[str, Any]) -> tuple[float, float, float]:
    command = trial["command"]
    return (
        command["linear_x_mps"],
        command["linear_y_mps"],
        command["angular_z_radps"],
    )


def distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "median": statistics.median(values),
        "minimum": min(values),
        "maximum": max(values),
        "range": max(values) - min(values),
    }


def summarize(dataset: dict[str, Any]) -> dict[str, Any]:
    groups: dict[tuple[float, float, float], list[dict[str, Any]]] = defaultdict(list)
    for trial in dataset["trials"]:
        groups[command_key(trial)].append(trial)
    summaries = []
    for command, trials in sorted(groups.items()):
        metrics = {}
        for name in SUMMARY_METRICS:
            values = [
                trial["metrics"][name]
                for trial in trials
                if trial["metrics"].get(name) is not None
            ]
            metrics[name] = distribution(values)
        summaries.append({
            "command": {
                "linear_x_mps": command[0],
                "linear_y_mps": command[1],
                "angular_z_radps": command[2],
            },
            "trial_count": len(trials),
            "valid_trial_count": sum(trial["valid_trial"] for trial in trials),
            "fall_count": sum(trial["metrics"]["fallen"] for trial in trials),
            "no_motion_count": sum(trial["metrics"]["no_motion"] for trial in trials),
            "stable_steady_count": sum(
                trial["metrics"]["stable_steady_interval"] for trial in trials
            ),
            "metrics": metrics,
        })
    return {
        "schema_version": 1,
        "dataset_id": dataset["dataset_id"],
        "trial_count": len(dataset["trials"]),
        "groups": summaries,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        dataset = json.loads(args.dataset.read_text())
        schema = json.loads(args.schema.read_text())
        validate_dataset(dataset, schema)
        summary = summarize(dataset)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Response summary failed: {error}")
        return 2
    print(f"Response repeatability summary: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
