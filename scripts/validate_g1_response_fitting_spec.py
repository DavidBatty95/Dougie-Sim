#!/usr/bin/env python3
"""Validate the frozen G1 response-envelope fitting and collection specs."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FIT_SPEC = REPO_ROOT / (
    "ros2_ws/src/dynamic_obstacle_benchmarks/config/"
    "g1_response_envelope_fit_v1.yaml"
)
DEFAULT_COMMAND_SPEC = REPO_ROOT / (
    "ros2_ws/src/dynamic_obstacle_benchmarks/config/"
    "g1_response_command_order_v1.yaml"
)

KNOWN_METRICS = {
    "latency_s",
    "steady_response",
    "effective_acceleration",
    "effective_deceleration",
    "stop_time_s",
    "stopping_distance_planar_m",
    "command_lateral_drift_m",
    "command_heading_drift_rad",
    "post_stop_drift_5s_m",
    "post_stop_yaw_drift_5s_rad",
    "stopping_angle_rad",
    "max_tilt_rad",
    "minimum_base_height_m",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate(fit: dict[str, Any], commands: dict[str, Any]) -> dict[str, Any]:
    require(fit.get("schema_version") == 1, "unsupported fitting schema version")
    require(commands.get("schema_version") == 1, "unsupported command schema version")
    require(
        fit.get("response_protocol_version") == commands.get("protocol_version"),
        "response protocol versions do not match",
    )

    calibration = fit["splits"]["calibration_seeds"]
    held_out = fit["splits"]["held_out_seeds"]
    require(len(calibration) == len(set(calibration)), "duplicate calibration seed")
    require(len(held_out) == len(set(held_out)), "duplicate held-out seed")
    require(set(calibration).isdisjoint(held_out), "response splits overlap")
    require(
        calibration == commands["splits"]["calibration"]["seeds"],
        "calibration seeds differ from command specification",
    )
    require(
        held_out == commands["splits"]["held_out"]["seeds"],
        "held-out seeds differ from command specification",
    )

    command_ids = {command["id"] for command in commands["commands"]}
    require(len(command_ids) == len(commands["commands"]), "duplicate command ID")
    orders = commands["execution_order_by_seed"]
    require(
        set(orders) == set(calibration) | set(held_out),
        "execution-order seeds do not exactly cover both splits",
    )
    for seed, order in orders.items():
        require(len(order) == len(command_ids), f"seed {seed} order length is wrong")
        require(set(order) == command_ids, f"seed {seed} order does not cover commands")

    family_counts: dict[str, int] = {}
    for command in commands["commands"]:
        family = command["family"]
        family_counts[family] = family_counts.get(family, 0) + 1
    metric_families = fit["metrics"]
    require(
        set(metric_families) == set(family_counts),
        "fitting metric families do not match command families",
    )
    for family, directions in metric_families.items():
        for direction in ("upper", "lower"):
            metrics = directions[direction]
            require(metrics, f"{family} has no {direction} metrics")
            require(
                len(metrics) == len(set(metrics)),
                f"{family} has duplicate {direction} metrics",
            )
            unknown = set(metrics) - KNOWN_METRICS
            require(not unknown, f"{family} has unknown metrics: {sorted(unknown)}")

    coverage = fit["coverage"]
    target = float(coverage["target"])
    confidence = float(coverage["confidence_level"])
    require(0.0 < target < 1.0, "coverage target must be between zero and one")
    require(0.0 < confidence < 1.0, "confidence level must be between zero and one")
    require(
        coverage["interval"] == "one_sided_clopper_pearson",
        "unsupported confidence interval",
    )
    minimum = int(coverage["minimum_evaluable_trials_per_family"])
    theoretical_minimum = math.ceil(math.log(1.0 - confidence) / math.log(target))
    require(
        minimum >= theoretical_minimum,
        "minimum held-out sample cannot meet target at the declared confidence",
    )
    planned_counts = {
        family: count * len(held_out) for family, count in family_counts.items()
    }
    require(
        all(count >= minimum for count in planned_counts.values()),
        "planned held-out trials are below the declared minimum",
    )
    declared_planned = int(coverage["planned_trials_per_family"])
    require(
        all(count == declared_planned for count in planned_counts.values()),
        "declared planned trials do not match commands and seeds",
    )
    all_success_lower_bound = (1.0 - confidence) ** (1.0 / minimum)
    require(
        all_success_lower_bound >= target,
        "all-success exact lower bound does not reach coverage target",
    )
    return {
        "calibration_seed_count": len(calibration),
        "held_out_seed_count": len(held_out),
        "minimum_evaluable_trials_per_family": minimum,
        "all_success_lower_bound_at_minimum": all_success_lower_bound,
        "planned_trials_by_family": planned_counts,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit-spec", type=Path, default=DEFAULT_FIT_SPEC)
    parser.add_argument("--command-spec", type=Path, default=DEFAULT_COMMAND_SPEC)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        fit = yaml.safe_load(args.fit_spec.read_text())
        commands = yaml.safe_load(args.command_spec.read_text())
        result = validate(fit, commands)
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"Response fitting specification is invalid: {error}")
        return 2
    print("Response fitting specification is valid")
    for key, value in result.items():
        print(f"  {key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
