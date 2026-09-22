#!/usr/bin/env python3
"""Extract a schema-validated G1 response trial from timestamped calibration CSV."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = REPO_ROOT / "docs" / "schemas" / "g1_response_calibration.schema.json"
REQUIRED_COLUMNS = {
    "trial_id", "source_time_s", "arrival_monotonic_s", "phase",
    "command_linear_x_mps", "command_linear_y_mps", "command_angular_z_radps",
    "policy_linear_x_mps", "policy_linear_y_mps", "policy_angular_z_radps",
    "policy_command_start_source_time_s", "policy_stop_source_time_s",
    "x_m", "y_m", "z_m", "yaw_rad",
    "reported_vx_mps", "reported_vy_mps",
    "reported_wz_radps", "derived_vx_mps", "derived_vy_mps",
    "derived_wz_radps", "max_tilt_rad", "fallen",
}


@dataclass(frozen=True)
class Sample:
    time_s: float
    phase: str
    command: tuple[float, float, float]
    policy_command: tuple[float, float, float]
    pose: tuple[float, float, float]
    base_height_m: float | None
    velocity: tuple[float, float, float]
    max_tilt_rad: float
    fallen: bool
    command_observed_time_s: float
    policy_stop_time_s: float


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _float(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def read_samples(path: Path, max_gap_s: float) -> tuple[list[Sample], float, int, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = REQUIRED_COLUMNS.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"sample CSV is missing columns: {sorted(missing)}")
        if "base_height_m" not in (reader.fieldnames or []) and "z_m" not in (reader.fieldnames or []):
            raise ValueError("sample CSV is missing a base-height signal")
        rows = list(reader)
    if not rows:
        raise ValueError("sample CSV is empty")
    raw_times = [_float(row, "source_time_s") for row in rows]
    if any(value <= 0.0 for value in raw_times):
        raise ValueError("source timestamps must be positive")
    origin = raw_times[0]
    times = [value - origin for value in raw_times]
    gaps = [right - left for left, right in zip(times, times[1:])]
    if any(gap <= 0.0 for gap in gaps):
        raise ValueError("source timestamps must be strictly increasing")
    if gaps and max(gaps) > max_gap_s + 1e-9:
        raise ValueError(f"source sample gap exceeds {max_gap_s} s")
    arrival_times = [_float(row, "arrival_monotonic_s") for row in rows]
    arrival_gaps = [right - left for left, right in zip(arrival_times, arrival_times[1:])]
    if any(gap <= 0.0 for gap in arrival_gaps):
        raise ValueError("arrival timestamps must be strictly increasing")
    median_gap = statistics.median(gaps) if gaps else math.inf
    sample_rate = 1.0 / median_gap if math.isfinite(median_gap) else 0.0
    dropped = sum(max(0, round(gap / median_gap) - 1) for gap in gaps) if gaps else 0
    trial_ids = {row["trial_id"] for row in rows}
    if len(trial_ids) != 1:
        raise ValueError("sample CSV contains multiple trial IDs")
    samples = [
        Sample(
            time_s=time_s,
            phase=row["phase"],
            command=(
                _float(row, "command_linear_x_mps"),
                _float(row, "command_linear_y_mps"),
                _float(row, "command_angular_z_radps"),
            ),
            policy_command=(
                _float(row, "policy_linear_x_mps"),
                _float(row, "policy_linear_y_mps"),
                _float(row, "policy_angular_z_radps"),
            ),
            pose=(_float(row, "x_m"), _float(row, "y_m"), _float(row, "yaw_rad")),
            base_height_m=(
                value if (
                    (raw_height := row.get("base_height_m") or row.get("z_m"))
                    and (value := float(raw_height)) > 0.1
                    and math.isfinite(value)
                ) else None
            ),
            velocity=(
                _float(row, "derived_vx_mps"),
                _float(row, "derived_vy_mps"),
                _float(row, "derived_wz_radps"),
            ),
            max_tilt_rad=_float(row, "max_tilt_rad"),
            fallen=row["fallen"].strip().lower() in {"1", "true", "yes"},
            command_observed_time_s=(
                _float(row, "policy_command_start_source_time_s") - origin
            ),
            policy_stop_time_s=_float(row, "policy_stop_source_time_s") - origin,
        )
        for row, time_s in zip(rows, times)
    ]
    return samples, sample_rate, dropped, next(iter(trial_ids))


def selected_axis(command: tuple[float, float, float]) -> tuple[int, str, str, float]:
    active = [index for index, value in enumerate(command) if abs(value) > 1e-9]
    if len(active) != 1:
        raise ValueError("response extraction currently requires exactly one nonzero command axis")
    index = active[0]
    return index, ("linear_x", "linear_y", "angular_z")[index], ("m/s", "m/s", "rad/s")[index], command[index]


def trailing_mean(values: list[tuple[float, float]], window_s: float) -> list[tuple[float, float]]:
    filtered = []
    start = 0
    total = 0.0
    for index, (time_s, value) in enumerate(values):
        total += value
        while values[start][0] < time_s - window_s - 1e-12:
            total -= values[start][1]
            start += 1
        filtered.append((time_s, total / (index - start + 1)))
    return filtered


def pose_derived_velocity(samples: list[Sample], window_s: float) -> list[tuple[float, float, float]]:
    velocities = []
    left = 0
    for index, sample in enumerate(samples):
        while left + 1 < index and samples[left + 1].time_s <= sample.time_s - window_s:
            left += 1
        reference = samples[left]
        dt = sample.time_s - reference.time_s
        if dt <= 1e-9:
            velocities.append((0.0, 0.0, 0.0))
            continue
        dx = sample.pose[0] - reference.pose[0]
        dy = sample.pose[1] - reference.pose[1]
        velocities.append((
            (math.cos(sample.pose[2]) * dx + math.sin(sample.pose[2]) * dy) / dt,
            (-math.sin(sample.pose[2]) * dx + math.cos(sample.pose[2]) * dy) / dt,
            angle_delta(sample.pose[2], reference.pose[2]) / dt,
        ))
    return velocities


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * quantile)]


def sustained_time(values: list[tuple[float, float]], threshold: float, hold_s: float, above: bool) -> float | None:
    tolerance = 1e-12
    for index, (start, value) in enumerate(values):
        qualifies = value + tolerance >= threshold if above else value <= threshold + tolerance
        if not qualifies:
            continue
        end = start + hold_s
        end_index = next(
            (candidate for candidate in range(index, len(values)) if values[candidate][0] + 1e-9 >= end),
            None,
        )
        if end_index is None:
            continue
        window = values[index:end_index + 1]
        if all(
            (item + tolerance >= threshold if above else item <= threshold + tolerance)
            for _, item in window
        ):
            return start
    return None


def angle_delta(target: float, source: float) -> float:
    return math.atan2(math.sin(target - source), math.cos(target - source))


def pose_at(samples: list[Sample], time_s: float) -> tuple[float, float, float] | None:
    if not samples or time_s < samples[0].time_s or time_s > samples[-1].time_s:
        return None
    for left, right in zip(samples, samples[1:]):
        if left.time_s <= time_s <= right.time_s:
            span = right.time_s - left.time_s
            fraction = 0.0 if span <= 0.0 else (time_s - left.time_s) / span
            return (
                left.pose[0] + fraction * (right.pose[0] - left.pose[0]),
                left.pose[1] + fraction * (right.pose[1] - left.pose[1]),
                left.pose[2] + fraction * angle_delta(right.pose[2], left.pose[2]),
            )
    return samples[-1].pose


def planar_distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def extract_metrics(
    samples: list[Sample], onset_hold_s: float = 0.2, stop_hold_s: float = 0.5
) -> tuple[dict[str, Any], tuple[float, float, float]]:
    command_samples = [sample for sample in samples if sample.phase == "command"]
    stop_phase_samples = [sample for sample in samples if sample.phase in {"stop", "cleanup_stop"}]
    stop_samples = [
        sample for sample in samples
        if sample.phase in {"stop", "cleanup_stop"}
        and all(abs(value) <= 1e-9 for value in sample.command)
        and all(abs(value) <= 1e-9 for value in sample.policy_command)
        and sample.policy_stop_time_s > 0.0
    ]
    settle_samples = [sample for sample in samples if sample.phase == "settle"]
    if not command_samples or not stop_samples or not settle_samples:
        raise ValueError("samples must contain settle, command, and stop phases")
    command = command_samples[0].command
    if any(sample.command != command for sample in command_samples):
        raise ValueError("command phase contains inconsistent requested commands")
    if any(abs(value) > 1e-9 for sample in settle_samples for value in sample.command):
        raise ValueError("settle phase contains a nonzero requested command")
    if not (
        max(sample.time_s for sample in settle_samples)
        < min(sample.time_s for sample in command_samples)
        < min(sample.time_s for sample in stop_phase_samples)
    ):
        raise ValueError("sample phases are missing or out of order")
    axis_index, axis_name, unit, target = selected_axis(command)
    sign = -1.0 if target < 0.0 else 1.0
    pose_velocity = pose_derived_velocity(samples, 0.10)
    settle_velocity = [
        velocity for sample, velocity in zip(samples, pose_velocity)
        if sample.phase == "settle"
    ]
    if any(math.hypot(vx, vy) > 0.03 or abs(wz) > 0.03 for vx, vy, wz in settle_velocity):
        raise ValueError("settle phase violates the stationary velocity gate")
    if any(sample.max_tilt_rad > 0.35 for sample in settle_samples):
        raise ValueError("settle phase violates the tilt gate")
    filtered_velocity = trailing_mean(
        [
            (sample.time_s, sign * velocity[axis_index])
            for sample, velocity in zip(samples, pose_velocity)
        ],
        0.10,
    )
    stability_velocity = trailing_mean(
        [
            (sample.time_s, sign * velocity[axis_index])
            for sample, velocity in zip(samples, pose_velocity)
        ],
        0.80,
    )
    filtered_by_time = {time_s: value for time_s, value in filtered_velocity}
    baseline_values = [filtered_by_time[sample.time_s] for sample in settle_samples]
    baseline = statistics.median(baseline_values)
    command_start = next(
        (
            sample.command_observed_time_s for sample in command_samples
            if sample.command_observed_time_s > 0.0
        ),
        None,
    )
    if command_start is None:
        raise ValueError("command phase has no observed nonzero policy transition")
    stop_start = stop_samples[0].policy_stop_time_s
    if not command_samples[0].time_s - 0.2 <= command_start <= command_samples[-1].time_s:
        raise ValueError("policy command start timestamp is outside the command phase")
    if not stop_samples[0].time_s - 0.2 <= stop_start <= stop_samples[-1].time_s:
        raise ValueError("policy stop timestamp is outside the stop phase")
    final_policy_commands = [
        sample.policy_command[axis_index]
        for sample in command_samples
        if sample.time_s >= command_samples[-1].time_s - 2.0
    ]
    if not final_policy_commands or any(
        abs(value - target) > 1e-6 for value in final_policy_commands
    ):
        raise ValueError("policy input command did not settle at the requested target")
    response = [
        (sample.time_s - command_start, filtered_by_time[sample.time_s] - baseline)
        for sample in command_samples
        if sample.time_s >= command_start
    ]
    if not response:
        raise ValueError("no command response samples follow the policy command timestamp")
    steady_window = [value for time_s, value in response if time_s >= response[-1][0] - 2.0]
    steady = statistics.median(steady_window) if steady_window else None
    steady_span = response[-1][0] - max(response[-1][0] - 2.0, response[0][0])
    stability_response = [
        (time_s - command_start, value - baseline)
        for time_s, value in stability_velocity
        if command_start <= time_s <= command_samples[-1].time_s
    ]
    stability_window = [
        value for time_s, value in stability_response
        if time_s >= stability_response[-1][0] - 2.0
    ]
    steady_range = (
        percentile(stability_window, 0.90) - percentile(stability_window, 0.10)
        if stability_window else None
    )
    stable_steady = (
        steady_span >= 1.9
        and steady_range is not None
        and steady_range <= max(0.03, 0.20 * abs(target))
    )
    onset_threshold = 0.03
    latency = sustained_time(response, onset_threshold, onset_hold_s, True)
    no_motion = latency is None
    rise_10 = None if steady is None or steady <= 0.0 else sustained_time(response, 0.1 * steady, 0.0, True)
    rise_90 = None if steady is None or steady <= 0.0 else sustained_time(response, 0.9 * steady, onset_hold_s, True)

    stop_response = [
        (sample.time_s - stop_start, abs(filtered_by_time[sample.time_s] - baseline))
        for sample in stop_samples
        if sample.time_s >= stop_start
    ]
    if not stop_response:
        raise ValueError("no stop response samples follow the policy stop timestamp")
    stop_time = sustained_time(stop_response, 0.03, stop_hold_s, False)
    effective_acceleration = (
        0.8 * steady / (rise_90 - rise_10)
        if steady is not None and rise_10 is not None and rise_90 is not None
        and rise_90 > rise_10 else None
    )
    effective_deceleration = (
        steady / stop_time
        if steady is not None and stop_time is not None and stop_time > 0.0 else None
    )
    stop_pose = pose_at(samples, stop_start)
    settled_pose = pose_at(samples, stop_start + stop_time) if stop_time is not None else None
    longitudinal = planar = stopping_angle = None
    if stop_pose is not None and settled_pose is not None:
        dx = settled_pose[0] - stop_pose[0]
        dy = settled_pose[1] - stop_pose[1]
        longitudinal = math.cos(stop_pose[2]) * dx + math.sin(stop_pose[2]) * dy
        planar = math.hypot(dx, dy)
        stopping_angle = abs(angle_delta(settled_pose[2], stop_pose[2]))
    command_pose = pose_at(samples, command_start)
    command_lateral_drift = command_heading_drift = None
    if axis_index < 2 and command_pose is not None and stop_pose is not None:
        command_dx = stop_pose[0] - command_pose[0]
        command_dy = stop_pose[1] - command_pose[1]
        command_lateral_drift = abs(
            -math.sin(command_pose[2]) * command_dx
            + math.cos(command_pose[2]) * command_dy
        )
        command_heading_drift = abs(angle_delta(stop_pose[2], command_pose[2]))

    drift: dict[float, float | None] = {}
    yaw_drift: dict[float, float | None] = {}
    drift_origin = settled_pose
    for horizon in (1.0, 3.0, 5.0):
        horizon_pose = (
            pose_at(samples, stop_start + stop_time + horizon)
            if stop_time is not None
            else None
        )
        drift[horizon] = planar_distance(drift_origin, horizon_pose) if drift_origin and horizon_pose else None
        yaw_drift[horizon] = (
            abs(angle_delta(horizon_pose[2], drift_origin[2]))
            if drift_origin and horizon_pose else None
        )

    first_fall = next((sample.time_s for sample in samples if sample.fallen), None)
    metrics = {
        "no_motion": no_motion,
        "velocity_filter": {"type": "trailing_mean", "window_s": 0.10},
        "stability_filter": {"type": "trailing_mean", "window_s": 0.80},
        "velocity_source": "pose_derived_body_twist",
        "stable_steady_interval": stable_steady,
        "steady_response_p90_p10": steady_range,
        "response_axis": axis_name,
        "response_unit": unit,
        "acceleration_unit": "m/s^2" if axis_index < 2 else "rad/s^2",
        "latency_s": latency,
        "rise_time_10_s": rise_10,
        "rise_time_90_s": rise_90,
        "steady_response": steady,
        "steady_tracking_error": abs(abs(target) - steady) if steady is not None else None,
        "steady_relative_tracking_error": (
            abs(abs(target) - steady) / abs(target) if steady is not None else None
        ),
        "effective_acceleration": effective_acceleration,
        "effective_deceleration": effective_deceleration,
        "steady_velocity_mps": steady if axis_index < 2 else None,
        "steady_tracking_error_mps": (
            abs(abs(target) - steady) if steady is not None and axis_index < 2 else None
        ),
        "stop_time_s": stop_time,
        "stopping_distance_longitudinal_m": longitudinal,
        "stopping_distance_planar_m": planar,
        "command_lateral_drift_m": command_lateral_drift,
        "command_heading_drift_rad": command_heading_drift,
        "post_stop_drift_1s_m": drift[1.0],
        "post_stop_drift_3s_m": drift[3.0],
        "post_stop_drift_5s_m": drift[5.0],
        "available_post_stop_drift_horizons_s": [
            horizon for horizon in (1.0, 3.0, 5.0) if drift[horizon] is not None
        ],
        "stopping_angle_rad": stopping_angle,
        "post_stop_yaw_drift_1s_rad": yaw_drift[1.0],
        "post_stop_yaw_drift_3s_rad": yaw_drift[3.0],
        "post_stop_yaw_drift_5s_rad": yaw_drift[5.0],
        "max_tilt_rad": max(sample.max_tilt_rad for sample in samples),
        "minimum_base_height_m": min(
            (sample.base_height_m for sample in samples if sample.base_height_m is not None),
            default=None,
        ),
        "fallen": any(sample.fallen for sample in samples),
        "first_fall_time_s": first_fall,
    }
    return metrics, command


def assemble_dataset(samples_path: Path, metadata: dict[str, Any], max_gap_s: float) -> dict[str, Any]:
    samples, sample_rate, dropped, csv_trial_id = read_samples(samples_path, max_gap_s)
    trial_metadata = metadata["trial"]
    if trial_metadata["command_family"] not in {"forward_step_stop", "yaw_step_stop"}:
        raise ValueError("offline extraction currently supports only step-stop trials")
    metrics, command = extract_metrics(samples)
    if trial_metadata["trial_id"] != csv_trial_id:
        raise ValueError("metadata trial ID does not match the sample CSV")
    trial = {
        "trial_id": trial_metadata["trial_id"],
        "split": trial_metadata["split"],
        "seed": trial_metadata["seed"],
        "repetition": trial_metadata["repetition"],
        "command_family": trial_metadata["command_family"],
        "command": {
            "linear_x_mps": command[0],
            "linear_y_mps": command[1],
            "angular_z_radps": command[2],
        },
        "initial_velocity_mps": trial_metadata.get("initial_velocity_mps", 0.0),
        "valid_trial": True,
        "invalid_reason": None,
        "sample_rate_hz": sample_rate,
        "dropped_samples": dropped,
        "metrics": metrics,
        "artifacts": [{
            "path": str(samples_path),
            "sha256": sha256_file(samples_path),
            "size_bytes": samples_path.stat().st_size,
        }],
    }
    return {
        "schema_version": 1,
        "dataset_id": metadata["dataset_id"],
        "platform": metadata["platform"],
        "collected_at": metadata["collected_at"],
        "protocol_version": "g1_response_v1",
        "description": metadata.get("description", ""),
        "provenance": metadata["provenance"],
        "trials": [trial],
    }


def validate_dataset(dataset: dict[str, Any], schema_path: Path) -> None:
    schema = json.loads(schema_path.read_text())
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    validator = validator_class(schema, format_checker=jsonschema.FormatChecker())
    errors = sorted(validator.iter_errors(dataset), key=lambda error: list(error.path))
    if errors:
        details = "; ".join(f"{'.'.join(map(str, error.path))}: {error.message}" for error in errors)
        raise ValueError(f"response dataset failed schema validation: {details}")
    for trial in dataset["trials"]:
        metrics = trial["metrics"]
        available = set(metrics["available_post_stop_drift_horizons_s"])
        for horizon in (1, 3, 5):
            present = metrics[f"post_stop_drift_{horizon}s_m"] is not None
            yaw_present = metrics[f"post_stop_yaw_drift_{horizon}s_rad"] is not None
            if (horizon in available) != present or present != yaw_present:
                raise ValueError(
                    f"trial {trial['trial_id']} has inconsistent {horizon}s drift availability"
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", type=Path)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--max-gap", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        metadata = json.loads(args.metadata.read_text())
        dataset = assemble_dataset(args.samples, metadata, args.max_gap)
        validate_dataset(dataset, args.schema)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(dataset, indent=2, sort_keys=True, allow_nan=False) + "\n")
    except (OSError, KeyError, ValueError, json.JSONDecodeError, jsonschema.SchemaError) as error:
        print(f"Response extraction failed: {error}")
        return 2
    print(f"Response dataset: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
