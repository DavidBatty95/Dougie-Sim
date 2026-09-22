from __future__ import annotations

import importlib.util
import csv
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml


SCRIPT = Path(__file__).with_name("extract_g1_response_metrics.py")
SPEC = importlib.util.spec_from_file_location("extract_g1_response_metrics", SCRIPT)
assert SPEC and SPEC.loader
extraction = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = extraction
SPEC.loader.exec_module(extraction)

ASSEMBLER_SCRIPT = Path(__file__).with_name("assemble_g1_response_dataset.py")
ASSEMBLER_SPEC = importlib.util.spec_from_file_location("assemble_g1_response_dataset", ASSEMBLER_SCRIPT)
assert ASSEMBLER_SPEC and ASSEMBLER_SPEC.loader
assembler = importlib.util.module_from_spec(ASSEMBLER_SPEC)
sys.modules[ASSEMBLER_SPEC.name] = assembler
ASSEMBLER_SPEC.loader.exec_module(assembler)

SUMMARY_SCRIPT = Path(__file__).with_name("summarize_g1_response_dataset.py")
SUMMARY_SPEC = importlib.util.spec_from_file_location("summarize_g1_response_dataset", SUMMARY_SCRIPT)
assert SUMMARY_SPEC and SUMMARY_SPEC.loader
summary = importlib.util.module_from_spec(SUMMARY_SPEC)
sys.modules[SUMMARY_SPEC.name] = summary
SUMMARY_SPEC.loader.exec_module(summary)


def synthetic_samples(axis: int, target: float, end_s: float = 17.0):
    samples = []
    x = y = yaw = 0.0
    previous_velocity = (0.0, 0.0, 0.0)
    steps = round(end_s * 10)
    for step in range(steps + 1):
        time_s = step / 10.0
        if time_s < 3.0:
            phase = "settle"
            command = (0.0, 0.0, 0.0)
            response = 0.0
        elif time_s < 11.0:
            phase = "command"
            command_values = [0.0, 0.0, 0.0]
            command_values[axis] = target
            command = tuple(command_values)
            response = target * min((time_s - 3.0) / 1.0, 1.0)
        else:
            phase = "stop"
            command = (0.0, 0.0, 0.0)
            response = target * max(1.0 - (time_s - 11.0) / 0.2, 0.0)
        if step:
            x += previous_velocity[0] * 0.1
            y += previous_velocity[1] * 0.1
            yaw += previous_velocity[2] * 0.1
        velocity_values = [0.0, 0.0, 0.0]
        velocity_values[axis] = response
        previous_velocity = tuple(velocity_values)
        samples.append(extraction.Sample(
            time_s=time_s,
            phase=phase,
            command=command,
            policy_command=command,
            pose=(x, y, yaw),
            base_height_m=0.79,
            velocity=previous_velocity,
            max_tilt_rad=0.04,
            fallen=False,
            command_observed_time_s=3.0 if phase == "command" else (11.0 if phase == "stop" else 0.0),
            policy_stop_time_s=11.0 if phase == "stop" else 0.0,
        ))
    return samples


def test_extracts_linear_response_and_all_drift_horizons() -> None:
    metrics, command = extraction.extract_metrics(synthetic_samples(0, 0.2))

    assert command == (0.2, 0.0, 0.0)
    assert metrics["response_axis"] == "linear_x"
    assert metrics["response_unit"] == "m/s"
    assert metrics["latency_s"] == pytest.approx(0.4)
    assert metrics["rise_time_90_s"] == pytest.approx(1.1)
    assert metrics["steady_response"] == pytest.approx(0.2)
    assert metrics["stable_steady_interval"] is True
    assert metrics["steady_relative_tracking_error"] == pytest.approx(0.0)
    assert metrics["effective_acceleration"] > 0.0
    assert metrics["effective_deceleration"] > 0.0
    assert metrics["stop_time_s"] == pytest.approx(0.4)
    assert metrics["minimum_base_height_m"] == pytest.approx(0.79)
    assert metrics["available_post_stop_drift_horizons_s"] == [1.0, 3.0, 5.0]


def test_gait_cycle_filter_accepts_periodic_steady_response() -> None:
    samples = synthetic_samples(0, 0.2)
    x = 0.0
    previous_velocity = 0.0
    oscillating = []
    for sample in samples:
        if oscillating:
            x += previous_velocity * 0.1
        previous_velocity = sample.velocity[0]
        if sample.phase == "command" and sample.time_s >= 4.0:
            previous_velocity += 0.08 * math.sin(
                2.0 * math.pi * (sample.time_s - 4.0) / 0.8
            )
        oscillating.append(replace(sample, pose=(x, 0.0, 0.0)))

    metrics, _ = extraction.extract_metrics(oscillating)

    assert metrics["stable_steady_interval"] is True
    assert metrics["steady_response_p90_p10"] < 0.05


def test_negative_yaw_uses_commanded_direction_and_explicit_units() -> None:
    metrics, _ = extraction.extract_metrics(synthetic_samples(2, -0.2))

    assert metrics["response_axis"] == "angular_z"
    assert metrics["response_unit"] == "rad/s"
    assert metrics["steady_response"] == pytest.approx(0.2)
    assert metrics["steady_velocity_mps"] is None
    assert metrics["stopping_angle_rad"] > 0.0


def test_unrecorded_drift_horizons_are_null_and_marked_unavailable() -> None:
    metrics, _ = extraction.extract_metrics(synthetic_samples(0, 0.2, end_s=13.0))

    assert metrics["post_stop_drift_1s_m"] is not None
    assert metrics["post_stop_drift_3s_m"] is None
    assert metrics["post_stop_drift_5s_m"] is None
    assert metrics["available_post_stop_drift_horizons_s"] == [1.0]


def test_rejects_multiaxis_commands() -> None:
    with pytest.raises(ValueError, match="exactly one"):
        extraction.selected_axis((0.1, 0.0, 0.1))


def test_rejects_inconsistent_or_clipped_policy_commands() -> None:
    samples = synthetic_samples(0, 0.2)
    command_index = next(index for index, sample in enumerate(samples) if sample.phase == "command")
    inconsistent = list(samples)
    inconsistent[command_index] = replace(
        inconsistent[command_index], command=(0.1, 0.0, 0.0)
    )
    with pytest.raises(ValueError, match="inconsistent requested"):
        extraction.extract_metrics(inconsistent)

    clipped = [
        replace(sample, policy_command=(0.15, 0.0, 0.0))
        if sample.phase == "command" else sample
        for sample in samples
    ]
    with pytest.raises(ValueError, match="did not settle"):
        extraction.extract_metrics(clipped)


def write_sample_csv(path: Path, trial_id: str, samples) -> None:
    fieldnames = [
        "trial_id", "source_time_s", "arrival_monotonic_s", "phase",
        "command_linear_x_mps", "command_linear_y_mps", "command_angular_z_radps",
        "policy_linear_x_mps", "policy_linear_y_mps", "policy_angular_z_radps",
        "policy_command_start_source_time_s", "policy_stop_source_time_s",
        "x_m", "y_m", "z_m", "base_height_m", "yaw_rad",
        "reported_vx_mps", "reported_vy_mps",
        "reported_wz_radps", "max_tilt_rad", "fallen",
        "derived_vx_mps", "derived_vy_mps", "derived_wz_radps",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow({
                "trial_id": trial_id,
                "source_time_s": sample.time_s + 1.0,
                "arrival_monotonic_s": sample.time_s + 10.0,
                "phase": sample.phase,
                "command_linear_x_mps": sample.command[0],
                "command_linear_y_mps": sample.command[1],
                "command_angular_z_radps": sample.command[2],
                "policy_linear_x_mps": sample.policy_command[0],
                "policy_linear_y_mps": sample.policy_command[1],
                "policy_angular_z_radps": sample.policy_command[2],
                "policy_command_start_source_time_s": sample.command_observed_time_s + 1.0,
                "policy_stop_source_time_s": sample.policy_stop_time_s + 1.0,
                "x_m": sample.pose[0],
                "y_m": sample.pose[1],
                "z_m": sample.base_height_m,
                "base_height_m": sample.base_height_m,
                "yaw_rad": sample.pose[2],
                "reported_vx_mps": sample.velocity[0],
                "reported_vy_mps": sample.velocity[1],
                "reported_wz_radps": sample.velocity[2],
                "derived_vx_mps": sample.velocity[0],
                "derived_vy_mps": sample.velocity[1],
                "derived_wz_radps": sample.velocity[2],
                "max_tilt_rad": sample.max_tilt_rad,
                "fallen": sample.fallen,
            })


def metadata(trial_id: str, family: str) -> dict:
    return {
        "dataset_id": "synthetic_pilot_v1",
        "platform": "mujoco",
        "collected_at": "2026-08-27T12:00:00Z",
        "description": "Synthetic extraction pilot",
        "provenance": {
            "git_commit": "0" * 40,
            "git_dirty": True,
            "policy_sha256": "1" * 64,
            "robot_model_sha256": "2" * 64,
            "configuration_sha256": "3" * 64,
            "scene_sha256": "4" * 64,
        },
        "trial": {
            "trial_id": trial_id,
            "split": "calibration",
            "seed": 101,
            "repetition": 1,
            "command_family": family,
        },
    }


def test_synthetic_three_command_pilot_assembles_and_validates(tmp_path: Path) -> None:
    trial_specs = [
        ("forward_010", 0, 0.1, "forward_step_stop"),
        ("forward_020", 0, 0.2, "forward_step_stop"),
        ("yaw_pos_010", 2, 0.1, "yaw_step_stop"),
    ]
    datasets = []
    for trial_id, axis, target, family in trial_specs:
        samples_path = tmp_path / f"{trial_id}.csv"
        write_sample_csv(samples_path, trial_id, synthetic_samples(axis, target))
        datasets.append(extraction.assemble_dataset(samples_path, metadata(trial_id, family), 0.2))

    combined = assembler.assemble(metadata("unused", "forward_step_stop"), datasets)
    schema = extraction.json.loads(extraction.DEFAULT_SCHEMA.read_text())
    extraction.validate_dataset(combined, extraction.DEFAULT_SCHEMA)
    assert [trial["trial_id"] for trial in combined["trials"]] == [
        "forward_010", "forward_020", "yaw_pos_010",
    ]
    assert len(schema["definitions"]["metrics"]["required"]) > 0

    repeatability = summary.summarize(combined)
    assert repeatability["trial_count"] == 3
    assert len(repeatability["groups"]) == 3
    assert all(group["valid_trial_count"] == 1 for group in repeatability["groups"])


def test_rejects_reversed_source_timestamps(tmp_path: Path) -> None:
    samples = synthetic_samples(0, 0.1)
    samples[5] = replace(samples[5], time_s=samples[4].time_s - 0.01)
    samples_path = tmp_path / "reversed.csv"
    write_sample_csv(samples_path, "forward_010", samples)

    with pytest.raises(ValueError, match="source timestamps"):
        extraction.read_samples(samples_path, 0.2)


def test_frozen_split_orders_cover_each_command_once() -> None:
    config_path = (
        SCRIPT.parents[1]
        / "ros2_ws/src/dynamic_obstacle_benchmarks/config/g1_response_command_order_v1.yaml"
    )
    config = yaml.safe_load(config_path.read_text())
    command_ids = {command["id"] for command in config["commands"]}
    calibration = set(config["splits"]["calibration"]["seeds"])
    held_out = set(config["splits"]["held_out"]["seeds"])

    assert calibration.isdisjoint(held_out)
    assert set(config["execution_order_by_seed"]) == calibration | held_out
    for order in config["execution_order_by_seed"].values():
        assert len(order) == len(command_ids)
        assert set(order) == command_ids
