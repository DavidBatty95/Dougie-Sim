from __future__ import annotations

import runpy
from pathlib import Path
from typing import Any

import pytest
import torch


def load_evaluator() -> dict[str, Any]:
    path = Path(__file__).with_name("evaluate_g1_stability.py")
    return runpy.run_path(str(path), run_name="evaluate_g1_stability")


def test_navigation_schedule_covers_supported_yaw_arcs_and_forward_speeds() -> None:
    evaluator = load_evaluator()
    phases = evaluator["command_schedule"](evaluator["NAVIGATION_TASK_ID"])
    commands = {phase.name: phase.command for phase in phases if phase.kind == "move"}

    assert commands == {
        "yaw_left_012": (0.0, 0.0, 0.12),
        "yaw_right_012": (0.0, 0.0, -0.12),
        "yaw_left_020": (0.0, 0.0, 0.20),
        "yaw_right_020": (0.0, 0.0, -0.20),
        "arc_left": (0.20, 0.0, 0.12),
        "arc_right": (0.20, 0.0, -0.12),
        "forward_005": (0.05, 0.0, 0.0),
        "forward_010": (0.10, 0.0, 0.0),
        "forward_020": (0.20, 0.0, 0.0),
    }


def test_reactive_schedule_independently_covers_bilateral_evasive_axes() -> None:
    evaluator = load_evaluator()
    phases = evaluator["command_schedule"](evaluator["REACTIVE_TASK_ID"])
    commands = {phase.name: phase.command for phase in phases if phase.kind == "move"}

    assert commands == {
        "lateral_left_005": (0.0, 0.05, 0.0),
        "lateral_right_005": (0.0, -0.05, 0.0),
        "lateral_left_010": (0.0, 0.10, 0.0),
        "lateral_right_010": (0.0, -0.10, 0.0),
        "yaw_left_012": (0.0, 0.0, 0.12),
        "yaw_right_012": (0.0, 0.0, -0.12),
        "yaw_left_020": (0.0, 0.0, 0.20),
        "yaw_right_020": (0.0, 0.0, -0.20),
        "diagonal_arc_left": (0.08, 0.08, 0.12),
        "diagonal_arc_right": (0.08, -0.08, -0.12),
        "forward_010": (0.10, 0.0, 0.0),
        "forward_020": (0.20, 0.0, 0.0),
    }


def test_step_foundation_schedule_regresses_stand_forward_reverse_and_yaw() -> None:
    evaluator = load_evaluator()
    phases = evaluator["command_schedule"](evaluator["STEP_FOUNDATION_TASK_ID"])
    commands = {phase.name: phase.command for phase in phases if phase.kind == "move"}

    assert commands == {
        "forward_005": (0.05, 0.0, 0.0),
        "forward_010": (0.10, 0.0, 0.0),
        "forward_020": (0.20, 0.0, 0.0),
        "forward_035": (0.35, 0.0, 0.0),
        "reverse_005": (-0.05, 0.0, 0.0),
        "reverse_010": (-0.10, 0.0, 0.0),
        "yaw_left": (0.0, 0.0, 0.20),
        "yaw_right": (0.0, 0.0, -0.20),
    }


def test_gait_metrics_measure_touchdowns_symmetry_clearance_and_scuffing() -> None:
    evaluator = load_evaluator()
    accumulator = evaluator["GaitMetricsAccumulator"](1, 2, torch.device("cpu"))
    actions = torch.zeros((1, 2))
    active = torch.tensor([True])
    root_height = torch.tensor([0.78])
    gravity = torch.tensor([[0.01, -0.02, -1.0]])

    samples = (
        (0.1, [[True, True]], [[0.0, 0.0]], [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        (0.2, [[False, True]], [[0.01, 0.0]], [[0.10, 0.0, 0.05], [0.0, 0.0, 0.0]]),
        (0.3, [[True, True]], [[0.05, 0.0]], [[0.0, 0.0, -0.20], [0.0, 0.0, 0.0]]),
        (0.4, [[True, False]], [[0.0, 0.04]], [[0.0, 0.0, 0.0], [0.08, 0.0, 0.03]]),
        (0.5, [[True, True]], [[0.0, 0.04]], [[0.0, 0.0, 0.0], [0.0, 0.0, -0.15]]),
        (0.6, [[False, True]], [[0.03, 0.0]], [[0.06, 0.0, 0.02], [0.0, 0.0, 0.0]]),
        (0.7, [[True, True]], [[0.03, 0.0]], [[0.0, 0.0, -0.10], [0.0, 0.0, 0.0]]),
    )
    for elapsed, contact, heights, velocities in samples:
        accumulator.update(
            elapsed_s=elapsed,
            step_dt=0.1,
            active=active,
            actions=actions,
            in_contact=torch.tensor(contact),
            foot_height=torch.tensor(heights),
            foot_velocity=torch.tensor([velocities]),
            root_height=root_height,
            projected_gravity=gravity,
        )

    result = accumulator.result()
    assert result["touchdown_count_left_right"] == [[2.0, 1.0]]
    assert result["stride_period_mean_s_left_right"][0][0] == pytest.approx(0.4)
    assert result["alternating_touchdown_fraction"] == [1.0]
    # Left airborne peaks are 0.01 and 0.03 m; the 0.05 m sample is already
    # in contact and must not be counted as swing clearance.
    assert result["mean_swing_peak_height_m_left_right"][0] == pytest.approx([0.02, 0.04])
    assert result["toe_scuff_count_left_right"] == [[1.0, 0.0]]
    assert result["max_touchdown_vertical_speed_mps_left_right"][0] == pytest.approx(
        [0.2, 0.15]
    )
