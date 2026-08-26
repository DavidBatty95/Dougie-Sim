from __future__ import annotations

import runpy
from pathlib import Path
from typing import Any


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
