from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


VALIDATOR = Path(__file__).with_name("validate_benchmark_result.py")


def write_result(tmp_path: Path, **overrides) -> Path:
    result = tmp_path / "result.json"
    data = {
        "backend": "mujoco",
        "valid_trial": True,
        "success": True,
        "collisions": 0,
        "fallen": False,
        "no_movement": False,
        "left_valid_area": False,
        "speed_limit_violations": 0,
        "people_count": 2,
        "pedestrian_scenario": "demo_endurance",
        "goals_requested": 2,
        "goals_reached": 2,
        "moving_pedestrian_time_s": 14.0,
        "avoidance_interventions": 2,
        "avoidance_time_s": 4.0,
        "moving_avoidance_time_s": 3.0,
        "post_avoidance_progress_m": 1.2,
        "minimum_body_clearance_m": 0.5,
        "max_abs_safe_lateral_cmd_mps": 0.0,
        "final_score": 90.0,
        "safety_score": 98.0,
        "task_score": 100.0,
        "speed_score": 40.0,
        "efficiency_score": 100.0,
        "total_time_s": 30.0,
        "actual_path_length_m": 3.0,
    }
    data.update(overrides)
    result.write_text(json.dumps(data))
    for suffix in ("_goals.csv", "_events.csv", "_scenario.yaml"):
        result.with_name(result.stem + suffix).write_text("data\n")
    return result


def run_validator(result: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        (
            sys.executable,
            str(VALIDATOR),
            str(result),
            "--backend",
            "mujoco",
            "--minimum-score",
            "70",
            "--require-avoidance",
        ),
        text=True,
        capture_output=True,
        check=False,
    )


def test_prolonged_avoidance_result_passes(tmp_path: Path) -> None:
    assert run_validator(write_result(tmp_path)).returncode == 0


def test_result_without_intervention_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, avoidance_interventions=0))
    assert completed.returncode == 1
    assert "avoidance intervention" in completed.stdout


def test_result_using_lateral_motion_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, max_abs_safe_lateral_cmd_mps=0.05))
    assert completed.returncode == 1
    assert "unsupported lateral motion" in completed.stdout
