from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

VALIDATOR = Path(__file__).with_name("validate_fixed_command_result.py")


def valid_result() -> dict[str, object]:
    return {
        "provenance": {
            "policy_sha256": "a" * 64,
            "scene": "scene_29dof_empty.xml",
        },
        "command": {"linear_x": 0.35, "linear_y": 0.0, "angular_z": 0.0},
        "reactivity": {
            "response_axis": "linear_x",
            "motion_onset_latency_s": 0.45,
            "steady_state_response": 0.30,
            "stop_settling_time_s": 0.70,
            "final_stop_speed_p95": 0.04,
            "post_stop_net_drift_m": 0.11,
        },
        "max_tilt_rad": 0.07,
        "fallen": False,
    }


def run_validator(tmp_path: Path, result: dict[str, object]) -> subprocess.CompletedProcess[str]:
    result_path = tmp_path / "result.json"
    result_path.write_text(json.dumps(result), encoding="utf-8")
    return subprocess.run(
        (sys.executable, str(VALIDATOR), str(result_path)),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def test_fixed_command_validator_accepts_safe_measured_response(tmp_path: Path) -> None:
    result = run_validator(tmp_path, valid_result())

    assert result.returncode == 0
    assert "validation passed" in result.stdout


def test_mixed_diagnostic_is_not_single_axis_acceptance(tmp_path):
    data = valid_result()
    data['command']['angular_z'] = .2
    data['metric_role'] = 'mixed_command_diagnostic_only'
    result = run_validator(tmp_path,data)
    assert result.returncode == 1
    assert 'exactly one command component' in result.stdout
    assert 'not single-axis acceptance' in result.stdout


def test_fixed_command_validator_rejects_fall_and_missing_policy_hash(
    tmp_path: Path,
) -> None:
    data = valid_result()
    data["fallen"] = True
    data["provenance"] = {"scene": "scene_29dof_empty.xml"}
    result = run_validator(tmp_path, data)

    assert result.returncode == 1
    assert "robot must not fall" in result.stdout
    assert "policy SHA-256 must be recorded" in result.stdout
