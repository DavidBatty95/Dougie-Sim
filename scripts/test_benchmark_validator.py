from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path


VALIDATOR = Path(__file__).with_name("validate_benchmark_result.py")


def write_result(tmp_path: Path, **overrides) -> Path:
    result = tmp_path / "result.json"
    data = {
        "schema_version": 2,
        "run_id": "test-run-id",
        "seed": 42,
        "mode": "person_avoidance",
        "backend": "mujoco",
        "valid_trial": True,
        "success": True,
        "collisions": 0,
        "fallen": False,
        "no_movement": False,
        "left_valid_area": False,
        "speed_limit_violations": 0,
        "people_count": 4,
        "people_requested": 4,
        "people_observed": 4,
        "people_active_initial": 4,
        "people_active_minimum": 0,
        "people_active_final": 0,
        "data_sources": {
            "requested": {
                "control_people": "mujoco_truth_oracle",
                "scoring_people": "mujoco_simulator_truth",
                "scan": "mujoco_raycast_requested_with_fallback",
            },
            "observed": {
                "control_people": "mujoco_truth_oracle",
                "scoring_people": "mujoco_simulator_truth",
                "scan": "mujoco_raycast",
            },
        },
        "data_health": {
            "require_exact_people_count": True,
            "odom_stale_timeout_s": 1.0,
            "pedestrian_stale_timeout_s": 1.0,
            "scan_stale_timeout_s": 1.0,
            "max_odom_age_s": 0.1,
            "max_pedestrian_age_s": 0.1,
            "max_scan_age_s": 0.1,
            "max_control_people_source_age_s": 0.1,
            "max_odom_sample_stall_s": 0.1,
            "max_pedestrian_sample_stall_s": 0.1,
            "max_scan_sample_stall_s": 0.1,
            "scan_messages_observed": 100,
        },
        "pedestrian_scenario": "demo_endurance_plaza",
        "goals_requested": 4,
        "goals_reached": 4,
        "moving_pedestrian_time_s": 24.0,
        "avoidance_interventions": 4,
        "yield_episodes": 4,
        "resume_episodes": 4,
        "yield_time_s": 4.0,
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
        "invalid_reason": None,
        "scenario_file": str(tmp_path / "result_scenario.yaml"),
        "events": [],
        "goals": [
            {"index": index, "reached": True, "failure_reason": None}
            for index in range(4)
        ],
    }
    data.update(overrides)
    goals_path = result.with_name(result.stem + "_goals.csv")
    events_path = result.with_name(result.stem + "_events.csv")
    scenario_path = result.with_name(result.stem + "_scenario.yaml")
    goals_path.write_text("index,reached\n" + "".join(f"{index},True\n" for index in range(len(data["goals"]))))
    events_path.write_text("time_s,type\n")
    scenario_path.write_text(json.dumps({
        "schema_version": 2,
        "run_id": data.get("run_id"),
        "seed": data.get("seed"),
        "mode": data.get("mode"),
        "backend": data.get("backend"),
        "pedestrian_scenario": data.get("pedestrian_scenario"),
        "goals": [{} for _ in range(data.get("goals_requested", 0))],
        "people": [{} for _ in range(data.get("people_requested", 0))],
    }))
    data["artifacts"] = {
        "goals_csv": artifact_reference(goals_path),
        "events_csv": artifact_reference(events_path),
        "scenario": artifact_reference(scenario_path),
    }
    result.write_text(json.dumps(data))
    return result


def artifact_reference(path: Path) -> dict[str, str]:
    return {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


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


def test_visual_session_is_not_a_scored_acceptance_result(tmp_path: Path) -> None:
    result = run_validator(write_result(tmp_path, visual_session=True))
    assert result.returncode != 0
    assert "visual sessions are not scored" in result.stdout + result.stderr


def test_result_without_yields_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, yield_episodes=0))
    assert completed.returncode == 1
    assert "yield episodes" in completed.stdout


def test_result_without_resumes_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, resume_episodes=0))
    assert completed.returncode == 1
    assert "returns to normal after yielding" in completed.stdout


def test_supervisor_overrides_are_not_used_as_yield_evidence(tmp_path: Path) -> None:
    assert run_validator(write_result(tmp_path, avoidance_interventions=0)).returncode == 0


def test_result_using_lateral_motion_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, max_abs_safe_lateral_cmd_mps=0.05))
    assert completed.returncode == 1
    assert "unsupported lateral motion" in completed.stdout


def test_missing_schema_version_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, schema_version=None))
    assert completed.returncode == 1
    assert "schema_version must be 2" in completed.stdout


def test_unsupported_or_non_integer_schema_version_fails(tmp_path: Path) -> None:
    for version in (1, 3, "2", True):
        completed = run_validator(write_result(tmp_path, schema_version=version))
        assert completed.returncode == 1
        assert "schema_version must be 2" in completed.stdout


def test_population_mismatch_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, people_observed=3))
    assert completed.returncode == 1
    assert "exactly match" in completed.stdout


def test_stale_live_stream_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, data_health={
        "require_exact_people_count": True,
        "odom_stale_timeout_s": 1.0,
        "pedestrian_stale_timeout_s": 1.0,
        "scan_stale_timeout_s": 1.0,
        "max_odom_age_s": 1.1,
        "max_pedestrian_age_s": 0.1,
        "max_scan_age_s": 0.1,
        "max_control_people_source_age_s": 0.1,
        "max_odom_sample_stall_s": 0.1,
        "max_pedestrian_sample_stall_s": 0.1,
        "max_scan_sample_stall_s": 0.1,
        "scan_messages_observed": 100,
    }))
    assert completed.returncode == 1
    assert "max_odom_age_s" in completed.stdout


def test_missing_source_provenance_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, data_sources={}))
    assert completed.returncode == 1
    assert "declare requested and observed control, scoring, and scan sources" in completed.stdout


def test_scan_source_mismatch_fails(tmp_path: Path) -> None:
    sources = {
        "requested": {
            "control_people": "mujoco_truth_oracle",
            "scoring_people": "mujoco_simulator_truth",
            "scan": "mujoco_raycast",
        },
        "observed": {
            "control_people": "mujoco_truth_oracle",
            "scoring_people": "mujoco_simulator_truth",
            "scan": "fallback_geometry",
        },
    }
    completed = run_validator(write_result(tmp_path, data_sources=sources))
    assert completed.returncode == 1
    assert "observed scan source must match" in completed.stdout


def test_boolean_numeric_fields_fail(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, final_score=True, collisions=False))
    assert completed.returncode == 1
    assert "collisions must be zero" in completed.stdout
    assert "final_score must be finite" in completed.stdout


def test_contradictory_goal_summary_fails(tmp_path: Path) -> None:
    goals = [
        {"index": index, "reached": index < 3, "failure_reason": None}
        for index in range(4)
    ]
    completed = run_validator(write_result(tmp_path, goals=goals))
    assert completed.returncode == 1
    assert "goal records must match goals_reached" in completed.stdout


def test_invalid_active_population_lifecycle_fails(tmp_path: Path) -> None:
    completed = run_validator(write_result(tmp_path, people_active_minimum=5))
    assert completed.returncode == 1
    assert "active pedestrian counts" in completed.stdout


def test_control_source_mismatch_fails(tmp_path: Path) -> None:
    result = write_result(tmp_path)
    data = json.loads(result.read_text())
    data["data_sources"]["observed"]["control_people"] = "sensor_or_fused"
    result.write_text(json.dumps(data))
    completed = run_validator(result)
    assert completed.returncode == 1
    assert "observed control people source must match" in completed.stdout


def test_artifact_checksum_mismatch_fails(tmp_path: Path) -> None:
    result = write_result(tmp_path)
    result.with_name(result.stem + "_scenario.yaml").write_text("stale: true\n")
    completed = run_validator(result)
    assert completed.returncode == 1
    assert "artifact checksum mismatch: scenario" in completed.stdout


def test_scenario_run_id_mismatch_fails(tmp_path: Path) -> None:
    result = write_result(tmp_path)
    data = json.loads(result.read_text())
    scenario = result.with_name(result.stem + "_scenario.yaml")
    scenario_data = json.loads(scenario.read_text())
    scenario_data["run_id"] = "another-run"
    scenario.write_text(json.dumps(scenario_data))
    data["artifacts"]["scenario"] = artifact_reference(scenario)
    result.write_text(json.dumps(data))
    completed = run_validator(result)
    assert completed.returncode == 1
    assert "scenario run_id must match result" in completed.stdout
