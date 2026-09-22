from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PATH_HELPER = REPO_ROOT / "scripts" / "benchmark_output_paths.sh"
PERSON_WRAPPER = REPO_ROOT / "scripts" / "run_extreme_lab_person_avoidance_benchmark.sh"
WAYPOINT_WRAPPER = REPO_ROOT / "scripts" / "run_extreme_lab_waypoint_benchmark.sh"
RESULTS_ROOT = REPO_ROOT / "ros2_ws" / "src" / "dynamic_obstacle_benchmarks" / "results"


def resolve_paths(output: str, result_dir: str = "", result_dir_set: str = "0") -> subprocess.CompletedProcess[str]:
    script = """
source "$1"
resolve_benchmark_output_paths "$2" "$3" "$4" "$5" default_stem || exit $?
printf '%s\n' "$HOST_JSON" "$CONTAINER_JSON" "$RESULT_DIR" "$BASENAME"
"""
    return subprocess.run(
        (
            "bash",
            "-c",
            script,
            "bash",
            str(PATH_HELPER),
            str(REPO_ROOT),
            output,
            result_dir,
            result_dir_set,
        ),
        text=True,
        capture_output=True,
        check=False,
    )


def write_fake_docker(directory: Path, log_path: Path) -> None:
    docker = directory / "docker"
    docker.write_text(
        """#!/usr/bin/env python3
import hashlib
import json
import os
import sys
from pathlib import Path

args = sys.argv[1:]
with Path(os.environ['FAKE_DOCKER_LOG']).open('a') as handle:
    handle.write(json.dumps(args) + '\\n')
if '--output' not in args:
    raise SystemExit(0)
output = args[args.index('--output') + 1]
if not output.startswith('/workspace/ros2_ws/'):
    raise SystemExit(9)
host = Path.cwd() / 'ros2_ws' / output.removeprefix('/workspace/ros2_ws/')
people = int(args[args.index('--people') + 1])
goals = int(args[args.index('--goals') + 1])
host.parent.mkdir(parents=True, exist_ok=True)
result = {
    'schema_version': 2,
    'run_id': 'fake-live-run',
    'seed': 42,
    'mode': 'person_avoidance',
    'pedestrian_scenario': None,
    'backend': 'mujoco',
    'valid_trial': True,
    'invalid_reason': None,
    'success': True,
    'collisions': 0,
    'fallen': False,
    'no_movement': False,
    'left_valid_area': False,
    'speed_limit_violations': 0,
    'people_count': people,
    'people_requested': people,
    'people_observed': people,
    'people_active_initial': people,
    'people_active_minimum': people,
    'people_active_final': people,
    'data_sources': {
        'requested': {
            'control_people': 'mujoco_truth_oracle',
            'scoring_people': 'mujoco_simulator_truth',
            'scan': 'mujoco_raycast',
        },
        'observed': {
            'control_people': 'mujoco_truth_oracle',
            'scoring_people': 'mujoco_simulator_truth',
            'scan': 'mujoco_raycast',
        },
    },
    'data_health': {
        'require_exact_people_count': True,
        'max_odom_age_s': 0.01,
        'odom_stale_timeout_s': 1.0,
        'max_pedestrian_age_s': 0.01,
        'pedestrian_stale_timeout_s': 1.0,
        'max_scan_age_s': 0.01,
        'max_control_people_source_age_s': 0.01,
        'max_odom_sample_stall_s': 0.01,
        'max_pedestrian_sample_stall_s': 0.01,
        'max_scan_sample_stall_s': 0.01,
        'scan_stale_timeout_s': 1.0,
        'scan_messages_observed': 10,
    },
    'goals_requested': goals,
    'goals_reached': goals,
    'final_score': 100.0,
    'safety_score': 100.0,
    'task_score': 100.0,
    'speed_score': 100.0,
    'efficiency_score': 100.0,
    'total_time_s': 1.0,
    'actual_path_length_m': 1.0,
    'scenario_file': str(host.with_name(host.stem + '_scenario.yaml')),
    'events': [],
    'goals': [
        {'index': index, 'reached': True, 'failure_reason': None}
        for index in range(goals)
    ],
}
goals_path = host.with_name(host.stem + '_goals.csv')
events_path = host.with_name(host.stem + '_events.csv')
scenario_path = host.with_name(host.stem + '_scenario.yaml')
goals_path.write_text('index,reached\\n' + ''.join(f'{index},True\\n' for index in range(goals)))
events_path.write_text('time_s,type\\n')
scenario_path.write_text(json.dumps({
    'schema_version': 2,
    'run_id': result['run_id'],
    'seed': result['seed'],
    'mode': result['mode'],
    'backend': result['backend'],
    'pedestrian_scenario': result['pedestrian_scenario'],
    'goals': [{} for _ in range(goals)],
    'people': [{} for _ in range(people)],
}))
result['artifacts'] = {
    'goals_csv': {'path': goals_path.name, 'sha256': hashlib.sha256(goals_path.read_bytes()).hexdigest()},
    'events_csv': {'path': events_path.name, 'sha256': hashlib.sha256(events_path.read_bytes()).hexdigest()},
    'scenario': {'path': scenario_path.name, 'sha256': hashlib.sha256(scenario_path.read_bytes()).hexdigest()},
}
host.write_text(json.dumps(result))
"""
    )
    docker.chmod(0o755)


def run_with_fake_docker(
    wrapper: Path,
    output: Path,
    people_free: bool,
    tmp_path: Path,
) -> tuple[subprocess.CompletedProcess[str], Path]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker_log = tmp_path / "docker.jsonl"
    write_fake_docker(fake_bin, docker_log)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["FAKE_DOCKER_LOG"] = str(docker_log)
    command = [str(wrapper), "--no-sim", "--minimum-score", "0", "--output", str(output)]
    if people_free:
        command.extend(["--goals", "1"])
    else:
        command.extend(["--people", "1", "--goals", "1", "--mode", "smoke"])
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed, docker_log


def test_resolves_exact_host_workspace_and_relative_paths() -> None:
    relative = resolve_paths("ros2_ws/results with spaces/result.json")
    assert relative.returncode == 0
    lines = relative.stdout.splitlines()
    assert lines[0] == str(REPO_ROOT / "ros2_ws" / "results with spaces" / "result.json")
    assert lines[1] == "/workspace/ros2_ws/results with spaces/result.json"

    workspace = resolve_paths("/workspace/ros2_ws/results/result.json")
    assert workspace.returncode == 0
    assert workspace.stdout.splitlines()[0] == str(REPO_ROOT / "ros2_ws/results/result.json")


def test_rejects_invalid_output_paths_and_conflicting_modes() -> None:
    outside = resolve_paths("/tmp/result.json")
    assert outside.returncode == 2
    assert "must be under" in outside.stderr
    wrong_extension = resolve_paths("ros2_ws/results/result.JSON")
    assert wrong_extension.returncode == 2
    conflict = resolve_paths(
        "ros2_ws/results/result.json",
        "ros2_ws/results",
        "1",
    )
    assert conflict.returncode == 2
    assert "mutually exclusive" in conflict.stderr


def test_wrappers_reject_missing_output_value() -> None:
    for wrapper in (PERSON_WRAPPER, WAYPOINT_WRAPPER):
        completed = subprocess.run(
            (str(wrapper), "--output"),
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        assert completed.returncode == 2
        assert "--output requires PATH" in completed.stderr


def test_person_wrapper_uses_exact_container_output(tmp_path: Path) -> None:
    result_dir = Path(tempfile.mkdtemp(prefix="output_contract_", dir=RESULTS_ROOT))
    try:
        output = result_dir / "person result.json"
        completed, docker_log = run_with_fake_docker(
            PERSON_WRAPPER, output, False, tmp_path
        )
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert output.is_file()
        assert output.with_suffix(".txt").is_file()
        calls = [json.loads(line) for line in docker_log.read_text().splitlines()]
        benchmark_call = next(call for call in calls if "--output" in call)
        assert benchmark_call.count("--output") == 1
        container_output = benchmark_call[benchmark_call.index("--output") + 1]
        assert container_output.endswith("/person result.json")
        assert container_output.startswith("/workspace/ros2_ws/")
    finally:
        shutil.rmtree(result_dir)


def test_waypoint_wrapper_uses_exact_container_output(tmp_path: Path) -> None:
    result_dir = Path(tempfile.mkdtemp(prefix="output_contract_", dir=RESULTS_ROOT))
    try:
        output = result_dir / "waypoint_result.json"
        completed, docker_log = run_with_fake_docker(
            WAYPOINT_WRAPPER, output, True, tmp_path
        )
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert output.is_file()
        assert output.with_suffix(".txt").is_file()
        calls = [json.loads(line) for line in docker_log.read_text().splitlines()]
        benchmark_call = next(call for call in calls if "--output" in call)
        assert benchmark_call.count("--output") == 1
        container_output = benchmark_call[benchmark_call.index("--output") + 1]
        assert container_output.endswith("/waypoint_result.json")
        assert container_output.startswith("/workspace/ros2_ws/")
    finally:
        shutil.rmtree(result_dir)
