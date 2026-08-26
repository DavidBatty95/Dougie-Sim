#!/usr/bin/env python3
"""Run reproducible G1 simulation demos and scored benchmarks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import selectors
import shlex
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
OFFICIAL_POLICY = (
    REPO_ROOT
    / "vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx"
)
DEFAULT_RESULTS = (
    REPO_ROOT / "ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite"
)


@dataclass(frozen=True)
class DemoCase:
    name: str
    description: str
    command: tuple[str, ...]
    timeout_s: int
    scored: bool = True
    interactive: bool = False
    needs_official_policy: bool = False
    needs_extreme_lab: bool = False


def case_catalog(run_dir: Path, seed: int, headless: bool) -> dict[str, DemoCase]:
    rviz = ("--no-rviz",) if headless else ()
    official_launch = (
        "--sim-arg",
        f"policy_path:={OFFICIAL_POLICY}",
        "--sim-arg",
        "policy_profile:=unitree_rl_lab_g1_29dof",
        "--sim-arg",
        "policy_obs_dim:=480",
        "--sim-arg",
        "policy_action_dim:=29",
        "--sim-arg",
        "mujoco_initial_keyframe:=unitree_rl_lab_stand",
        "--sim-arg",
        "max_lin_vel_y:=0.0",
    )
    waypoint_motion = (
        "--sim-arg",
        "max_lin_vel_x:=0.45",
        "--sim-arg",
        "max_ang_vel_z:=0.20",
        "--sim-arg",
        "cmd_accel_x:=0.35",
        "--sim-arg",
        "cmd_accel_yaw:=0.35",
        "--sim-arg",
        "turning_arc_min_forward_speed:=0.20",
        "--sim-arg",
        "turning_arc_min_yaw_speed:=0.12",
    )
    return {
        "mock-avoidance": DemoCase(
            name="mock-avoidance",
            description="Deterministic person-avoidance benchmark mechanics",
            command=(
                str(SCRIPTS / "run_person_avoidance_benchmark.sh"),
                "--mode",
                "smoke",
                "--seed",
                str(seed),
                "--people",
                "4",
                "--goals",
                "3",
                "--minimum-score",
                "95",
                "--output",
                str(run_dir / "mock-avoidance/result.json"),
            ),
            timeout_s=300,
        ),
        "fixed-forward": DemoCase(
            name="fixed-forward",
            description="Official-policy forward response and stop characterization",
            command=(
                str(SCRIPTS / "run_g1_fixed_command_test.sh"),
                "--policy",
                str(OFFICIAL_POLICY),
                "--policy-profile",
                "unitree_rl_lab_g1_29dof",
                "--initial-keyframe",
                "unitree_rl_lab_stand",
                "--scene",
                "scene_29dof_empty.xml",
                "--linear-x",
                "0.35",
                "--duration",
                "8",
                "--stop-duration",
                "5",
                "--max-linear-x",
                "0.45",
                "--cmd-accel-x",
                "0.35",
                "--no-rviz",
                "--validate",
                "--result-dir",
                str(run_dir / "fixed-forward"),
            ),
            timeout_s=480,
            needs_official_policy=True,
        ),
        "waypoint-live": DemoCase(
            name="waypoint-live",
            description="Scored people-free Nav2 waypoint baseline",
            command=(
                str(SCRIPTS / "run_extreme_lab_waypoint_benchmark.sh"),
                "--seed",
                str(seed),
                "--goals",
                "2",
                "--minimum-score",
                "80",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_waypoint_benchmark.yaml",
                "--result-dir",
                str(run_dir / "waypoint-live"),
                *rviz,
                *official_launch,
                *waypoint_motion,
            ),
            timeout_s=900,
            needs_official_policy=True,
            needs_extreme_lab=True,
        ),
        "social-live": DemoCase(
            name="social-live",
            description="Scored conservative live person avoidance",
            command=(
                str(SCRIPTS / "run_extreme_lab_person_avoidance_benchmark.sh"),
                "--mode",
                "smoke",
                "--seed",
                str(seed),
                "--people",
                "2",
                "--goals",
                "1",
                "--minimum-score",
                "70",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_social_benchmark.yaml",
                "--result-dir",
                str(run_dir / "social-live"),
                *rviz,
                *official_launch,
                *waypoint_motion,
                "--sim-arg",
                "mujoco_pedestrian_start_delay:=45.0",
                "--sim-arg",
                "tracker_use_sensor_input:=false",
                "--sim-arg",
                "tracker_local_tracking_radius:=0.0",
                "--sim-arg",
                "livox_scan_points:=1200",
                "--sim-arg",
                "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_oracle_v1.yaml",
            ),
            timeout_s=900,
            needs_official_policy=True,
            needs_extreme_lab=True,
        ),
        "social-endurance": DemoCase(
            name="social-endurance",
            description="Prolonged deterministic two-pedestrian yield-and-resume demonstration",
            command=(
                str(SCRIPTS / "run_extreme_lab_person_avoidance_benchmark.sh"),
                "--mode",
                "smoke",
                "--seed",
                str(seed),
                "--people",
                "2",
                "--goals",
                "2",
                "--minimum-score",
                "70",
                "--require-avoidance",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_social_endurance_benchmark.yaml",
                "--result-dir",
                str(run_dir / "social-endurance"),
                *rviz,
                *official_launch,
                *waypoint_motion,
                "--sim-arg",
                "mujoco_pedestrian_scenario:=demo_endurance",
                "--sim-arg",
                "mujoco_pedestrian_start_delay:=30.0",
                "--sim-arg",
                "tracker_use_sensor_input:=false",
                "--sim-arg",
                "tracker_local_tracking_radius:=0.0",
                "--sim-arg",
                "livox_scan_points:=1200",
                "--sim-arg",
                "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_oracle_v1.yaml",
            ),
            timeout_s=1200,
            needs_official_policy=True,
            needs_extreme_lab=True,
        ),
        "waypoint-visual": DemoCase(
            name="waypoint-visual",
            description="Interactive bundled-scene waypoint and RViz showcase",
            command=(
                str(SCRIPTS / "run_waypoint_navigation_demo.sh"),
                "mujoco_scene:=scene_29dof_obstacles.xml",
            ),
            timeout_s=0,
            scored=False,
            interactive=True,
            needs_official_policy=True,
        ),
        "social-visual": DemoCase(
            name="social-visual",
            description="Interactive Extreme Lab social-navigation showcase",
            command=(
                str(SCRIPTS / "run_collision_avoidance_demo.sh"),
                f"policy_path:={OFFICIAL_POLICY}",
                "policy_profile:=unitree_rl_lab_g1_29dof",
                "policy_obs_dim:=480",
                "policy_action_dim:=29",
                "mujoco_initial_keyframe:=unitree_rl_lab_stand",
                "max_lin_vel_y:=0.0",
            ),
            timeout_s=0,
            scored=False,
            interactive=True,
            needs_official_policy=True,
            needs_extreme_lab=True,
        ),
    }


PRESETS = {
    "quick": ("mock-avoidance",),
    "locomotion": ("mock-avoidance", "fixed-forward"),
    "navigation": ("mock-avoidance", "fixed-forward", "waypoint-live"),
    "social": ("mock-avoidance", "social-live"),
    "endurance": ("mock-avoidance", "fixed-forward", "social-endurance"),
    "full": (
        "mock-avoidance",
        "fixed-forward",
        "waypoint-live",
        "social-live",
        "social-endurance",
    ),
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(*args: str) -> str | None:
    result = subprocess.run(
        ("git", *args),
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def select_cases(
    catalog: dict[str, DemoCase], preset: str | None, requested: list[str]
) -> list[DemoCase]:
    names = list(PRESETS[preset]) if preset else []
    names.extend(requested)
    if not names:
        names.extend(PRESETS["quick"])
    selected: list[DemoCase] = []
    seen: set[str] = set()
    for name in names:
        if name not in catalog:
            raise ValueError(f"unknown demo case: {name}")
        if name not in seen:
            selected.append(catalog[name])
            seen.add(name)
    if len(selected) > 1 and any(case.interactive for case in selected):
        raise ValueError("interactive cases must be run alone")
    return selected


def write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def stream_process(
    command: tuple[str, ...], log: TextIO, timeout_s: int
) -> tuple[int, bool]:
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout_s if timeout_s > 0 else None
    timed_out = False
    try:
        while True:
            for key, _ in selector.select(timeout=0.2):
                line = key.fileobj.readline()
                if line:
                    print(line, end="")
                    log.write(line)
                    log.flush()
            if process.poll() is not None:
                for remainder in process.stdout:
                    print(remainder, end="")
                    log.write(remainder)
                break
            if deadline is not None and time.monotonic() >= deadline:
                timed_out = True
                os.killpg(process.pid, 15)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, 9)
                break
    except KeyboardInterrupt:
        os.killpg(process.pid, 15)
        process.wait(timeout=10)
        raise
    finally:
        selector.close()
    return (124 if timed_out else process.returncode), timed_out


def preflight(cases: list[DemoCase], dry_run: bool) -> dict[str, object]:
    checks: dict[str, object] = {}
    compose = subprocess.run(
        ("docker", "compose", "config", "--quiet"),
        cwd=REPO_ROOT,
        check=False,
    )
    if compose.returncode != 0:
        raise RuntimeError("docker compose configuration is invalid")
    checks["docker_compose"] = "ok"
    for case in cases:
        executable = Path(case.command[0])
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f"demo entry point is not executable: {executable}")
    checks["entry_points"] = "ok"
    if any(case.needs_official_policy for case in cases):
        if not dry_run:
            subprocess.run(
                (str(SCRIPTS / "download_unitree_rl_lab_policy.sh"),),
                cwd=REPO_ROOT,
                check=True,
            )
        checks["official_policy"] = (
            file_sha256(OFFICIAL_POLICY) if OFFICIAL_POLICY.is_file() else "dry-run"
        )
    extreme_scene = (
        REPO_ROOT
        / "ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_extreme_lab.xml"
    )
    if any(case.needs_extreme_lab for case in cases):
        if not extreme_scene.is_file():
            raise RuntimeError(f"Extreme Lab scene is unavailable: {extreme_scene}")
        checks["extreme_lab_scene"] = str(extreme_scene)
    return checks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=tuple(PRESETS))
    parser.add_argument("--case", action="append", default=[], dest="cases")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--result-root", type=Path, default=DEFAULT_RESULTS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.result_root.expanduser().resolve() / f"suite_{stamp}"
    if not run_dir.is_relative_to(REPO_ROOT):
        print(
            f"Demo suite result root must be inside {REPO_ROOT}: {args.result_root}",
            file=sys.stderr,
        )
        return 2
    catalog = case_catalog(run_dir, args.seed, args.headless)
    if args.list:
        print("Presets:")
        for name, cases in PRESETS.items():
            print(f"  {name:12} {' '.join(cases)}")
        print("Cases:")
        for case in catalog.values():
            kind = "interactive" if case.interactive else "scored"
            print(f"  {case.name:16} [{kind}] {case.description}")
        return 0
    try:
        cases = select_cases(catalog, args.suite, args.cases)
        checks = preflight(cases, args.dry_run)
    except (RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Demo suite preflight failed: {error}", file=sys.stderr)
        return 2

    manifest = {
        "schema_version": 1,
        "run_id": run_dir.name,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "repository": str(REPO_ROOT),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_dirty": bool(git_output("status", "--porcelain")),
        "seed": args.seed,
        "headless": args.headless,
        "dry_run": args.dry_run,
        "preflight": checks,
        "cases": [
            {
                **asdict(case),
                "command": list(case.command),
                "command_shell": shlex.join(case.command),
            }
            for case in cases
        ],
    }
    write_json(run_dir / "manifest.json", manifest)
    results: list[dict[str, object]] = []
    overall_status = "dry-run" if args.dry_run else "passed"
    try:
        for case in cases:
            case_dir = run_dir / case.name
            case_dir.mkdir(parents=True, exist_ok=True)
            print(f"\n[{case.name}] {case.description}")
            print(f"Command: {shlex.join(case.command)}")
            started = time.monotonic()
            if args.dry_run:
                return_code, timed_out = 0, False
            else:
                with (case_dir / "suite.log").open("w", encoding="utf-8") as log:
                    return_code, timed_out = stream_process(
                        case.command, log, case.timeout_s
                    )
            result = {
                "name": case.name,
                "return_code": return_code,
                "timed_out": timed_out,
                "duration_s": round(time.monotonic() - started, 3),
                "status": (
                    "planned"
                    if args.dry_run
                    else ("passed" if return_code == 0 else "failed")
                ),
            }
            results.append(result)
            write_json(case_dir / "suite_status.json", result)
            if return_code != 0:
                overall_status = "failed"
                if not args.continue_on_failure:
                    break
    except KeyboardInterrupt:
        overall_status = "interrupted"
    finally:
        status = {
            "run_id": run_dir.name,
            "status": overall_status,
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "results": results,
        }
        write_json(run_dir / "status.json", status)
        print(f"\nSuite status: {overall_status}")
        print(f"Artifacts: {run_dir}")
    return 0 if overall_status in ("passed", "dry-run") else 1


if __name__ == "__main__":
    raise SystemExit(main())
