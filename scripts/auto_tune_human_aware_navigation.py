#!/usr/bin/env python3
"""Automated tuning harness for Extreme Lab human-aware navigation.

This script repeatedly launches the simulation through the existing Extreme Lab
benchmark wrapper with different launch-argument sets, ranks the results, and
writes a best-args file that can be passed back to bringup_full_sim.sh.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BENCHMARK_SCRIPT = REPO_ROOT / "scripts" / "run_extreme_lab_person_avoidance_benchmark.sh"
DEFAULT_RESULT_DIR = REPO_ROOT / "ros2_ws" / "src" / "dynamic_obstacle_benchmarks" / "results" / "human_aware_auto_tuning"


@dataclass(frozen=True)
class TrialConfig:
    name: str
    sim_args: tuple[str, ...]


TRIAL_CONFIGS = [
    TrialConfig(
        "baseline_current",
        (),
    ),
    TrialConfig(
        "stronger_motion_default_arc",
        (
            "human_aware_max_forward_near_human:=0.42",
            "human_aware_max_lateral_near_human:=0.18",
            "human_aware_max_angular_near_human:=0.62",
            "avoidance_max_lateral_linear:=0.16",
            "avoidance_max_avoid_angular:=0.45",
        ),
    ),
    TrialConfig(
        "move_away_high_lateral",
        (
            "human_aware_max_forward_near_human:=0.38",
            "human_aware_max_lateral_near_human:=0.22",
            "human_aware_max_reverse_velocity:=0.12",
            "human_aware_comfort_radius:=1.45",
            "avoidance_max_lateral_linear:=0.18",
        ),
    ),
    TrialConfig(
        "wide_comfort_pass_behind",
        (
            "human_aware_comfort_radius:=1.65",
            "human_aware_front_arc_distance:=1.5",
            "human_aware_front_arc_angle_deg:=30.0",
            "human_aware_front_arc_weight:=0.85",
            "human_aware_w_pass_front:=26.0",
            "human_aware_w_ttc:=14.0",
        ),
    ),
    TrialConfig(
        "less_stop_more_progress",
        (
            "human_aware_max_forward_near_human:=0.45",
            "human_aware_w_personal_space:=9.0",
            "human_aware_w_ttc:=8.0",
            "human_aware_w_smooth:=2.5",
            "avoidance_danger_radius:=2.3",
            "avoidance_corridor_width:=2.0",
        ),
    ),
    TrialConfig(
        "cautious_close_people",
        (
            "human_aware_max_forward_near_human:=0.32",
            "human_aware_max_lateral_near_human:=0.18",
            "human_aware_comfort_radius:=1.7",
            "human_aware_w_personal_space:=16.0",
            "human_aware_w_ttc:=16.0",
            "avoidance_danger_radius:=2.8",
            "avoidance_critical_radius:=1.8",
        ),
    ),
    TrialConfig(
        "fast_nav_conservative_social",
        (
            "human_aware_max_forward_near_human:=0.48",
            "human_aware_max_lateral_near_human:=0.18",
            "human_aware_max_angular_near_human:=0.70",
            "human_aware_comfort_radius:=1.55",
            "human_aware_w_pass_front:=24.0",
            "human_aware_w_smooth:=3.0",
        ),
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--people", type=int, default=6)
    parser.add_argument("--goals", type=int, default=6)
    parser.add_argument("--mode", default="standard", choices=["smoke", "standard", "stress", "crossing", "bottleneck", "occlusion"])
    parser.add_argument("--backend", default="mujoco", choices=["mock", "mujoco"])
    parser.add_argument("--startup-seconds", type=int, default=32)
    parser.add_argument("--max-trials", type=int, default=len(TRIAL_CONFIGS))
    parser.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR)
    parser.add_argument("--with-rviz", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def run_trial(config: TrialConfig, args: argparse.Namespace, index: int) -> dict:
    trial_dir = args.result_dir / f"trial_{index:02d}_{config.name}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(BENCHMARK_SCRIPT),
        "--seed",
        str(args.seed),
        "--people",
        str(args.people),
        "--goals",
        str(args.goals),
        "--mode",
        args.mode,
        "--backend",
        args.backend,
        "--startup-seconds",
        str(args.startup_seconds),
        "--result-dir",
        str(trial_dir),
    ]
    if not args.with_rviz:
        command.append("--no-rviz")
    for sim_arg in config.sim_args:
        command.extend(["--sim-arg", sim_arg])

    print(f"\n=== Trial {index}: {config.name} ===", flush=True)
    print(" ".join(command), flush=True)
    if args.dry_run:
        return {"trial": index, "name": config.name, "score": 0.0, "sim_args": " ".join(config.sim_args), "dry_run": True}
    completed = subprocess.run(command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    sys.stdout.write(completed.stdout)
    result_json = _newest_json(trial_dir)
    sim_log = _sim_log_from_output(completed.stdout) or _newest_sim_log(trial_dir)
    metrics = parse_result_json(result_json) if result_json else {}
    metrics.update(parse_sim_log(sim_log) if sim_log else {})
    metrics.update(
        {
            "trial": index,
            "name": config.name,
            "returncode": completed.returncode,
            "result_json": str(result_json) if result_json else "",
            "sim_log": str(sim_log) if sim_log else "",
            "sim_args": " ".join(config.sim_args),
        }
    )
    metrics["score"] = tuning_score(metrics)
    return metrics


def _sim_log_from_output(output: str) -> Path | None:
    match = re.search(r"Simulation log: (.+_sim\.log)", output)
    if not match:
        return None
    path = Path(match.group(1).strip())
    return path if path.exists() else None


def _newest_sim_log(directory: Path) -> Path | None:
    logs = sorted(directory.glob("*_sim.log"), key=lambda path: path.stat().st_mtime)
    return logs[-1] if logs else None


def _newest_json(directory: Path) -> Path | None:
    jsons = sorted((path for path in directory.glob("*.json") if not path.name.endswith("_scenario.json")), key=lambda path: path.stat().st_mtime)
    return jsons[-1] if jsons else None


def parse_result_json(path: Path) -> dict:
    data = json.loads(path.read_text())
    return {
        "goals_requested": data.get("goals_requested", 0),
        "goals_reached": data.get("goals_reached", 0),
        "collisions": data.get("collisions", 0),
        "near_misses": data.get("near_misses", 0),
        "critical_near_misses": data.get("critical_near_misses", 0),
        "minimum_separation_m": data.get("minimum_separation_m", ""),
        "minimum_ttc_s": data.get("minimum_ttc_s", ""),
        "total_time_s": data.get("total_time_s", 0.0),
        "path_length_m": data.get("actual_path_length_m", 0.0),
        "efficiency": data.get("efficiency", 0.0),
        "benchmark_final_score": data.get("final_score", data.get("score", 0.0)),
    }


def parse_sim_log(path: Path) -> dict:
    policy_status = re.compile(r"policy status:.*tilt=([0-9.]+).*cmd=\(([-+0-9.]+), ([-+0-9.]+), ([-+0-9.]+)\)")
    override = re.compile(r"Human collision risk")
    planner_debug = re.compile(r"human_avoidance_debug|Human-aware")
    fall = re.compile(r"Robot fall detected")
    max_tilt = 0.0
    max_cmd = 0.0
    nonzero_cmd = 0
    status_count = 0
    override_count = 0
    debug_count = 0
    fallen = False
    for line in path.read_text(errors="replace").splitlines():
        match = policy_status.search(line)
        if match:
            status_count += 1
            tilt = float(match.group(1))
            cmd_mag = abs(float(match.group(2))) + abs(float(match.group(3))) + abs(float(match.group(4)))
            max_tilt = max(max_tilt, tilt)
            max_cmd = max(max_cmd, cmd_mag)
            if cmd_mag > 0.08:
                nonzero_cmd += 1
        if override.search(line):
            override_count += 1
        if planner_debug.search(line):
            debug_count += 1
        if fall.search(line):
            fallen = True
    return {
        "fallen": fallen,
        "max_tilt": round(max_tilt, 3),
        "max_cmd_l1": round(max_cmd, 3),
        "nonzero_cmd_ratio": round(nonzero_cmd / max(1, status_count), 3),
        "override_count": override_count,
        "planner_debug_count": debug_count,
    }


def tuning_score(metrics: dict) -> float:
    goals_requested = max(1, int(metrics.get("goals_requested", 0) or 0))
    goals_reached = int(metrics.get("goals_reached", 0) or 0)
    completion = goals_reached / goals_requested
    score = 100.0 * completion
    score += 20.0 * float(metrics.get("efficiency", 0.0) or 0.0)
    score += 10.0 * float(metrics.get("nonzero_cmd_ratio", 0.0) or 0.0)
    score -= 60.0 * int(metrics.get("collisions", 0) or 0)
    score -= 16.0 * int(metrics.get("critical_near_misses", 0) or 0)
    score -= 6.0 * int(metrics.get("near_misses", 0) or 0)
    score -= 0.4 * int(metrics.get("override_count", 0) or 0)
    if metrics.get("fallen", False):
        score -= 80.0
    if int(metrics.get("collisions", 0) or 0) > 0:
        score -= 1000.0
    score -= 12.0 * max(0.0, float(metrics.get("max_tilt", 0.0) or 0.0) - 0.55)
    return round(score, 3)


def write_results(rows: list[dict], result_dir: Path) -> None:
    result_dir.mkdir(parents=True, exist_ok=True)
    ranked = sorted(rows, key=lambda row: row.get("score", -9999.0), reverse=True)
    json_path = result_dir / "human_aware_tuning_results.json"
    csv_path = result_dir / "human_aware_tuning_results.csv"
    best_args_path = result_dir / "best_human_aware_launch_args.txt"
    json_path.write_text(json.dumps(ranked, indent=2) + "\n")
    fieldnames = sorted({key for row in ranked for key in row.keys()}) if ranked else ["trial"]
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(ranked)
    best = ranked[0] if ranked else None
    if best:
        best_args = str(best.get("sim_args", "")).strip()
        best_args_path.write_text(best_args.replace(" ", "\n") + ("\n" if best_args else ""))
        print(f"\nBest trial: {best['name']} score={best['score']} goals={best.get('goals_reached')}/{best.get('goals_requested')}")
        print(f"Best launch args: {best_args_path}")
    print(f"Results: {csv_path}")
    print(f"Results: {json_path}")


def main() -> int:
    args = parse_args()
    rows = []
    for index, config in enumerate(TRIAL_CONFIGS[: max(0, args.max_trials)], start=1):
        rows.append(run_trial(config, args, index))
        write_results(rows, args.result_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
