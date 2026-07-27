#!/usr/bin/env python3
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
DEFAULT_RESULT_DIR = REPO_ROOT / "ros2_ws" / "src" / "dynamic_obstacle_benchmarks" / "results" / "extreme_lab_tuning"


@dataclass(frozen=True)
class TrialConfig:
    name: str
    sim_args: tuple[str, ...]


TRIAL_CONFIGS = [
    TrialConfig(
        "baseline_confirmed_only",
        (),
    ),
    TrialConfig(
        "early_wide_slow",
        (
            "tracker_min_track_age:=3",
            "tracker_min_person_observations:=3",
            "avoidance_danger_radius:=2.8",
            "avoidance_critical_radius:=1.8",
            "avoidance_corridor_width:=2.6",
            "avoidance_max_reverse_linear:=0.07",
            "avoidance_max_lateral_linear:=0.10",
            "avoidance_max_avoid_angular:=0.50",
        ),
    ),
    TrialConfig(
        "conservative_confirm",
        (
            "tracker_min_track_age:=5",
            "tracker_min_person_observations:=4",
            "tracker_confirm_confidence_threshold:=0.76",
            "avoidance_danger_radius:=2.5",
            "avoidance_critical_radius:=1.6",
            "avoidance_corridor_width:=2.2",
            "avoidance_max_avoid_angular:=0.45",
        ),
    ),
    TrialConfig(
        "candidate_visual_only_wide",
        (
            "tracker_min_track_age:=3",
            "tracker_min_dynamic_observations:=2",
            "tracker_min_person_observations:=3",
            "avoidance_danger_radius:=3.0",
            "avoidance_critical_radius:=1.9",
            "avoidance_corridor_width:=2.8",
            "avoidance_people_repulsion_gain:=0.08",
        ),
    ),
    TrialConfig(
        "narrow_low_yaw",
        (
            "avoidance_danger_radius:=2.3",
            "avoidance_critical_radius:=1.5",
            "avoidance_corridor_width:=2.0",
            "avoidance_corridor_critical_width:=1.1",
            "avoidance_max_reverse_linear:=0.06",
            "avoidance_max_lateral_linear:=0.08",
            "avoidance_max_avoid_angular:=0.40",
        ),
    ),
    TrialConfig(
        "object_shape_guarded",
        (
            "tracker_use_object_motion_detection:=true",
            "tracker_immediate_person_risk:=false",
            "tracker_publish_candidate_predictions:=false",
            "tracker_candidate_person_score_threshold:=0.72",
            "tracker_confirmed_person_score_threshold:=0.70",
            "tracker_confirm_confidence_threshold:=0.78",
            "tracker_min_track_age:=5",
            "tracker_min_person_observations:=4",
            "avoidance_danger_radius:=2.6",
            "avoidance_corridor_width:=2.4",
        ),
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune Extreme Lab survival by sweeping launch parameters.")
    parser.add_argument("--duration", type=int, default=45, help="Seconds to keep each sim trial alive.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--people", type=int, default=3)
    parser.add_argument("--goals", type=int, default=10)
    parser.add_argument("--max-trials", type=int, default=len(TRIAL_CONFIGS), help="Limit number of trial configs.")
    parser.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR)
    parser.add_argument("--with-rviz", action="store_true", help="Show RViz during tuning. Off by default for speed.")
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
        "--startup-seconds",
        str(args.duration),
        "--result-dir",
        str(trial_dir),
    ]
    if not args.with_rviz:
        command.append("--no-rviz")
    for sim_arg in config.sim_args:
        command.extend(["--sim-arg", sim_arg])

    print(f"\n=== Trial {index}: {config.name} ===", flush=True)
    print(" ".join(command), flush=True)
    completed = subprocess.run(command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    sys.stdout.write(completed.stdout)
    sim_log = _sim_log_from_output(completed.stdout) or _newest_sim_log(trial_dir)
    metrics = parse_sim_log(sim_log, args.duration) if sim_log else {"survival_s": 0.0, "fallen": True}
    metrics.update(
        {
            "trial": index,
            "name": config.name,
            "returncode": completed.returncode,
            "sim_log": str(sim_log) if sim_log else "",
            "sim_args": " ".join(config.sim_args),
        }
    )
    metrics["score"] = survival_score(metrics, args.duration)
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


def parse_sim_log(path: Path, duration: int) -> dict:
    start_time = None
    end_time = None
    first_fall_time = None
    max_tilt = 0.0
    max_cmd = 0.0
    override_count = 0
    max_confirmed = 0
    max_candidates = 0
    min_risk = None
    nonzero_cmd_count = 0
    status_count = 0
    policy_status = re.compile(r"\[(\d+\.\d+)\].*policy status:.*tilt=([0-9.]+).*cmd=\(([-+0-9.]+), ([-+0-9.]+), ([-+0-9.]+)\)")
    override = re.compile(r"\[(\d+\.\d+)\].*Human collision risk.*closest predicted point ([0-9.inf]+)m")
    tracker = re.compile(r"pedestrian tracker: clusters=(\d+) candidates=(\d+) confirmed=(\d+)")
    fall = re.compile(r"\[(\d+\.\d+)\].*Robot fall detected")
    timestamp = re.compile(r"\[(\d+\.\d+)\]")

    for line in path.read_text(errors="replace").splitlines():
        time_match = timestamp.search(line)
        if time_match:
            t = float(time_match.group(1))
            start_time = t if start_time is None else min(start_time, t)
            end_time = t if end_time is None else max(end_time, t)
        match = policy_status.search(line)
        if match:
            t = float(match.group(1))
            start_time = t if start_time is None else min(start_time, t)
            end_time = t if end_time is None else max(end_time, t)
            tilt = float(match.group(2))
            cmd_mag = abs(float(match.group(3))) + abs(float(match.group(4))) + abs(float(match.group(5)))
            max_tilt = max(max_tilt, tilt)
            max_cmd = max(max_cmd, cmd_mag)
            status_count += 1
            if cmd_mag > 0.05:
                nonzero_cmd_count += 1
        match = override.search(line)
        if match:
            override_count += 1
            value = match.group(2)
            if value != "inf":
                risk = float(value)
                min_risk = risk if min_risk is None else min(min_risk, risk)
        match = tracker.search(line)
        if match:
            max_candidates = max(max_candidates, int(match.group(2)))
            max_confirmed = max(max_confirmed, int(match.group(3)))
        match = fall.search(line)
        if match and first_fall_time is None:
            first_fall_time = float(match.group(1))

    if start_time is None:
        survival = 0.0
    elif first_fall_time is not None:
        survival = max(0.0, first_fall_time - start_time)
    elif end_time is not None:
        survival = min(float(duration), max(0.0, end_time - start_time))
    else:
        survival = 0.0

    return {
        "survival_s": round(survival, 3),
        "fallen": first_fall_time is not None,
        "max_tilt": round(max_tilt, 3),
        "max_cmd_l1": round(max_cmd, 3),
        "override_count": override_count,
        "max_confirmed": max_confirmed,
        "max_candidates": max_candidates,
        "min_risk_m": round(min_risk, 3) if min_risk is not None else "",
        "nonzero_cmd_ratio": round(nonzero_cmd_count / max(1, status_count), 3),
    }


def survival_score(metrics: dict, duration: int) -> float:
    score = float(metrics.get("survival_s", 0.0))
    if not metrics.get("fallen", True) and score >= duration * 0.80:
        score += 10.0
    score -= 0.5 * max(0, int(metrics.get("max_candidates", 0)) - 2)
    score -= 0.5 * max(0, int(metrics.get("max_confirmed", 0)) - 4)
    score -= 3.0 * max(0.0, float(metrics.get("max_tilt", 0.0)) - 0.55)
    return round(score, 3)


def write_results(rows: list[dict], result_dir: Path) -> None:
    result_dir.mkdir(parents=True, exist_ok=True)
    ranked = sorted(rows, key=lambda row: row["score"], reverse=True)
    json_path = result_dir / "survival_tuning_results.json"
    csv_path = result_dir / "survival_tuning_results.csv"
    json_path.write_text(json.dumps(ranked, indent=2) + "\n")
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ranked[0].keys()) if ranked else ["trial"])
        writer.writeheader()
        writer.writerows(ranked)
    print(f"\nBest trial: {ranked[0]['name']} score={ranked[0]['score']} survival={ranked[0]['survival_s']}s" if ranked else "No trials ran")
    print(f"Results: {csv_path}")
    print(f"Results: {json_path}")


def main() -> int:
    args = parse_args()
    rows = []
    for index, config in enumerate(TRIAL_CONFIGS[: max(1, args.max_trials)], start=1):
        rows.append(run_trial(config, args, index))
    write_results(rows, args.result_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
