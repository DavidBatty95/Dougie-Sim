from __future__ import annotations

import argparse
from pathlib import Path

try:
    import rclpy
except ModuleNotFoundError:
    rclpy = None

from .benchmark_runner import BenchmarkRunner
from .config import load_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the person avoidance benchmark.")
    parser.add_argument("--config", default=None, help="YAML benchmark config file.")
    parser.add_argument("--mode", default=None, choices=["smoke", "standard", "stress", "crossing", "bottleneck", "occlusion"], help="Benchmark mode override.")
    parser.add_argument("--backend", default=None, choices=["mock", "mujoco"], help="Simulation backend override.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed override.")
    parser.add_argument("--people", type=int, default=None, help="People count override.")
    parser.add_argument("--goals", type=int, default=None, help="Goal count override.")
    parser.add_argument("--headless", action="store_true", help="Run without visualization.")
    parser.add_argument("--record-rosbag", action="store_true", help="Reserved for MuJoCo/Nav2 backend rosbag recording.")
    parser.add_argument("--output", default=None, help="JSON output path.")
    parser.add_argument("--scenario-file", default=None, help="Reserved scenario replay file path.")
    return parser


def apply_overrides(config, args) -> None:
    if args.mode:
        config.mode = args.mode
        preset = config.mode_presets.get(args.mode)
        if preset is None:
            raise ValueError(f"mode preset not found in person_scenarios.yaml: {args.mode}")
        config.people_count = int(preset.get("people", config.people_count))
        config.goals_count = int(preset.get("goals", config.goals_count))
        config.behaviours = list(preset.get("behaviours", config.behaviours))
        config.goal_mode = str(preset.get("goal_mode", config.goal_mode))
        config.max_goal_time_s = float(preset.get("max_goal_time_s", config.max_goal_time_s))
    if args.backend:
        config.backend = args.backend
    if args.seed is not None:
        config.seed = args.seed
    if args.people is not None:
        config.people_count = args.people
    if args.goals is not None:
        config.goals_count = args.goals
    if args.headless:
        config.headless = True
    if args.record_rosbag:
        config.record_rosbag = True
    if args.output:
        config.output = args.output
    if args.scenario_file:
        scenario_path = Path(args.scenario_file)
        if not scenario_path.exists():
            raise FileNotFoundError(f"scenario file not found: {scenario_path}")
        config.scenario_file = str(scenario_path)
    config.validate()


def main() -> None:
    args = build_parser().parse_args()
    config = load_config(args.config)
    apply_overrides(config, args)
    if rclpy is not None:
        rclpy.init()
    try:
        score, result_path = BenchmarkRunner(config, args.output).run()
    finally:
        if rclpy is not None:
            rclpy.shutdown()
    print_summary(config, score, result_path)


def print_summary(config, score, result_path: str) -> None:
    import json

    data = json.loads(Path(result_path).read_text())
    print("Person Avoidance Benchmark")
    print("--------------------------")
    print(f"Seed: {config.seed}")
    print(f"Backend: {config.backend}")
    print(f"People: {data['people_count']}")
    print(f"Goals requested: {data['goals_requested']}")
    print(f"Goals reached: {data['goals_reached']}")
    print(f"Total time: {data['total_time_s']:.1f} s")
    print(f"Actual path length: {data['actual_path_length_m']:.2f} m")
    print(f"Ideal path length: {data['ideal_path_length_m']:.2f} m")
    print(f"Efficiency: {data['efficiency']:.2f}")
    print(f"Collisions: {data['collisions']}")
    print(f"Near misses: {data['near_misses']}")
    print(f"Critical near misses: {data['critical_near_misses']}")
    print(f"Minimum center distance: {data['minimum_center_distance_m']} m")
    print(f"Minimum body clearance: {data['minimum_body_clearance_m']} m")
    print(f"Minimum TTC: {data['minimum_ttc_s']} s")
    print(f"Safety score: {score.safety_score:.1f}")
    print(f"Task score: {score.task_score:.1f}")
    print(f"Speed score: {score.speed_score:.1f}")
    print(f"Efficiency score: {score.efficiency_score:.1f}")
    print(f"Final score: {score.final_score:.1f} / 100")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()
