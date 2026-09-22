#!/usr/bin/env python3
"""Run a long visual session, restarting clean episodes after falls or data loss."""
import argparse
from datetime import datetime, timezone
from dataclasses import replace
import json
import math
from pathlib import Path
import shlex
import signal
import time

from demo_suite import DEFAULT_RESULTS, REPO_ROOT, case_catalog, preflight, resolve_seed, stream_process, write_json
from avoidance_worlds import WORLDS, generate


def in_world(case, world):
    if world == 'plaza':
        return case
    command = list(case.command)
    scene = f'scene_29dof_avoidance_{world}.xml'
    command[command.index('--scene')+1] = scene
    command[command.index('--environment-name')+1] = world
    return replace(case, command=tuple(command), required_scene=scene)


def should_restart(return_code, end_reason, remaining, goal_limit):
    return (return_code == 0 and goal_limit is None and remaining > 5.
            and end_reason not in ("duration_elapsed", "goal_budget_reached"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=1800., help="session budget in seconds (default: 30 minutes)")
    parser.add_argument("--seed", type=int, default=None, help="base seed; restarted episodes advance this seed")
    parser.add_argument("--goals", type=int, default=None, help="optional goal limit for one episode")
    parser.add_argument("--people", type=int, choices=range(1, 9), default=None,
                        help="independent pedestrians, 1–8 (default: 2 small rooms, 4 medium, 8 large/plaza)")
    parser.add_argument("--headless", action="store_true", help="disable RViz")
    parser.add_argument("--fast-start", action="store_true", help="reuse existing ROS and simulator builds")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--world", choices=['plaza', *WORLDS], default='plaza')
    parser.add_argument("--result-root", type=Path, default=DEFAULT_RESULTS)
    args = parser.parse_args()
    if args.people is None:
        args.people = 2 if args.world.startswith('small_') else 4 if args.world.startswith('medium_') else 8
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if args.goals is not None and args.goals < 1:
        parser.error("--goals must be positive")
    root = args.result_root.expanduser().resolve()
    if not root.is_relative_to(REPO_ROOT):
        parser.error("--result-root must be inside the repository")
    root /= "visual_session_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base_seed = resolve_seed(args.seed)
    if args.world != 'plaza':
        generate()
    first = case_catalog(root / "episode_001", base_seed, args.headless, args.fast_start,
                         visual_duration_s=args.duration, visual_goals=args.goals, onboard_people=args.people)["onboard-visual"]
    first = in_world(first, args.world)
    preflight([first], args.dry_run)
    if args.dry_run:
        print(shlex.join(first.command))
        return 0
    root.mkdir(parents=True, exist_ok=True)
    summary = {"visual_session": True, "scored": False, "duration_budget_s": args.duration,
               "base_seed": base_seed, "world": args.world, "people": args.people, "rviz_enabled": not args.headless,
               "status": "running", "episodes": []}
    started = time.monotonic()
    deadline = started + args.duration
    return_code = 0

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        while time.monotonic() < deadline:
            number = len(summary["episodes"]) + 1
            seed = (base_seed + number - 1) % (2**31)
            remaining = max(1., deadline - time.monotonic())
            episode_dir = root / f"episode_{number:03d}"
            episode_dir.mkdir(parents=True, exist_ok=True)
            case = case_catalog(episode_dir, seed, args.headless, args.fast_start or number > 1,
                visual_duration_s=remaining, visual_goals=args.goals, onboard_people=args.people)["onboard-visual"]
            case = in_world(case, args.world)
            record = {"number": number, "seed": seed, "status": "running", "command": list(case.command)}
            summary["episodes"].append(record)
            write_json(root / "session.json", summary)
            print(f"\nVisual episode {number}, seed {seed}; {remaining/60:.1f} minutes remaining", flush=True)
            with (episode_dir / "session.log").open("w", encoding="utf-8") as log:
                return_code, timed_out = stream_process(case.command, log, case.timeout_s)
            results = sorted((episode_dir / "onboard-visual").glob("person_avoidance_*.json"))
            outcome = json.loads(results[-1].read_text()) if results else {}
            reason = outcome.get("session_end_reason", "process_failed")
            record.update(status="completed" if return_code == 0 else "failed", return_code=return_code,
                          timed_out=timed_out, end_reason=reason, goals_reached=outcome.get("goals_reached"),
                          collisions=outcome.get("collisions"), result=str(results[-1]) if results else None)
            write_json(root / "session.json", summary)
            if not should_restart(return_code, reason, deadline-time.monotonic(), args.goals):
                break
            print(f"Episode ended ({reason}); restarting the simulator and viewers with a fresh sensor-only map.", flush=True)
            time.sleep(2.)
        summary["status"] = "completed" if return_code == 0 else "failed"
    except KeyboardInterrupt:
        summary["status"] = "interrupted"
        if summary["episodes"] and summary["episodes"][-1]["status"] == "running":
            summary["episodes"][-1]["status"] = "interrupted"
        return_code = 130
    finally:
        summary["wall_duration_s"] = time.monotonic() - started
        write_json(root / "session.json", summary)
        print(f"\nVisual session {summary['status']}. Artifacts: {root}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
