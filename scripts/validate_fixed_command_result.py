#!/usr/bin/env python3
"""Validate a live fixed-command locomotion result."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    parser.add_argument("--max-tilt", type=float, default=0.35)
    parser.add_argument("--max-onset", type=float, default=1.5)
    parser.add_argument("--minimum-response-fraction", type=float, default=0.25)
    parser.add_argument("--max-final-stop-speed", type=float, default=0.08)
    parser.add_argument("--max-stop-drift", type=float, default=0.20)
    return parser.parse_args()


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def main() -> int:
    args = parse_args()
    if not args.result.is_file():
        raise SystemExit(f"fixed-command result does not exist: {args.result}")
    data = json.loads(args.result.read_text(encoding="utf-8"))
    command = data.get("command") or {}
    reactivity = data.get("reactivity") or {}
    provenance = data.get("provenance") or {}
    axis = reactivity.get("response_axis")
    command_key = {
        "linear_x": "linear_x",
        "linear_y": "linear_y",
        "angular_z": "angular_z",
    }.get(axis)
    target = abs(float(command.get(command_key, 0.0))) if command_key else 0.0
    steady = reactivity.get("steady_state_response")
    onset = reactivity.get("motion_onset_latency_s")
    stop_settling = reactivity.get("stop_settling_time_s")
    final_stop_speed = reactivity.get("final_stop_speed_p95")
    failures: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            failures.append(message)

    require(data.get("fallen") is False, "robot must not fall")
    components = [command.get(key,0.) for key in ("linear_x","linear_y","angular_z")]
    require(all(finite_number(value) for value in components), "command components must be finite")
    require(sum(finite_number(value) and abs(value)>1e-9 for value in components) == 1,
            "exactly one command component must be nonzero")
    require(data.get("metric_role") != "mixed_command_diagnostic_only",
            "mixed-command diagnostics are not single-axis acceptance trials")
    require(
        finite_number(data.get("max_tilt_rad"))
        and data["max_tilt_rad"] <= args.max_tilt,
        f"max tilt must be <= {args.max_tilt} rad",
    )
    require(target > 0.0, "command must select a nonzero response axis")
    require(
        finite_number(steady)
        and steady >= args.minimum_response_fraction * target,
        f"steady response must reach {args.minimum_response_fraction:.0%} of command",
    )
    require(
        finite_number(onset) and onset <= args.max_onset,
        f"motion onset must be <= {args.max_onset} s",
    )
    require(
        finite_number(final_stop_speed),
        "final stop speed p95 must be recorded",
    )
    require(
        finite_number(stop_settling)
        or (
            finite_number(final_stop_speed)
            and final_stop_speed <= args.max_final_stop_speed
        ),
        "motion must settle or final stop speed p95 must be "
        f"<= {args.max_final_stop_speed}",
    )
    require(
        finite_number(reactivity.get("post_stop_net_drift_m"))
        and reactivity["post_stop_net_drift_m"] <= args.max_stop_drift,
        f"post-stop drift must be <= {args.max_stop_drift} m",
    )
    require(bool(provenance.get("policy_sha256")), "policy SHA-256 must be recorded")
    require(bool(provenance.get("scene")), "scene provenance must be recorded")

    if failures:
        print("Fixed-command validation failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(
        "Fixed-command validation passed: "
        f"axis={axis}, onset={onset:.3f}s, steady={steady:.3f}, "
        f"stop_drift={reactivity['post_stop_net_drift_m']:.3f}m, "
        f"final_stop_p95={final_stop_speed:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
