#!/usr/bin/env python3
"""Measure G1 ground-truth motion under a fixed velocity command."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--linear-x", type=float, default=0.10)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--stop-duration", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--policy-path", type=Path)
    parser.add_argument("--policy-profile", default="unknown")
    parser.add_argument("--scene", default="unknown")
    parser.add_argument("--max-linear-x", type=float)
    parser.add_argument("--max-linear-y", type=float)
    parser.add_argument("--max-angular-z", type=float)
    parser.add_argument("--cmd-accel-x", type=float)
    parser.add_argument("--cmd-accel-y", type=float)
    parser.add_argument("--cmd-accel-yaw", type=float)
    return parser.parse_args()


def file_sha256(path: Path | None) -> str | None:
    if path is None or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class FixedCommandMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("g1_fixed_command_measurement")
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(
            String,
            "/g1/locomotion_diagnostics",
            self._diagnostics_callback,
            10,
        )
        self.pose: tuple[float, float] | None = None
        self.yaw: float | None = None
        self.velocity = (0.0, 0.0, 0.0)
        self._odom_history: deque[tuple[float, float, float, float]] = deque()
        self.max_tilt = 0.0
        self.fallen = False
        self.lowstate_ready = False

    def _odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.pose = (x, y)
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        self.yaw = yaw
        self._odom_history.append((now, x, y, yaw))
        while len(self._odom_history) > 2 and now - self._odom_history[1][0] >= 0.10:
            self._odom_history.popleft()
        if len(self._odom_history) > 1:
            last_time, last_x, last_y, last_yaw = self._odom_history[0]
            dt = now - last_time
            if dt >= 0.05:
                dx = x - last_x
                dy = y - last_y
                self.velocity = (
                    (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt,
                    (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt,
                    math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)) / dt,
                )

    def _diagnostics_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        lowstate = (data.get("lowstate") or {}).get("value") or {}
        self.lowstate_ready = int((data.get("lowstate") or {}).get("count", 0) or 0) > 0
        tilt = float(lowstate.get("tilt", 0.0) or 0.0)
        self.max_tilt = max(self.max_tilt, tilt, float(data.get("max_tilt_rad", 0.0) or 0.0))
        self.fallen |= self.max_tilt >= 0.95

    def publish_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.linear.y = linear_y
        command.angular.z = angular_z
        self.command_pub.publish(command)


def run_phase(
    node: FixedCommandMeasurement,
    duration: float,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    sample: Callable[[], None] | None = None,
) -> None:
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_command(linear_x, linear_y, angular_z)
        rclpy.spin_once(node, timeout_sec=0.02)
        if sample is not None:
            sample()


def first_crossing(samples: list[tuple[float, float]], threshold: float) -> float | None:
    for elapsed, value in samples:
        if value >= threshold:
            return elapsed
    return None


def settling_time(
    samples: list[tuple[float, float]], threshold: float, hold_time: float = 0.5
) -> float | None:
    """Return the first time speed remains below threshold for a sustained window."""
    for index, (elapsed, value) in enumerate(samples):
        if value > threshold:
            continue
        window_end = elapsed + hold_time
        window = [sample for sample in samples[index:] if sample[0] <= window_end]
        if window and window[-1][0] >= window_end and all(
            sample_value <= threshold for _, sample_value in window
        ):
            return elapsed
    return None


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * quantile)]


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FixedCommandMeasurement()
    try:
        deadline = time.monotonic() + args.timeout
        while (
            rclpy.ok()
            and (node.pose is None or not node.lowstate_ready)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose is None:
            raise RuntimeError("Timed out waiting for /odom")
        if not node.lowstate_ready:
            raise RuntimeError("Timed out waiting for /lowstate")

        run_phase(node, args.settle, 0.0, 0.0, 0.0)
        start = node.pose
        start_yaw = node.yaw
        if start is None or start_yaw is None:
            raise RuntimeError("Lost /odom before command phase")
        previous = start
        previous_yaw = start_yaw
        path_length = 0.0
        yaw_change = 0.0
        command_samples: list[tuple[float, float]] = []
        command_started = time.monotonic()
        if abs(args.angular_z) > 0.0 and abs(args.linear_x) < 1e-9 and abs(args.linear_y) < 1e-9:
            response_index = 2
            target_response = args.angular_z
        elif abs(args.linear_y) > 0.0 and abs(args.linear_x) < 1e-9:
            response_index = 1
            target_response = args.linear_y
        else:
            response_index = 0
            target_response = args.linear_x
        response_sign = -1.0 if target_response < 0.0 else 1.0

        def sample() -> None:
            nonlocal previous, previous_yaw, path_length, yaw_change
            if node.pose is None or node.yaw is None:
                return
            path_length += math.dist(previous, node.pose)
            previous = node.pose
            yaw_delta = math.atan2(
                math.sin(node.yaw - previous_yaw),
                math.cos(node.yaw - previous_yaw),
            )
            yaw_change += yaw_delta
            previous_yaw = node.yaw
            command_samples.append(
                (time.monotonic() - command_started, response_sign * node.velocity[response_index])
            )

        started = command_started
        run_phase(node, args.duration, args.linear_x, args.linear_y, args.angular_z, sample)
        elapsed = time.monotonic() - started
        end = node.pose or previous
        end_yaw = node.yaw if node.yaw is not None else previous_yaw
        stop_start = end
        stop_path_length = 0.0
        stop_previous = stop_start
        stop_samples: list[tuple[float, float]] = []
        stop_started = time.monotonic()

        def sample_stop() -> None:
            nonlocal stop_path_length, stop_previous
            if node.pose is not None:
                stop_path_length += math.dist(stop_previous, node.pose)
                stop_previous = node.pose
            vx, vy, wz = node.velocity
            stop_samples.append(
                (time.monotonic() - stop_started, max(math.hypot(vx, vy), abs(wz)))
            )

        run_phase(node, args.stop_duration, 0.0, 0.0, 0.0, sample_stop)
        stop_end = node.pose or stop_previous
        final_stop_samples = [
            value
            for sample_time, value in stop_samples
            if sample_time >= max(0.0, args.stop_duration - 1.0)
        ]
        target_magnitude = abs(target_response)
        steady_samples = [
            value for sample_time, value in command_samples if sample_time >= 0.8 * elapsed
        ]
        steady_response = (
            sum(steady_samples) / len(steady_samples) if steady_samples else 0.0
        )
        peak_response = percentile([value for _, value in command_samples], 0.95)
        onset_threshold = min(0.03, 0.25 * target_magnitude)

        result = {
            "provenance": {
                "policy_path": str(args.policy_path) if args.policy_path else None,
                "policy_sha256": file_sha256(args.policy_path),
                "policy_profile": args.policy_profile,
                "scene": args.scene,
                "limits": {
                    "max_linear_x": args.max_linear_x,
                    "max_linear_y": args.max_linear_y,
                    "max_angular_z": args.max_angular_z,
                },
                "acceleration_limits": {
                    "linear_x": args.cmd_accel_x,
                    "linear_y": args.cmd_accel_y,
                    "angular_z": args.cmd_accel_yaw,
                },
            },
            "command": {
                "linear_x": args.linear_x,
                "linear_y": args.linear_y,
                "angular_z": args.angular_z,
            },
            "duration_s": elapsed,
            "start_xy": start,
            "end_xy": end,
            "net_displacement_m": math.dist(start, end),
            "forward_displacement_m": end[0] - start[0],
            "lateral_displacement_m": end[1] - start[1],
            "start_yaw_rad": start_yaw,
            "end_yaw_rad": end_yaw,
            "accumulated_yaw_change_rad": yaw_change,
            "average_yaw_rate_radps": yaw_change / max(elapsed, 1e-6),
            "path_length_m": path_length,
            "average_path_speed_mps": path_length / max(elapsed, 1e-6),
            "reactivity": {
                "response_axis": ("linear_x", "linear_y", "angular_z")[response_index],
                "motion_onset_threshold": onset_threshold,
                "motion_onset_latency_s": first_crossing(command_samples, onset_threshold),
                "rise_time_50_s": first_crossing(command_samples, 0.5 * target_magnitude),
                "rise_time_90_s": first_crossing(command_samples, 0.9 * target_magnitude),
                "steady_state_response": steady_response,
                "steady_state_error": target_magnitude - steady_response,
                "peak_response_p95": peak_response,
                "overshoot_fraction": (
                    max(0.0, peak_response - target_magnitude) / target_magnitude
                    if target_magnitude > 0.0
                    else 0.0
                ),
                "stop_settling_threshold": 0.03,
                "stop_settling_hold_time_s": 0.5,
                "stop_settling_time_s": settling_time(stop_samples, 0.03),
                "stop_duration_s": args.stop_duration,
                "final_stop_speed_p95": percentile(final_stop_samples, 0.95),
                "post_stop_net_drift_m": math.dist(stop_start, stop_end),
                "post_stop_path_length_m": stop_path_length,
            },
            "max_tilt_rad": node.max_tilt,
            "fallen": node.fallen,
        }
        text = json.dumps(result, indent=2) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text, encoding="utf-8")
        print(text, end="")
        return 1 if node.fallen else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
