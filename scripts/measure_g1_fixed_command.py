#!/usr/bin/env python3
"""Measure G1 ground-truth motion under a fixed velocity command."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


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
    parser.add_argument("--samples-output", type=Path)
    parser.add_argument("--trial-id", default="fixed_command")
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--diagnostics-timeout", type=float, default=1.0)
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
        self.create_subscription(
            TwistStamped, "/g1/policy_cmd_vel", self._policy_command_callback, 20
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(Float32, "/g1/base_height", self._base_height_callback, 20)
        self.create_subscription(
            String,
            "/g1/locomotion_diagnostics",
            self._diagnostics_callback,
            10,
        )
        self.pose: tuple[float, float] | None = None
        self.yaw: float | None = None
        self.velocity = (0.0, 0.0, 0.0)
        self.derived_velocity = (0.0, 0.0, 0.0)
        self._odom_history: deque[tuple[float, float, float, float]] = deque()
        self.max_tilt = 0.0
        self.fallen = False
        self.lowstate_ready = False
        self.last_odom_arrival: float | None = None
        self.odom_count = 0
        self.last_diagnostics_arrival: float | None = None
        self.phase = "startup"
        self.current_command = (0.0, 0.0, 0.0)
        self.policy_command = (0.0, 0.0, 0.0)
        self.policy_command_start_source_time_s = 0.0
        self.policy_stop_source_time_s = 0.0
        self.last_policy_command_arrival: float | None = None
        self.base_height_m: float | None = None
        self.last_base_height_arrival: float | None = None
        self.sample_origin = time.monotonic()
        self.sample_handle = None
        self.sample_writer = None
        self.trial_id = "fixed_command"

    def start_sample_log(self, path: Path, trial_id: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.sample_handle = path.open("x", newline="", encoding="utf-8")
        self.sample_writer = csv.DictWriter(
            self.sample_handle,
            fieldnames=[
                "trial_id", "elapsed_s", "source_time_s", "arrival_monotonic_s",
                "phase", "command_linear_x_mps", "command_linear_y_mps",
                "command_angular_z_radps", "policy_linear_x_mps",
                "policy_linear_y_mps", "policy_angular_z_radps",
                "policy_command_start_source_time_s", "policy_stop_source_time_s",
                "x_m", "y_m", "z_m", "base_height_m", "yaw_rad",
                "reported_vx_mps", "reported_vy_mps", "reported_wz_radps",
                "derived_vx_mps", "derived_vy_mps", "derived_wz_radps",
                "max_tilt_rad", "fallen",
            ],
        )
        self.sample_writer.writeheader()
        self.sample_origin = time.monotonic()
        self.trial_id = trial_id

    def _odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        self.last_odom_arrival = now
        self.odom_count += 1
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
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
                self.derived_velocity = (
                    (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt,
                    (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt,
                    math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)) / dt,
                )
        reported = msg.twist.twist
        reported_velocity = (
            float(reported.linear.x),
            float(reported.linear.y),
            float(reported.angular.z),
        )
        if all(math.isfinite(value) for value in reported_velocity):
            self.velocity = reported_velocity
        if self.sample_writer is not None:
            source_time = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
            self.sample_writer.writerow({
                "trial_id": self.trial_id,
                "elapsed_s": now - self.sample_origin,
                "source_time_s": source_time,
                "arrival_monotonic_s": now,
                "phase": self.phase,
                "command_linear_x_mps": self.current_command[0],
                "command_linear_y_mps": self.current_command[1],
                "command_angular_z_radps": self.current_command[2],
                "policy_linear_x_mps": self.policy_command[0],
                "policy_linear_y_mps": self.policy_command[1],
                "policy_angular_z_radps": self.policy_command[2],
                "policy_command_start_source_time_s": self.policy_command_start_source_time_s,
                "policy_stop_source_time_s": self.policy_stop_source_time_s,
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "base_height_m": self.base_height_m,
                "yaw_rad": yaw,
                "reported_vx_mps": self.velocity[0],
                "reported_vy_mps": self.velocity[1],
                "reported_wz_radps": self.velocity[2],
                "derived_vx_mps": self.derived_velocity[0],
                "derived_vy_mps": self.derived_velocity[1],
                "derived_wz_radps": self.derived_velocity[2],
                "max_tilt_rad": self.max_tilt,
                "fallen": self.fallen,
            })

    def _diagnostics_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_diagnostics_arrival = time.monotonic()
        lowstate = (data.get("lowstate") or {}).get("value") or {}
        self.lowstate_ready = int((data.get("lowstate") or {}).get("count", 0) or 0) > 0
        tilt = float(lowstate.get("tilt", 0.0) or 0.0)
        self.max_tilt = max(self.max_tilt, tilt, float(data.get("max_tilt_rad", 0.0) or 0.0))
        self.fallen |= self.max_tilt >= 0.95

    def _base_height_callback(self, msg: Float32) -> None:
        value = float(msg.data)
        if math.isfinite(value) and value > 0.0:
            self.base_height_m = value
            self.last_base_height_arrival = time.monotonic()

    def _policy_command_callback(self, msg: TwistStamped) -> None:
        observed = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.angular.z),
        )
        if not all(math.isfinite(value) for value in observed):
            return
        self.last_policy_command_arrival = time.monotonic()
        stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        was_zero = all(abs(value) <= 1e-9 for value in self.policy_command)
        is_zero = all(abs(value) <= 1e-9 for value in observed)
        if was_zero and not is_zero:
            self.policy_command_start_source_time_s = stamp
        elif not was_zero and is_zero:
            self.policy_stop_source_time_s = stamp
        self.policy_command = observed

    def publish_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self.current_command = (linear_x, linear_y, angular_z)
        command = Twist()
        command.linear.x = linear_x
        command.linear.y = linear_y
        command.angular.z = angular_z
        self.command_pub.publish(command)

    def streams_are_fresh(self, odom_timeout: float, diagnostics_timeout: float) -> bool:
        now = time.monotonic()
        return (
            self.last_odom_arrival is not None
            and now - self.last_odom_arrival <= odom_timeout
            and self.last_diagnostics_arrival is not None
            and now - self.last_diagnostics_arrival <= diagnostics_timeout
            and self.last_policy_command_arrival is not None
            and now - self.last_policy_command_arrival <= odom_timeout
            and self.last_base_height_arrival is not None
            and now - self.last_base_height_arrival <= odom_timeout
        )

    def close_sample_log(self) -> None:
        if self.sample_handle is not None:
            self.sample_handle.flush()
            self.sample_handle.close()
            self.sample_handle = None
            self.sample_writer = None


def run_phase(
    node: FixedCommandMeasurement,
    duration: float,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    phase: str,
    odom_timeout: float,
    diagnostics_timeout: float,
    sample: Callable[[], None] | None = None,
) -> None:
    node.phase = phase
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_command(linear_x, linear_y, angular_z)
        rclpy.spin_once(node, timeout_sec=0.02)
        if not node.streams_are_fresh(odom_timeout, diagnostics_timeout):
            raise RuntimeError(f"Stale calibration input during {phase}")
        if sample is not None:
            sample()


def first_crossing(samples: list[tuple[float, float]], threshold: float) -> float | None:
    for elapsed, value in samples:
        if value >= threshold:
            return elapsed
    return None


def first_sustained_crossing(
    samples: list[tuple[float, float]], threshold: float, hold_time: float = 0.2
) -> float | None:
    for index, (elapsed, value) in enumerate(samples):
        if value < threshold:
            continue
        window_end = elapsed + hold_time
        end_index = next(
            (candidate for candidate in range(index, len(samples)) if samples[candidate][0] + 1e-9 >= window_end),
            None,
        )
        window = samples[index:end_index + 1] if end_index is not None else []
        if window and all(
            sample_value >= threshold for _, sample_value in window
        ):
            return elapsed
    return None


def validate_args(args: argparse.Namespace) -> None:
    finite = (args.linear_x, args.linear_y, args.angular_z)
    if not all(math.isfinite(value) for value in finite):
        raise ValueError("command values must be finite")
    active = [index for index, value in enumerate(finite) if abs(value) > 1e-9]
    if len(active) != 1:
        raise ValueError("exactly one command component must be nonzero")
    limits = (
        getattr(args, "max_linear_x", None),
        getattr(args, "max_linear_y", None),
        getattr(args, "max_angular_z", None),
    )
    index = active[0]
    if limits[index] is not None and abs(finite[index]) > limits[index] + 1e-9:
        raise ValueError("command exceeds the configured policy limit")
    for name in ("duration", "settle", "stop_duration", "timeout", "odom_timeout", "diagnostics_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")


def settling_time(
    samples: list[tuple[float, float]], threshold: float, hold_time: float = 0.5
) -> float | None:
    """Return the first time speed remains below threshold for a sustained window."""
    for index, (elapsed, value) in enumerate(samples):
        if value > threshold:
            continue
        window_end = elapsed + hold_time
        end_index = next(
            (candidate for candidate in range(index, len(samples)) if samples[candidate][0] + 1e-9 >= window_end),
            None,
        )
        window = samples[index:end_index + 1] if end_index is not None else []
        if window and all(
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
    validate_args(args)
    rclpy.init()
    node = FixedCommandMeasurement()
    try:
        if args.samples_output is not None:
            node.start_sample_log(args.samples_output, args.trial_id)
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

        settle_samples: list[tuple[float, float, float]] = []
        last_settle_odom_count = node.odom_count

        def sample_settle() -> None:
            nonlocal last_settle_odom_count
            if node.odom_count == last_settle_odom_count:
                return
            last_settle_odom_count = node.odom_count
            vx, vy, wz = node.derived_velocity
            settle_samples.append((math.hypot(vx, vy), abs(wz), node.max_tilt))

        run_phase(
            node, args.settle, 0.0, 0.0, 0.0, "settle",
            args.odom_timeout, args.diagnostics_timeout, sample_settle,
        )
        if not settle_samples:
            raise RuntimeError("No odometry samples during settle gate")
        if any(linear > 0.03 or yaw > 0.03 for linear, yaw, _ in settle_samples):
            raise RuntimeError("Robot did not remain stationary during settle gate")
        if any(tilt > 0.35 for _, _, tilt in settle_samples):
            raise RuntimeError("Robot exceeded the settle tilt limit")
        start = node.pose
        start_yaw = node.yaw
        if start is None or start_yaw is None:
            raise RuntimeError("Lost /odom before command phase")
        previous = start
        previous_yaw = start_yaw
        path_length = 0.0
        yaw_change = 0.0
        command_samples: list[tuple[float, float]] = []
        last_command_odom_count = node.odom_count
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
            nonlocal previous, previous_yaw, path_length, yaw_change, last_command_odom_count
            if node.odom_count == last_command_odom_count:
                return
            last_command_odom_count = node.odom_count
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
                (
                    time.monotonic() - command_started,
                    response_sign * node.derived_velocity[response_index],
                )
            )

        started = command_started
        run_phase(
            node, args.duration, args.linear_x, args.linear_y, args.angular_z,
            "command", args.odom_timeout, args.diagnostics_timeout, sample,
        )
        elapsed = time.monotonic() - started
        end = node.pose or previous
        end_yaw = node.yaw if node.yaw is not None else previous_yaw
        stop_start = end
        stop_path_length = 0.0
        stop_previous = stop_start
        stop_samples: list[tuple[float, float]] = []
        last_stop_odom_count = node.odom_count
        stop_started = time.monotonic()

        def sample_stop() -> None:
            nonlocal stop_path_length, stop_previous, last_stop_odom_count
            if node.odom_count == last_stop_odom_count:
                return
            last_stop_odom_count = node.odom_count
            if node.pose is not None:
                stop_path_length += math.dist(stop_previous, node.pose)
                stop_previous = node.pose
            stop_samples.append(
                (
                    time.monotonic() - stop_started,
                    abs(response_sign * node.derived_velocity[response_index]),
                )
            )

        run_phase(
            node, args.stop_duration, 0.0, 0.0, 0.0, "stop",
            args.odom_timeout, args.diagnostics_timeout, sample_stop,
        )
        stop_end = node.pose or stop_previous
        final_stop_samples = [
            value
            for sample_time, value in stop_samples
            if sample_time >= max(0.0, args.stop_duration - 1.0)
        ]
        target_magnitude = abs(target_response)
        steady_samples = [
            value for sample_time, value in command_samples
            if sample_time >= max(0.0, elapsed - 2.0)
        ]
        steady_response = statistics.median(steady_samples) if steady_samples else 0.0
        peak_response = percentile([value for _, value in command_samples], 0.95)
        onset_threshold = 0.03

        world_dx = end[0] - start[0]
        world_dy = end[1] - start[1]
        forward_displacement = math.cos(start_yaw) * world_dx + math.sin(start_yaw) * world_dy
        lateral_displacement = -math.sin(start_yaw) * world_dx + math.cos(start_yaw) * world_dy
        result = {
            "schema_version": 1,
            "metric_role": "online_diagnostic_only",
            "trial_id": args.trial_id,
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
            "forward_displacement_m": forward_displacement,
            "lateral_displacement_m": lateral_displacement,
            "start_yaw_rad": start_yaw,
            "end_yaw_rad": end_yaw,
            "accumulated_yaw_change_rad": yaw_change,
            "average_yaw_rate_radps": yaw_change / max(elapsed, 1e-6),
            "path_length_m": path_length,
            "average_path_speed_mps": path_length / max(elapsed, 1e-6),
            "reactivity": {
                "response_axis": ("linear_x", "linear_y", "angular_z")[response_index],
                "motion_onset_threshold": onset_threshold,
                "motion_onset_latency_s": first_sustained_crossing(command_samples, onset_threshold),
                "rise_time_50_s": first_crossing(command_samples, 0.5 * target_magnitude),
                "rise_time_90_s": first_sustained_crossing(
                    command_samples, 0.9 * target_magnitude
                ),
                "steady_state_response": steady_response,
                "steady_state_error": abs(target_magnitude - steady_response),
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
            "raw_samples": str(args.samples_output) if args.samples_output else None,
        }
        node.close_sample_log()
        result["raw_samples_sha256"] = file_sha256(args.samples_output)
        text = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text, encoding="utf-8")
        print(text, end="")
        return 1 if node.fallen else 0
    finally:
        node.phase = "cleanup_stop"
        for _ in range(10):
            node.publish_command(0.0, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.close_sample_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
