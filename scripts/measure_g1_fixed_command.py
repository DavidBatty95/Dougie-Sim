#!/usr/bin/env python3
"""Measure G1 ground-truth motion under a fixed velocity command."""

from __future__ import annotations

import argparse
import json
import math
import time
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
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


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
        self.max_tilt = 0.0
        self.fallen = False

    def _odom_callback(self, msg: Odometry) -> None:
        self.pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        orientation = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    def _diagnostics_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        lowstate = (data.get("lowstate") or {}).get("value") or {}
        tilt = float(lowstate.get("tilt", 0.0) or 0.0)
        self.max_tilt = max(self.max_tilt, tilt, float(data.get("max_tilt_rad", 0.0) or 0.0))
        self.fallen |= self.max_tilt >= 0.95

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.command_pub.publish(command)


def run_phase(
    node: FixedCommandMeasurement,
    duration: float,
    linear_x: float,
    angular_z: float,
    sample: Callable[[], None] | None = None,
) -> None:
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_command(linear_x, angular_z)
        rclpy.spin_once(node, timeout_sec=0.02)
        if sample is not None:
            sample()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FixedCommandMeasurement()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose is None:
            raise RuntimeError("Timed out waiting for /odom")

        run_phase(node, args.settle, 0.0, 0.0)
        start = node.pose
        start_yaw = node.yaw
        if start is None or start_yaw is None:
            raise RuntimeError("Lost /odom before command phase")
        previous = start
        previous_yaw = start_yaw
        path_length = 0.0
        yaw_change = 0.0

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

        started = time.monotonic()
        run_phase(node, args.duration, args.linear_x, args.angular_z, sample)
        elapsed = time.monotonic() - started
        end = node.pose or previous
        end_yaw = node.yaw if node.yaw is not None else previous_yaw
        run_phase(node, 1.0, 0.0, 0.0)

        result = {
            "command": {"linear_x": args.linear_x, "angular_z": args.angular_z},
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
