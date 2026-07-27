import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from unitree_hg.msg import LowState


class G1LocomotionDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("g1_locomotion_diagnostics")
        self.declare_parameter("raw_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("human_aware_cmd_vel_topic", "/cmd_vel_human_aware")
        self.declare_parameter("safe_cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("body_command_topic", "/g1/body_position_commands")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("diagnostics_topic", "/g1/locomotion_diagnostics")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("min_motion_speed", 0.02)

        self.raw_cmd = TimedValue()
        self.human_aware_cmd = TimedValue()
        self.safe_cmd = TimedValue()
        self.body_cmd = TimedValue()
        self.odom = TimedValue()
        self.lowstate = TimedValue()
        self.start_pose: Optional[tuple[float, float]] = None
        self.last_pose: Optional[tuple[float, float]] = None
        self.max_tilt = 0.0
        self.nonzero_safe_samples = 0
        self.samples = 0

        self.pub = self.create_publisher(String, str(self.get_parameter("diagnostics_topic").value), 10)
        self.create_subscription(Twist, str(self.get_parameter("raw_cmd_vel_topic").value), lambda msg: self._twist_cb(msg, self.raw_cmd), 10)
        self.create_subscription(Twist, str(self.get_parameter("human_aware_cmd_vel_topic").value), lambda msg: self._twist_cb(msg, self.human_aware_cmd), 10)
        self.create_subscription(Twist, str(self.get_parameter("safe_cmd_vel_topic").value), lambda msg: self._twist_cb(msg, self.safe_cmd), 10)
        self.create_subscription(Float64MultiArray, str(self.get_parameter("body_command_topic").value), self._body_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
        self.create_subscription(LowState, str(self.get_parameter("lowstate_topic").value), self._lowstate_cb, 10)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter("rate_hz").value)), self._publish)
        self.get_logger().info("G1 locomotion diagnostics ready")

    def _twist_cb(self, msg: Twist, value: "TimedValue") -> None:
        value.stamp = self.get_clock().now().nanoseconds * 1e-9
        value.count += 1
        value.value = {
            "x": float(msg.linear.x),
            "y": float(msg.linear.y),
            "yaw": float(msg.angular.z),
            "l1": abs(float(msg.linear.x)) + abs(float(msg.linear.y)) + abs(float(msg.angular.z)),
        }

    def _body_cb(self, msg: Float64MultiArray) -> None:
        self.body_cmd.stamp = self.get_clock().now().nanoseconds * 1e-9
        self.body_cmd.count += 1
        self.body_cmd.value = {"joints": len(msg.data)}

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom.stamp = self.get_clock().now().nanoseconds * 1e-9
        self.odom.count += 1
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.last_pose = (x, y)
        if self.start_pose is None:
            self.start_pose = self.last_pose
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)
        self.odom.value = {"x": x, "y": y, "speed": math.hypot(vx, vy), "yaw_rate": wz}

    def _lowstate_cb(self, msg: LowState) -> None:
        self.lowstate.stamp = self.get_clock().now().nanoseconds * 1e-9
        self.lowstate.count += 1
        roll = 0.0
        pitch = 0.0
        if len(msg.imu_state.rpy) >= 2:
            roll = self._wrap_pi(float(msg.imu_state.rpy[0]))
            pitch = self._wrap_pi(float(msg.imu_state.rpy[1]))
        tilt = max(abs(roll), abs(pitch))
        self.max_tilt = max(self.max_tilt, tilt)
        self.lowstate.value = {"tilt": tilt, "roll": roll, "pitch": pitch}

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.samples += 1
        safe_l1 = float((self.safe_cmd.value or {}).get("l1", 0.0))
        if safe_l1 > 0.03:
            self.nonzero_safe_samples += 1
        displacement = 0.0
        if self.start_pose is not None and self.last_pose is not None:
            displacement = math.hypot(self.last_pose[0] - self.start_pose[0], self.last_pose[1] - self.start_pose[1])
        odom_speed = float((self.odom.value or {}).get("speed", 0.0))
        status = self._status(now, displacement, odom_speed)
        payload = {
            "stamp": now,
            "status": status,
            "raw_cmd": self.raw_cmd.as_dict(now),
            "human_aware_cmd": self.human_aware_cmd.as_dict(now),
            "safe_cmd": self.safe_cmd.as_dict(now),
            "body_cmd": self.body_cmd.as_dict(now),
            "odom": self.odom.as_dict(now),
            "lowstate": self.lowstate.as_dict(now),
            "displacement_m": round(displacement, 4),
            "max_tilt_rad": round(self.max_tilt, 4),
            "nonzero_safe_cmd_ratio": round(self.nonzero_safe_samples / max(1, self.samples), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.pub.publish(msg)
        self.get_logger().info(
            f"locomotion status={status} disp={displacement:.3f}m speed={odom_speed:.3f}m/s "
            f"raw_age={self.raw_cmd.age(now):.2f}s safe_l1={safe_l1:.2f} body_age={self.body_cmd.age(now):.2f}s "
            f"tilt={float((self.lowstate.value or {}).get('tilt', 0.0)):.2f}",
            throttle_duration_sec=2.0,
        )

    def _status(self, now: float, displacement: float, odom_speed: float) -> str:
        stale = float(self.get_parameter("stale_timeout").value)
        min_motion = float(self.get_parameter("min_motion_speed").value)
        if self.lowstate.age(now) > stale:
            return "NO_LOWSTATE"
        if self.raw_cmd.age(now) > stale:
            return "WAITING_FOR_RAW_CMD"
        if self.safe_cmd.age(now) > stale:
            return "NO_SAFE_CMD"
        if self.body_cmd.age(now) > stale:
            return "NO_POLICY_BODY_COMMAND"
        safe_l1 = float((self.safe_cmd.value or {}).get("l1", 0.0))
        raw_l1 = float((self.raw_cmd.value or {}).get("l1", 0.0))
        if raw_l1 > 0.03 and safe_l1 <= 0.03:
            return "SAFETY_FILTER_STOPPING"
        if safe_l1 > 0.03 and odom_speed < min_motion and displacement < 0.05:
            return "COMMAND_SENT_BUT_NO_ODOM_MOTION"
        return "MOVING_OR_READY"

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))


class TimedValue:
    def __init__(self) -> None:
        self.stamp = -math.inf
        self.count = 0
        self.value = None

    def age(self, now: float) -> float:
        return now - self.stamp if math.isfinite(self.stamp) else math.inf

    def as_dict(self, now: float) -> dict:
        return {
            "age_s": round(self.age(now), 3) if math.isfinite(self.age(now)) else None,
            "count": self.count,
            "value": self.value,
        }


def main() -> None:
    rclpy.init()
    node = G1LocomotionDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
