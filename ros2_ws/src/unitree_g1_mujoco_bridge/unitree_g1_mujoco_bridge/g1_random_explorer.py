import math
import random

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class G1RandomExplorer(Node):
    def __init__(self) -> None:
        super().__init__("g1_random_explorer")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("startup_delay", 4.0)
        self.declare_parameter("max_linear", 0.28)
        self.declare_parameter("max_angular", 0.7)
        self.declare_parameter("min_command_time", 2.5)
        self.declare_parameter("max_command_time", 6.0)
        self.declare_parameter("front_clearance", 1.25)
        self.declare_parameter("side_clearance", 0.8)
        self.declare_parameter("slow_clearance", 2.0)
        self.declare_parameter("reverse_time", 1.0)
        self.declare_parameter("seed", 7)

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.publisher = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_callback, 10)
        self.started_at = self.get_clock().now()
        self.command_until = self.started_at
        self.reverse_until = None
        self.linear = 0.0
        self.angular = 0.0
        self.front_range = math.inf
        self.left_range = math.inf
        self.right_range = math.inf
        self.scan_count = 0
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._tick)
        self.create_timer(2.0, self._diagnostics)
        self.get_logger().info("Random exploration controller ready; publishing /cmd_vel")

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan_count += 1
        self.front_range = self._sector_min(msg, -0.35, 0.35)
        self.left_range = self._sector_min(msg, 0.35, 1.25)
        self.right_range = self._sector_min(msg, -1.25, -0.35)

    @staticmethod
    def _valid_range(value: float, msg: LaserScan) -> bool:
        return math.isfinite(value) and msg.range_min <= value <= msg.range_max

    def _sector_min(self, msg: LaserScan, angle_min: float, angle_max: float) -> float:
        best = math.inf
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if angle_min <= angle <= angle_max and self._valid_range(value, msg):
                best = min(best, float(value))
        return best

    def _tick(self) -> None:
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < float(self.get_parameter("startup_delay").value):
            self._publish(0.0, 0.0)
            return

        front_clearance = float(self.get_parameter("front_clearance").value)
        side_clearance = float(self.get_parameter("side_clearance").value)
        if self.front_range < front_clearance:
            self.reverse_until = now + Duration(seconds=float(self.get_parameter("reverse_time").value))
            turn_left = self.left_range >= self.right_range
            self.linear = -0.08
            self.angular = 0.45 if turn_left else -0.45
        elif self.reverse_until is not None and (self.reverse_until - now).nanoseconds > 0:
            pass
        elif self.left_range < side_clearance:
            self.linear = 0.10
            self.angular = -0.35
        elif self.right_range < side_clearance:
            self.linear = 0.10
            self.angular = 0.35
        elif (self.command_until - now).nanoseconds <= 0:
            self._choose_new_command(now)

        linear = self.linear
        slow_clearance = float(self.get_parameter("slow_clearance").value)
        if self.front_range < slow_clearance and linear > 0.0:
            linear *= max(0.25, (self.front_range - front_clearance) / max(0.1, slow_clearance - front_clearance))
        self._publish(linear, self.angular)

    def _choose_new_command(self, now) -> None:
        max_linear = float(self.get_parameter("max_linear").value)
        max_angular = float(self.get_parameter("max_angular").value)
        min_time = float(self.get_parameter("min_command_time").value)
        max_time = float(self.get_parameter("max_command_time").value)
        if self.rng.random() < 0.72:
            self.linear = self.rng.uniform(0.12, max_linear)
            self.angular = self.rng.uniform(-0.22, 0.22)
        else:
            self.linear = self.rng.uniform(0.0, 0.12)
            self.angular = self.rng.choice((-1.0, 1.0)) * self.rng.uniform(0.22, max_angular)
        self.command_until = now + Duration(seconds=self.rng.uniform(min_time, max_time))

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def _diagnostics(self) -> None:
        self.get_logger().info(
            "random explorer: "
            f"cmd=({self.linear:+.2f}, {self.angular:+.2f}) "
            f"front={self.front_range:.2f} left={self.left_range:.2f} right={self.right_range:.2f} scans={self.scan_count}",
            throttle_duration_sec=2.0,
        )


def main() -> None:
    rclpy.init()
    node = G1RandomExplorer()
    try:
        rclpy.spin(node)
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
