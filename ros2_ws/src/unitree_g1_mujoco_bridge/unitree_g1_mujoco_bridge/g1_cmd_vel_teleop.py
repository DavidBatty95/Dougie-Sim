import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Arrow key teleop for /cmd_vel

  Up/Down:    increase/decrease forward velocity
  Left/Right: increase/decrease yaw velocity
  W/S:        increase/decrease forward velocity
  A/D:        increase/decrease yaw velocity
  Space:      stop
  q/Ctrl-C:   quit
"""


class G1CmdVelTeleop(Node):
    def __init__(self) -> None:
        super().__init__("g1_cmd_vel_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.05)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 0.5)

        self.linear = 0.0
        self.angular = 0.0
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._publish)
        self._print_status()

    def handle_key(self, key: str) -> bool:
        linear_step = float(self.get_parameter("linear_step").value)
        angular_step = float(self.get_parameter("angular_step").value)
        max_linear = float(self.get_parameter("max_linear").value)
        max_angular = float(self.get_parameter("max_angular").value)

        changed = True
        if key in ("\x1b[A", "w", "W"):
            self.linear = min(max_linear, self.linear + linear_step)
        elif key in ("\x1b[B", "s", "S"):
            self.linear = max(-max_linear, self.linear - linear_step)
        elif key in ("\x1b[C", "d", "D"):
            self.angular = max(-max_angular, self.angular - angular_step)
        elif key in ("\x1b[D", "a", "A"):
            self.angular = min(max_angular, self.angular + angular_step)
        elif key == " ":
            self.linear = 0.0
            self.angular = 0.0
        elif key in ("q", "\x03"):
            self.linear = 0.0
            self.angular = 0.0
            self._publish()
            return False
        else:
            changed = False
        if changed:
            self._publish()
            self._print_status()
        return True

    def _publish(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    def _print_status(self) -> None:
        print(
            f"\rcmd_vel linear.x={self.linear:+.2f} m/s angular.z={self.angular:+.2f} rad/s",
            end="",
            flush=True,
        )


def _read_key(timeout: float = 0.05) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    first = sys.stdin.read(1)
    if first != "\x1b":
        return first
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return first
    second = sys.stdin.read(1)
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return first + second
    return first + second + sys.stdin.read(1)


def main() -> None:
    rclpy.init()
    node = G1CmdVelTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key()
            if key and not node.handle_key(key):
                break
    finally:
        node.linear = 0.0
        node.angular = 0.0
        node._publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
