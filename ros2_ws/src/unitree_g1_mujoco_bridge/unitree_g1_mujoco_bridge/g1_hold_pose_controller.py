import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .g1_constants import G1_29DOF_JOINTS


class G1HoldPoseController(Node):
    def __init__(self) -> None:
        super().__init__("g1_hold_pose_controller")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("enable_hands", True)
        self.declare_parameter("body_command_topic", "/g1/body_position_commands")
        self.declare_parameter("body_positions", [0.0] * len(G1_29DOF_JOINTS))

        self.body_positions = [float(value) for value in self.get_parameter("body_positions").value]
        if len(self.body_positions) != len(G1_29DOF_JOINTS) or not all(
            math.isfinite(value) for value in self.body_positions
        ):
            raise ValueError("body_positions must contain 29 finite joint positions")

        self.body_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("body_command_topic").value),
            10,
        )
        self.left_hand_pub = self.create_publisher(Float64MultiArray, "/g1/left_hand_position_commands", 10)
        self.right_hand_pub = self.create_publisher(Float64MultiArray, "/g1/right_hand_position_commands", 10)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(period, self._publish_hold_pose)
        self.get_logger().info("Publishing neutral G1 hold pose commands")

    def _publish_hold_pose(self) -> None:
        body = Float64MultiArray()
        body.data = self.body_positions
        self.body_pub.publish(body)

        if bool(self.get_parameter("enable_hands").value):
            hand = Float64MultiArray()
            hand.data = [0.0] * 7
            self.left_hand_pub.publish(hand)
            self.right_hand_pub.publish(hand)


def main() -> None:
    rclpy.init()
    node = G1HoldPoseController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
