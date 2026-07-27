import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class G1PointGoal(Node):
    def __init__(self) -> None:
        super().__init__("g1_point_goal")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("goal_topic", "/g1/point_goal")
        self.declare_parameter("goal_marker_topic", "/exploration_goal")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("startup_delay", 8.0)
        self.declare_parameter("goal_x", math.nan)
        self.declare_parameter("goal_y", math.nan)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("validate_costmap", True)
        self.declare_parameter("goal_clearance", 0.45)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("goal_timeout", 90.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_marker_topic").value), 10)
        self.costmap: Optional[OccupancyGrid] = None
        self.started_at = self.get_clock().now()
        self.active_goal = None
        self.send_goal_future = None
        self.result_future = None
        self.goal_started_at = None
        self.sent_param_goal = False

        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("costmap_topic").value), self._costmap_cb, costmap_qos)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self._goal_cb, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("G1 point-to-point goal sender ready")

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.costmap = msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._send_goal(msg)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < float(self.get_parameter("startup_delay").value):
            return
        if not self.action_client.server_is_ready():
            self.get_logger().warn("Waiting for Nav2 navigate_to_pose action server", throttle_duration_sec=5.0)
            return
        if self.result_future is not None:
            if self.result_future.done():
                result = self.result_future.result()
                status = getattr(result, "status", "unknown")
                self.get_logger().info(f"Point goal finished with status={status}")
                self.result_future = None
                self.active_goal = None
                self.goal_started_at = None
            elif self.goal_started_at is not None:
                elapsed = (now - self.goal_started_at).nanoseconds * 1e-9
                if elapsed > float(self.get_parameter("goal_timeout").value):
                    self.get_logger().warn("Point goal timed out; cancelling")
                    self.active_goal.cancel_goal_async()
                    self.result_future = None
                    self.active_goal = None
                    self.goal_started_at = None
            return
        if self.send_goal_future is not None:
            if self.send_goal_future.done():
                self.active_goal = self.send_goal_future.result()
                self.send_goal_future = None
                if not self.active_goal.accepted:
                    self.get_logger().warn("Nav2 rejected point goal")
                    self.active_goal = None
                    return
                self.goal_started_at = now
                self.result_future = self.active_goal.get_result_async()
            return
        if not self.sent_param_goal:
            self.sent_param_goal = True
            x = float(self.get_parameter("goal_x").value)
            y = float(self.get_parameter("goal_y").value)
            if math.isfinite(x) and math.isfinite(y):
                self._send_goal(self._pose(x, y, float(self.get_parameter("goal_yaw").value)))

    def _send_goal(self, pose: PoseStamped) -> None:
        if self.active_goal is not None or self.send_goal_future is not None:
            self.get_logger().warn("A point goal is already active; ignoring new goal")
            return
        if bool(self.get_parameter("validate_costmap").value) and not self._goal_is_clear(pose):
            self.get_logger().warn(
                f"Rejecting point goal x={pose.pose.position.x:+.2f} y={pose.pose.position.y:+.2f}: occupied or unknown in costmap"
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal.pose)
        self.get_logger().info(f"Sending point goal x={pose.pose.position.x:+.2f} y={pose.pose.position.y:+.2f}")
        self.send_goal_future = self.action_client.send_goal_async(goal)

    def _goal_is_clear(self, pose: PoseStamped) -> bool:
        if self.costmap is None:
            self.get_logger().warn("No costmap yet; accepting goal without validation", throttle_duration_sec=5.0)
            return True
        xy = self._transform_xy(pose)
        if xy is None:
            return False
        x, y = xy
        info = self.costmap.info
        radius = float(self.get_parameter("goal_clearance").value)
        cells = max(1, int(math.ceil(radius / max(info.resolution, 1e-6))))
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
                    return False
                wx = info.origin.position.x + (gx + 0.5) * info.resolution
                wy = info.origin.position.y + (gy + 0.5) * info.resolution
                if math.hypot(wx - x, wy - y) > radius:
                    continue
                value = self.costmap.data[gy * info.width + gx]
                if value < 0 or value >= occupied_threshold:
                    return False
        return True

    def _transform_xy(self, pose: PoseStamped) -> Optional[tuple[float, float]]:
        if pose.header.frame_id == self.costmap.header.frame_id:
            return float(pose.pose.position.x), float(pose.pose.position.y)
        try:
            transform = self.tf_buffer.lookup_transform(self.costmap.header.frame_id, pose.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"No transform {self.costmap.header.frame_id} <- {pose.header.frame_id}: {exc}", throttle_duration_sec=2.0)
            return None
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = math.cos(yaw) * pose.pose.position.x - math.sin(yaw) * pose.pose.position.y + tx
        y = math.sin(yaw) * pose.pose.position.x + math.cos(yaw) * pose.pose.position.y + ty
        return x, y

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose


def main() -> None:
    rclpy.init()
    node = G1PointGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
