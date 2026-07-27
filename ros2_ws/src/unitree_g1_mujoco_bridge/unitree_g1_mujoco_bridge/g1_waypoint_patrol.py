import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class G1WaypointPatrol(Node):
    """Simple world-frame waypoint command source for smoke testing motion."""

    def __init__(self) -> None:
        super().__init__("g1_waypoint_patrol")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("waypoints", "-2.2,-1.2;1.8,-1.2;2.0,1.2;-1.8,1.4;0.0,0.0")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("startup_delay", 5.0)
        self.declare_parameter("goal_tolerance", 0.45)
        self.declare_parameter("max_linear", 0.18)
        self.declare_parameter("max_angular", 0.45)
        self.declare_parameter("heading_tolerance", 0.45)
        self.declare_parameter("front_clearance", 0.85)
        self.declare_parameter("marker_topic", "/patrol_waypoint_markers")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_callback, 10)
        self.started_at = self.get_clock().now()
        self.front_range = math.inf
        self.waypoints = self._parse_waypoints(str(self.get_parameter("waypoints").value))
        self.index = 0
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("rate_hz").value)), self._tick)
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(f"Waypoint patrol ready with {len(self.waypoints)} waypoints")

    @staticmethod
    def _parse_waypoints(text: str) -> list[tuple[float, float]]:
        waypoints = []
        for item in text.split(";"):
            parts = [part.strip() for part in item.split(",")]
            if len(parts) != 2:
                continue
            try:
                waypoints.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
        return waypoints or [(0.0, 0.0)]

    def _scan_callback(self, msg: LaserScan) -> None:
        best = math.inf
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) > 0.35:
                continue
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                best = min(best, float(value))
        self.front_range = best

    def _tick(self) -> None:
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < float(self.get_parameter("startup_delay").value):
            self._publish(0.0, 0.0)
            return
        pose = self._pose()
        if pose is None:
            self._publish(0.0, 0.0)
            return
        x, y, yaw = pose
        goal_x, goal_y = self.waypoints[self.index]
        distance = math.hypot(goal_x - x, goal_y - y)
        if distance <= float(self.get_parameter("goal_tolerance").value):
            self.index = (self.index + 1) % len(self.waypoints)
            goal_x, goal_y = self.waypoints[self.index]
            distance = math.hypot(goal_x - x, goal_y - y)
            self.get_logger().info(f"Reached patrol waypoint; next x={goal_x:+.2f} y={goal_y:+.2f}")
        target_heading = math.atan2(goal_y - y, goal_x - x)
        heading_error = self._wrap_pi(target_heading - yaw)
        max_linear = float(self.get_parameter("max_linear").value)
        max_angular = float(self.get_parameter("max_angular").value)
        angular = max(-max_angular, min(max_angular, 1.2 * heading_error))
        linear = min(max_linear, 0.35 * distance)
        if abs(heading_error) > float(self.get_parameter("heading_tolerance").value):
            linear *= max(0.0, math.cos(heading_error))
        if self.front_range < float(self.get_parameter("front_clearance").value):
            linear = min(linear, 0.03)
            angular = max(angular, 0.25)
        self._publish(linear, angular)

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("target_frame").value),
                str(self.get_parameter("robot_frame").value),
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"No patrol TF: {exc}", throttle_duration_sec=2.0)
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for index, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = str(self.get_parameter("target_frame").value)
            marker.header.stamp = now
            marker.ns = "patrol_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.28 if index == self.index else 0.18
            marker.color = ColorRGBA(r=0.1, g=1.0 if index == self.index else 0.4, b=1.0, a=0.95)
            markers.markers.append(marker)
        path = Marker()
        path.header.frame_id = str(self.get_parameter("target_frame").value)
        path.header.stamp = now
        path.ns = "patrol_waypoint_path"
        path.id = 1000
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.scale.x = 0.04
        path.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.65)
        for x, y in self.waypoints + [self.waypoints[0]]:
            path.points.append(Point(x=x, y=y, z=0.05))
        markers.markers.append(path)
        self.marker_pub.publish(markers)

    @staticmethod
    def _wrap_pi(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main() -> None:
    rclpy.init()
    node = G1WaypointPatrol()
    try:
        rclpy.spin(node)
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
