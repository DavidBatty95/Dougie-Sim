import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_hg.msg import LowState
from visualization_msgs.msg import Marker

from .pointcloud import iter_xyz


class G1WaypointExplorer(Node):
    def __init__(self) -> None:
        super().__init__("g1_waypoint_explorer")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("startup_delay", 8.0)
        self.declare_parameter("goal_timeout", 45.0)
        self.declare_parameter("goal_tolerance", 0.8)
        self.declare_parameter("min_goal_distance", 2.0)
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("obstacle_frame", "world")
        self.declare_parameter("predicted_cloud_topic", "/predicted_dynamic_obstacles")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("goal_marker_topic", "/exploration_goal_marker")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("dynamic_goal_clearance", 1.35)
        self.declare_parameter("dynamic_route_clearance", 1.05)
        self.declare_parameter("dynamic_cloud_timeout", 1.5)
        self.declare_parameter("grid_goal_clearance", 0.45)
        self.declare_parameter("grid_route_clearance", 0.35)
        self.declare_parameter("require_known_free_goal", True)
        self.declare_parameter("max_goal_tilt", 0.45)
        self.declare_parameter("seed", 11)

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_pub = self.create_publisher(PoseStamped, "/exploration_goal", 10)
        self.marker_pub = self.create_publisher(Marker, str(self.get_parameter("goal_marker_topic").value), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.started_at = self.get_clock().now()
        self.goal_started_at = None
        self.active_goal = None
        self.send_goal_future = None
        self.result_future = None
        self.last_goal: Optional[tuple[float, float]] = None
        self.last_goal_pose: Optional[PoseStamped] = None
        self.map_grid: Optional[OccupancyGrid] = None
        self.costmap_grid: Optional[OccupancyGrid] = None
        self.last_tilt = 0.0
        self.last_lowstate_time = None
        self.dynamic_points_by_source: dict[str, list[tuple[float, float]]] = {}
        self.dynamic_points_stamp_by_source: dict[str, object] = {}
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._map_callback, map_qos)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("costmap_topic").value), self._costmap_callback, costmap_qos)
        self.create_subscription(LowState, str(self.get_parameter("lowstate_topic").value), self._lowstate_callback, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("predicted_cloud_topic").value),
            lambda msg: self._dynamic_cloud_callback(msg, "predicted"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("tracked_cloud_topic").value),
            lambda msg: self._dynamic_cloud_callback(msg, "tracked"),
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._tick)
        self.create_timer(0.5, self._publish_active_goal_marker)
        self.get_logger().info("Nav2 waypoint explorer ready; choosing random reachable people-aware goals")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_grid = msg

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self.costmap_grid = msg

    def _lowstate_callback(self, msg: LowState) -> None:
        rpy = msg.imu_state.rpy
        if len(rpy) >= 2:
            self.last_tilt = math.hypot(float(rpy[0]), float(rpy[1]))
            self.last_lowstate_time = self.get_clock().now()

    def _dynamic_cloud_callback(self, msg: PointCloud2, source: str) -> None:
        self.dynamic_points_by_source[source] = [(x, y) for x, y, _ in iter_xyz(msg)]
        self.dynamic_points_stamp_by_source[source] = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < float(self.get_parameter("startup_delay").value):
            return
        if not self.action_client.server_is_ready():
            self.get_logger().warn("Waiting for Nav2 navigate_to_pose action server", throttle_duration_sec=5.0)
            return
        if not self._robot_upright():
            self.get_logger().warn(
                f"Waiting for upright robot before sending goals (tilt={self.last_tilt:.2f})",
                throttle_duration_sec=5.0,
            )
            return
        if self.result_future is not None:
            if self.result_future.done():
                self.get_logger().info("Navigation goal finished; choosing another exploration point")
                self.result_future = None
                self.active_goal = None
                self.goal_started_at = None
                self.last_goal_pose = None
            elif self.goal_started_at is not None:
                elapsed = (now - self.goal_started_at).nanoseconds * 1e-9
                if elapsed > float(self.get_parameter("goal_timeout").value):
                    self.get_logger().warn("Navigation goal timed out; cancelling and choosing another")
                    self.active_goal.cancel_goal_async()
                    self.result_future = None
                    self.active_goal = None
                    self.goal_started_at = None
                    self.last_goal_pose = None
                elif self.last_goal is not None and self._dynamic_route_blocked(self.last_goal):
                    self.get_logger().warn("Dynamic person obstacle moved into active route; cancelling and replanning")
                    self.active_goal.cancel_goal_async()
                    self.result_future = None
                    self.active_goal = None
                    self.goal_started_at = None
                    self.last_goal_pose = None
            return
        if self.send_goal_future is not None:
            if self.send_goal_future.done():
                self.active_goal = self.send_goal_future.result()
                self.send_goal_future = None
                if not self.active_goal.accepted:
                    self.get_logger().warn("Nav2 rejected exploration goal")
                    self.active_goal = None
                    return
                self.goal_started_at = now
                self.result_future = self.active_goal.get_result_async()
            return
        self._send_next_goal()

    def _send_next_goal(self) -> None:
        if not self._robot_upright():
            return
        chosen = self._choose_goal()
        if chosen is None:
            self.get_logger().warn("No verified reachable exploration goal found yet; waiting for map/costmap updates", throttle_duration_sec=5.0)
            return
        x, y = chosen
        self.last_goal = (x, y)
        yaw = self.rng.uniform(-math.pi, math.pi)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter("frame_id").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(goal.pose)
        self.last_goal_pose = goal.pose
        self._publish_goal_marker(goal.pose)
        self.get_logger().info(f"Sending verified exploration goal x={x:+.2f} y={y:+.2f}")
        self.send_goal_future = self.action_client.send_goal_async(goal)

    def _robot_upright(self) -> bool:
        if self.last_lowstate_time is None:
            return False
        age = (self.get_clock().now() - self.last_lowstate_time).nanoseconds * 1e-9
        if age > 0.75:
            return False
        return self.last_tilt <= float(self.get_parameter("max_goal_tilt").value)

    def _choose_goal(self) -> Optional[tuple[float, float]]:
        frame_id = str(self.get_parameter("frame_id").value)
        obstacle_frame = str(self.get_parameter("obstacle_frame").value)
        robot_xy = self._robot_xy(frame_id)
        robot_xy_obstacle = self._robot_xy(obstacle_frame)
        for _ in range(80):
            x = self.rng.uniform(-4.35, 4.35)
            y = self.rng.uniform(-3.00, 3.00)
            if not self._candidate_is_valid(x, y, frame_id, obstacle_frame, robot_xy, robot_xy_obstacle):
                continue
            return x, y

        fallback = [(-3.8, -2.1), (-3.4, 2.0), (-1.0, -1.9), (1.7, -1.8), (3.3, 1.2), (3.7, -0.8)]
        for point in fallback:
            if self._candidate_is_valid(point[0], point[1], frame_id, obstacle_frame, robot_xy, robot_xy_obstacle):
                return point

        if robot_xy is None:
            return None
        for distance in (1.6, 2.2, 2.8):
            headings = [self.rng.uniform(-math.pi, math.pi) for _ in range(12)]
            for heading in headings:
                x = robot_xy[0] + distance * math.cos(heading)
                y = robot_xy[1] + distance * math.sin(heading)
                if self._candidate_is_valid(x, y, frame_id, obstacle_frame, robot_xy, robot_xy_obstacle):
                    return x, y
        return None

    def _candidate_is_valid(
        self,
        x: float,
        y: float,
        frame_id: str,
        obstacle_frame: str,
        robot_xy: Optional[tuple[float, float]],
        robot_xy_obstacle: Optional[tuple[float, float]],
    ) -> bool:
        if robot_xy is not None and math.hypot(x - robot_xy[0], y - robot_xy[1]) < 1.4:
            return False
        if not self._is_walkable(x, y):
            return False
        if not self._grid_goal_is_free(x, y, frame_id):
            return False
        if robot_xy is not None and self._segment_static_blocked(robot_xy[0], robot_xy[1], x, y):
            return False
        if robot_xy is not None and self._segment_grid_blocked(robot_xy[0], robot_xy[1], x, y, frame_id):
            return False
        obstacle_xy = self._transform_xy(x, y, frame_id, obstacle_frame)
        if obstacle_xy is None:
            return False
        if self._near_dynamic_obstacle(obstacle_xy[0], obstacle_xy[1], float(self.get_parameter("dynamic_goal_clearance").value)):
            return False
        if robot_xy_obstacle is not None and self._segment_near_dynamic(robot_xy_obstacle[0], robot_xy_obstacle[1], obstacle_xy[0], obstacle_xy[1]):
            return False
        if self.last_goal is not None:
            if math.hypot(x - self.last_goal[0], y - self.last_goal[1]) < float(self.get_parameter("min_goal_distance").value):
                return False
        return True

    def _is_walkable(self, x: float, y: float) -> bool:
        margin = 0.55
        if x < -4.55 + margin or x > 4.55 - margin or y < -3.25 + margin or y > 3.25 - margin:
            return False
        boxes = [
            (-5.30, -2.15, 0.20, 1.00),
            (5.25, 1.75, 0.20, 0.95),
            (-2.15, -2.75, 0.95, 0.30),
            (2.35, 2.65, 0.95, 0.30),
            (0.15, 2.55, 0.70, 0.45),
        ]
        for cx, cy, hx, hy in boxes:
            if abs(x - cx) <= hx + margin and abs(y - cy) <= hy + margin:
                return False
        return True

    def _segment_static_blocked(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        for index in range(1, 18):
            t = index / 18.0
            if not self._is_walkable(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return True
        return False

    def _grid_goal_is_free(self, x: float, y: float, frame_id: str) -> bool:
        for grid in (self.costmap_grid, self.map_grid):
            if grid is None:
                continue
            xy = self._transform_xy(x, y, frame_id, grid.header.frame_id)
            if xy is None:
                continue
            if not self._grid_region_is_free(grid, xy[0], xy[1], float(self.get_parameter("grid_goal_clearance").value)):
                return False
        return True

    def _segment_grid_blocked(self, x0: float, y0: float, x1: float, y1: float, frame_id: str) -> bool:
        for index in range(1, 22):
            t = index / 22.0
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            for grid in (self.costmap_grid, self.map_grid):
                if grid is None:
                    continue
                xy = self._transform_xy(x, y, frame_id, grid.header.frame_id)
                if xy is None:
                    continue
                if not self._grid_region_is_free(grid, xy[0], xy[1], float(self.get_parameter("grid_route_clearance").value)):
                    return True
        return False

    def _grid_region_is_free(self, grid: OccupancyGrid, x: float, y: float, radius: float) -> bool:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0 or grid.info.width == 0 or grid.info.height == 0:
            return True
        cx = int((x - grid.info.origin.position.x) / resolution)
        cy = int((y - grid.info.origin.position.y) / resolution)
        cells = max(1, int(math.ceil(radius / resolution)))
        saw_known_free = False
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if math.hypot((gx - cx) * resolution, (gy - cy) * resolution) > radius:
                    continue
                if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
                    if bool(self.get_parameter("require_known_free_goal").value):
                        return False
                    continue
                value = int(grid.data[gy * grid.info.width + gx])
                if value < 0:
                    if bool(self.get_parameter("require_known_free_goal").value):
                        return False
                    continue
                if value >= 50:
                    return False
                saw_known_free = True
        return saw_known_free or not bool(self.get_parameter("require_known_free_goal").value)

    def _dynamic_route_blocked(self, goal_xy: tuple[float, float]) -> bool:
        robot_xy = self._robot_xy(str(self.get_parameter("obstacle_frame").value))
        if robot_xy is None:
            return False
        goal_xy = self._transform_xy(goal_xy[0], goal_xy[1], str(self.get_parameter("frame_id").value), str(self.get_parameter("obstacle_frame").value))
        if goal_xy is None:
            return False
        return self._near_dynamic_obstacle(*goal_xy, float(self.get_parameter("dynamic_goal_clearance").value)) or self._segment_near_dynamic(
            robot_xy[0], robot_xy[1], goal_xy[0], goal_xy[1]
        )

    def _active_dynamic_points(self) -> list[tuple[float, float]]:
        now = self.get_clock().now()
        timeout = float(self.get_parameter("dynamic_cloud_timeout").value)
        points = []
        for source, stamp in self.dynamic_points_stamp_by_source.items():
            age = (now - stamp).nanoseconds * 1e-9
            if age <= timeout:
                points.extend(self.dynamic_points_by_source.get(source, []))
        return points

    def _near_dynamic_obstacle(self, x: float, y: float, clearance: float) -> bool:
        return any(math.hypot(x - px, y - py) <= clearance for px, py in self._active_dynamic_points())

    def _segment_near_dynamic(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        clearance = float(self.get_parameter("dynamic_route_clearance").value)
        length_sq = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)
        if length_sq < 1e-6:
            return self._near_dynamic_obstacle(x1, y1, clearance)
        for px, py in self._active_dynamic_points():
            t = ((px - x0) * (x1 - x0) + (py - y0) * (y1 - y0)) / length_sq
            t = max(0.0, min(1.0, t))
            closest_x = x0 + (x1 - x0) * t
            closest_y = y0 + (y1 - y0) * t
            if math.hypot(px - closest_x, py - closest_y) <= clearance:
                return True
        return False

    def _robot_xy(self, target_frame: str) -> Optional[tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, str(self.get_parameter("robot_frame").value), rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"No robot transform into {target_frame}: {exc}", throttle_duration_sec=5.0)
            return None
        return transform.transform.translation.x, transform.transform.translation.y

    def _transform_xy(self, x: float, y: float, from_frame: str, to_frame: str) -> Optional[tuple[float, float]]:
        if from_frame == to_frame:
            return x, y
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"No transform {to_frame} <- {from_frame}: {exc}", throttle_duration_sec=5.0)
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        return tx + math.cos(yaw) * x - math.sin(yaw) * y, ty + math.sin(yaw) * x + math.cos(yaw) * y

    def _publish_active_goal_marker(self) -> None:
        if self.last_goal_pose is not None:
            self._publish_goal_marker(self.last_goal_pose)

    def _publish_goal_marker(self, pose: PoseStamped) -> None:
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "exploration_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = 0.18
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.36
        marker.scale.y = 0.36
        marker.scale.z = 0.36
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.lifetime.sec = 2
        self.marker_pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = G1WaypointExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
