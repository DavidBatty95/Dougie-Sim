import math
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class G1FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("g1_frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/exploration_goal")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("startup_delay", 12.0)
        self.declare_parameter("goal_timeout", 60.0)
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("min_goal_distance", 1.5)
        self.declare_parameter("frontier_blacklist_radius", 1.0)
        self.declare_parameter("complete_unknown_threshold", 0.04)

        self.map: Optional[OccupancyGrid] = None
        self.blacklist: list[tuple[float, float]] = []
        self.started_at = self.get_clock().now()
        self.goal_started_at = None
        self.active_goal = None
        self.send_goal_future = None
        self.result_future = None
        self.last_goal: Optional[tuple[float, float]] = None
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._map_callback, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("Frontier explorer ready; will map unknown space using Nav2 NavigateToPose goals")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _tick(self) -> None:
        if self.done:
            return
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < float(self.get_parameter("startup_delay").value):
            return
        if self.map is None:
            self.get_logger().warn("Waiting for /map before frontier exploration", throttle_duration_sec=5.0)
            return
        if not self.action_client.server_is_ready():
            self.get_logger().warn("Waiting for Nav2 navigate_to_pose action server", throttle_duration_sec=5.0)
            return

        if self.result_future is not None:
            if self.result_future.done():
                self.result_future = None
                self.active_goal = None
                self.goal_started_at = None
            elif self.goal_started_at is not None:
                elapsed = (now - self.goal_started_at).nanoseconds * 1e-9
                if elapsed > float(self.get_parameter("goal_timeout").value):
                    if self.last_goal is not None:
                        self.blacklist.append(self.last_goal)
                    self.get_logger().warn("Frontier goal timed out; blacklisting and selecting another")
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
                    if self.last_goal is not None:
                        self.blacklist.append(self.last_goal)
                    self.get_logger().warn("Nav2 rejected frontier goal; selecting another")
                    self.active_goal = None
                    return
                self.goal_started_at = now
                self.result_future = self.active_goal.get_result_async()
            return

        robot = self._robot_xy()
        if robot is None:
            return
        unknown_fraction = self.map.data.count(-1) / max(1, len(self.map.data))
        if unknown_fraction < float(self.get_parameter("complete_unknown_threshold").value):
            self.done = True
            self.get_logger().info("Frontier exploration complete: remaining unknown area below threshold")
            return

        goal = self._choose_frontier_goal(robot)
        if goal is None:
            self.done = True
            self.get_logger().info("Frontier exploration complete: no usable frontiers found")
            return
        self._send_goal(goal)

    def _robot_xy(self) -> Optional[tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("robot_frame").value),
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for robot pose in map frame: {exc}", throttle_duration_sec=5.0)
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _choose_frontier_goal(self, robot: tuple[float, float]) -> Optional[tuple[float, float]]:
        assert self.map is not None
        width = self.map.info.width
        height = self.map.info.height
        data = self.map.data
        visited = set()
        best = None
        best_score = -math.inf
        min_size = int(self.get_parameter("min_frontier_size").value)
        min_distance = float(self.get_parameter("min_goal_distance").value)

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                if idx in visited or not self._is_frontier_cell(x, y):
                    continue
                cells = self._collect_frontier(x, y, visited)
                if len(cells) < min_size:
                    continue
                wx, wy = self._frontier_goal_from_cells(cells)
                distance = math.hypot(wx - robot[0], wy - robot[1])
                if distance < min_distance:
                    continue
                if any(math.hypot(wx - bx, wy - by) < float(self.get_parameter("frontier_blacklist_radius").value) for bx, by in self.blacklist):
                    continue
                score = len(cells) + 4.0 * distance
                if score > best_score:
                    best_score = score
                    best = (wx, wy)
        return best

    def _is_frontier_cell(self, x: int, y: int) -> bool:
        assert self.map is not None
        width = self.map.info.width
        data = self.map.data
        if data[y * width + x] != 0:
            return False
        for ny in (y - 1, y, y + 1):
            for nx in (x - 1, x, x + 1):
                if data[ny * width + nx] == -1:
                    return True
        return False

    def _collect_frontier(self, start_x: int, start_y: int, visited: set[int]) -> list[tuple[int, int]]:
        assert self.map is not None
        width = self.map.info.width
        queue = deque([(start_x, start_y)])
        cells = []
        visited.add(start_y * width + start_x)
        while queue:
            x, y = queue.popleft()
            cells.append((x, y))
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                idx = ny * width + nx
                if idx not in visited and self._is_frontier_cell(nx, ny):
                    visited.add(idx)
                    queue.append((nx, ny))
        return cells

    def _frontier_goal_from_cells(self, cells: list[tuple[int, int]]) -> tuple[float, float]:
        assert self.map is not None
        cx = sum(x for x, _ in cells) / len(cells)
        cy = sum(y for _, y in cells) / len(cells)
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        return origin.x + (cx + 0.5) * resolution, origin.y + (cy + 0.5) * resolution

    def _send_goal(self, goal_xy: tuple[float, float]) -> None:
        self.last_goal = goal_xy
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_xy[0]
        goal.pose.pose.position.y = goal_xy[1]
        goal.pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal.pose)
        self.get_logger().info(f"Sending frontier goal x={goal_xy[0]:+.2f} y={goal_xy[1]:+.2f}")
        self.send_goal_future = self.action_client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = G1FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
