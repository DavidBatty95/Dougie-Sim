"""Bounded single-goal frontier exploration through Nav2, using sensed SLAM maps."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .frontiers import frontier_goals


class Explorer(Node):
    def __init__(self):
        super().__init__('dougie_explorer')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else None
        self.map = None
        self.received = 0.
        self.pending = None
        self.goal_handle = None
        self.result = None
        self.cancel = None
        self.selected = None
        self.started = 0.
        self.excluded = []
        self.goal_count = 0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.receive, qos)
        self.status = self.create_publisher(String, '/dougie/exploration_status', 10)
        self.timer = self.create_timer(1., self.tick)

    def receive(self, message):
        if message.header.frame_id == 'map':
            self.map, self.received = message, time.monotonic()

    def report(self, state, **fields):
        record = dict(state=state, goals_sent=self.goal_count, **fields)
        self.status.publish(String(data=json.dumps(record)))
        self.get_logger().info(json.dumps(record))

    def tick(self):
        now = time.monotonic()
        if self.pending:
            if not self.pending.done():
                return
            self.goal_handle = self.pending.result()
            self.pending = None
            if not self.goal_handle.accepted:
                self.report('goal_rejected')
                self.excluded.append((self.selected, now))
                self.goal_handle = None
                return
            self.result = self.goal_handle.get_result_async()
        if self.result:
            if self.result.done():
                status = self.result.result().status
                self.report('arrived' if status == GoalStatus.STATUS_SUCCEEDED else 'navigation_failed', status=status)
                self.excluded.append((self.selected, now))
                self.result = self.goal_handle = self.cancel = None
            elif now-self.started > 75. and self.cancel is None:
                self.cancel = self.goal_handle.cancel_goal_async()
                self.report('cancelling_timeout')
                return
            else:
                return  # No new goal until the prior action has a terminal result.
        if self.map is None or now-self.received > 10. or not self.client.server_is_ready():
            self.report('waiting_for_map_or_navigation')
            return
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        except TransformException:
            return
        robot = transform.translation.x, transform.translation.y
        self.excluded = [(p, stamp) for p, stamp in self.excluded if now-stamp < 60.]
        try:
            goals = frontier_goals(self.map, robot, [p for p, _ in self.excluded])
        except ValueError as exc:
            self.report('invalid_map', error=str(exc))
            return
        if not goals:
            self.report('no_reachable_frontier')
            return
        _, x, y, yaw = goals[0]
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pending = self.client.send_goal_async(NavigateToPose.Goal(pose=pose))
        self.selected, self.started = (x, y), now
        self.goal_count += 1
        self.report('goal_sent', goal=[x, y, yaw], candidates=len(goals))


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
