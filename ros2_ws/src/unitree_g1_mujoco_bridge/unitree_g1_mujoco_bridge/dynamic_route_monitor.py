"""Request Nav2 replans from observed route changes without owning action goals."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt64
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_social_navigation_msgs.msg import TrackedPersons

from .observed_route import route_is_free
from .route_replanning import ReplanPolicy, corridor_people, distance_to_route, forward_route


class DynamicRouteMonitor(Node):
    def __init__(self):
        super().__init__("dynamic_route_monitor")
        defaults = {"minimum_replan_interval": .2, "maximum_replan_interval": .5,
                    "prediction_horizon": 1.5, "robot_radius": .38,
                    "off_route_distance": .45, "people_timeout": .6,
                    "map_timeout": 2.5, "robot_frame": "base_footprint"}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.policy = ReplanPolicy(float(self.get_parameter("minimum_replan_interval").value),
                                   float(self.get_parameter("maximum_replan_interval").value))
        self.goal = self.path = self.people = self.map = None
        self.map_received = self.mission_received = -math.inf
        self.active = False
        self.sequence = time.time_ns()
        self.pending_since = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.request_pub = self.create_publisher(UInt64, "/navigation/replan_request", 1)
        self.debug_pub = self.create_publisher(String, "/navigation/replanning_status", 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, "/benchmark/current_goal", self._goal, 1)
        self.create_subscription(Bool, "/benchmark/goal_active", self._mission, 1)
        self.create_subscription(Path, "/plan", self._path, 1)
        self.create_subscription(TrackedPersons, "/tracked_persons", self._people, qos)
        self.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        self.create_timer(.05, self._tick)

    def _goal(self, msg):
        if self.goal is None or (msg.header.frame_id != self.goal.header.frame_id or
                math.hypot(msg.pose.position.x-self.goal.pose.position.x, msg.pose.position.y-self.goal.pose.position.y) > .01):
            self.path = None
            self.policy = ReplanPolicy(self.policy.minimum_interval, self.policy.maximum_interval)
        self.goal = msg

    def _mission(self, msg):
        self.active = msg.data
        self.mission_received = time.monotonic()
        if not self.active:
            self.pending_since = None

    def _path(self, msg):
        if not self.active or self.goal is None or not msg.poses or msg.header.frame_id != self.goal.header.frame_id:
            return
        endpoint = msg.poses[-1].pose.position
        if math.hypot(endpoint.x-self.goal.pose.position.x, endpoint.y-self.goal.pose.position.y) > .30:
            return
        if not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in msg.poses):
            return
        if (self.path is not None and (msg.header.stamp.sec, msg.header.stamp.nanosec) <
                (self.path.header.stamp.sec, self.path.header.stamp.nanosec)):
            return
        self.path = msg
        now = time.monotonic()
        self.policy.accept_plan(now)
        if self.pending_since is not None:
            self._status("route_received", request_to_route_ms=(now-self.pending_since)*1000.)
            self.pending_since = None

    def _people(self, msg):
        self.people = msg

    def _map(self, msg):
        self.map, self.map_received = msg, time.monotonic()

    def _transform(self, target, source):
        if target == source:
            return 0., 0., 1., 0.
        transform = self.tf.lookup_transform(target, source, rclpy.time.Time()).transform
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return transform.translation.x, transform.translation.y, math.cos(yaw), math.sin(yaw)

    def _tick(self):
        now = time.monotonic()
        if not self.active or now-self.mission_received > 1.0 or self.goal is None:
            return
        # Avoid doing expensive route checks faster than requests can be served.
        if now-self.policy.last_request < self.policy.minimum_interval:
            return
        frame = self.goal.header.frame_id
        try:
            x, y, _, _ = self._transform(frame, str(self.get_parameter("robot_frame").value))
        except TransformException:
            return
        points = [] if self.path is None else [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        ahead = forward_route(points, (x, y))
        observed = []
        people_fresh = False
        if self.people is not None:
            stamp = self.people.header.stamp.sec + self.people.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9 - stamp
            if 0 <= age <= float(self.get_parameter("people_timeout").value):
                people_fresh = True
                try:
                    tx, ty, c, s = self._transform(frame, self.people.header.frame_id)
                except TransformException:
                    return
                for person in self.people.persons:
                    if not math.isfinite(person.confidence) or person.confidence < .5 or person.track_state not in (person.TRACK_STATE_CONFIRMED, person.TRACK_STATE_LOST):
                        continue
                    observation_age = person.observation_age.sec + person.observation_age.nanosec*1e-9
                    if person.track_state == person.TRACK_STATE_LOST and observation_age+age > float(self.get_parameter("people_timeout").value):
                        continue
                    vx, vy = person.velocity.twist.linear.x, person.velocity.twist.linear.y
                    px, py = person.pose.pose.position.x+age*vx, person.pose.pose.position.y+age*vy
                    if not all(math.isfinite(v) for v in (px, py, vx, vy, person.body_radius)):
                        continue
                    observed.append((person.track_id, tx+c*px-s*py, ty+s*px+c*py,
                                     c*vx-s*vy, s*vx+c*vy, max(.1, person.body_radius)))
        relevant = (corridor_people(observed, ahead, float(self.get_parameter("prediction_horizon").value))
                    if people_fresh else self.policy.people_at_plan)
        blocked = False
        if (ahead and self.map is not None and self.map.header.frame_id == frame
                and now-self.map_received <= float(self.get_parameter("map_timeout").value)):
            route = Path()
            route.header.frame_id = frame
            for px, py in ahead:
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = px, py
                route.poses.append(pose)
            blocked = not route_is_free(self.map, route, float(self.get_parameter("robot_radius").value))
        reason = self.policy.reason(now, relevant, bool(points), blocked,
            bool(points) and distance_to_route((x, y), points) > float(self.get_parameter("off_route_distance").value))
        if reason:
            self.sequence += 1
            request = UInt64()
            request.data = self.sequence
            self.request_pub.publish(request)
            if self.pending_since is None:
                self.pending_since = now
            self._status(reason, relevant_people=len(relevant))

    def _status(self, reason, **values):
        msg = String()
        msg.data = json.dumps({"reason": reason, "sequence": self.sequence, **values})
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicRouteMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
