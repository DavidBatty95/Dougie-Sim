"""Send an estimated robot-relative goal to the viewer, never to navigation."""
import json
import math
import socket
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

GOAL_MAGIC = 0x314C4F47  # GOL1, little endian
GOAL_PACKET = struct.Struct("<IIQfff")


def encode_goal(active, forward=0., left=0., radius=.50, stamp_ns=None):
    if (not all(math.isfinite(v) for v in (forward, left, radius)) or not .05 <= radius <= 3.
            or abs(forward) > 1000 or abs(left) > 1000):
        raise ValueError("Invalid viewer goal geometry")
    return GOAL_PACKET.pack(GOAL_MAGIC, int(bool(active)),
                            time.monotonic_ns() if stamp_ns is None else stamp_ns,
                            forward, left, radius)


def relative_goal(goal, transform):
    q, p = transform.rotation, transform.translation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return p.x+c*goal.x-s*goal.y, p.y+s*goal.x+c*goal.y


class MujocoGoalVisualizer(Node):
    def __init__(self):
        super().__init__("mujoco_goal_visualizer")
        self.declare_parameter("socket_path", "/tmp/unitree_g1_goal_visual.sock")
        self.declare_parameter("goal_radius", .50)
        self.declare_parameter("robot_frame", "base_footprint")
        self.socket_path = str(self.get_parameter("socket_path").value)
        self.radius = float(self.get_parameter("goal_radius").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.goal = None
        self.active = False
        self.heartbeat = -math.inf
        self.status = self.create_publisher(String, "/visualization/mujoco_goal_status", 1)
        self.create_subscription(PoseStamped, "/benchmark/current_goal", self._goal, 1)
        self.create_subscription(Bool, "/benchmark/goal_active", self._mission, 1)
        self.create_timer(.1, self._tick)

    def _goal(self, msg):
        self.goal = msg

    def _mission(self, msg):
        self.active, self.heartbeat = bool(msg.data), time.monotonic()
        if not self.active:
            self._send(False)

    def _send(self, active, forward=0., left=0.):
        delivered = False
        try:
            self.sock.sendto(encode_goal(active, forward, left, self.radius), self.socket_path)
            delivered = True
        except (OSError, ValueError):
            pass  # Viewer loss must never affect navigation.
        msg = String()
        msg.data = json.dumps({"active": active, "delivered": delivered,
                               "forward": forward, "left": left, "render_only": True})
        if rclpy.ok():
            self.status.publish(msg)

    def _tick(self):
        if not self.active or time.monotonic()-self.heartbeat > 1. or self.goal is None:
            self._send(False)
            return
        if self.goal.header.frame_id == self.robot_frame:
            self._send(True, self.goal.pose.position.x, self.goal.pose.position.y)
            return
        try:
            transform = self.tf.lookup_transform(self.robot_frame, self.goal.header.frame_id, rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
            if not 0 <= self.get_clock().now().nanoseconds*1e-9-stamp <= .5:
                self._send(False)
                return
            forward, left = relative_goal(self.goal.pose.position, transform.transform)
            self._send(True, forward, left)
        except TransformException:
            self._send(False)

    def destroy_node(self):
        self._send(False)
        self.sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoGoalVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
