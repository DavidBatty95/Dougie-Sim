"""Release independent pedestrian motion once per ready navigation episode."""
import socket
import struct
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PedestrianStartBridge(Node):
    def __init__(self):
        super().__init__('pedestrian_start_bridge')
        self.declare_parameter('socket_path', '/tmp/unitree_g1_pedestrian_start.sock')
        self.path = str(self.get_parameter('socket_path').value)
        self.active = self.started = False
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind('\0unitree-actor-start-' + uuid.uuid4().hex)
        self.pub = self.create_publisher(Bool, '/simulation/pedestrians_started', 1)
        self.create_subscription(Bool, '/benchmark/goal_active', self.goal_callback, 1)
        self.create_timer(.05, self.tick)

    def goal_callback(self, message):
        self.active = message.data

    def tick(self):
        if self.active:
            try:
                self.sock.sendto(struct.pack('<II', 0x31535047, 1), self.path)
            except OSError:
                pass  # Benchmark requires a real acknowledgement, never this send attempt.
        try:
            while True:
                payload = self.sock.recv(16)
                if payload == struct.pack('<II', 0x31415347, 1):
                    self.started = True
        except BlockingIOError:
            pass
        message = Bool()
        message.data = self.started
        self.pub.publish(message)

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = PedestrianStartBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
