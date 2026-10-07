"""Independent ROS interface observations; never fed back to the controller."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, JointState, Imu
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self, directory):
        super().__init__('dougie_recorder')
        self.directory = directory
        self.counts = {}
        self.latest = {}
        self.initial_position = None
        self.path_length = 0.
        self.previous = None
        self.events = []
        self.subscriptions_owned = []
        for topic, kind in (('/odom', Odometry), ('/scan', LaserScan), ('/joint_states', JointState),
                            ('/imu', Imu), ('/cmd_vel', Twist), ('/dougie/exploration_status', String)):
            self.subscriptions_owned.append(self.create_subscription(kind, topic, lambda msg, t=topic:self.receive(t, msg), 10))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_owned.append(self.create_subscription(OccupancyGrid, '/map', lambda msg:self.receive('/map', msg), qos))
        self.timer = self.create_timer(1., self.save)

    def receive(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0)+1
        if topic == '/odom':
            import math
            point = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            if self.previous is not None:
                self.path_length += math.dist(point, self.previous)
            else:
                self.initial_position = point
            self.previous = point
            self.latest['position'] = point
        elif topic == '/joint_states':
            self.latest['joint_count'] = len(msg.name)
        elif topic == '/map':
            self.latest['map'] = dict(width=msg.info.width, height=msg.info.height,
                known_cells=sum(value >= 0 for value in msg.data), free_cells=sum(value == 0 for value in msg.data))
            (self.directory/'observed_map.json').write_text(json.dumps(dict(width=msg.info.width,
                height=msg.info.height, resolution=msg.info.resolution,
                origin=[msg.info.origin.position.x, msg.info.origin.position.y], data=list(msg.data))))
        elif topic == '/dougie/exploration_status':
            event = json.loads(msg.data)
            if not self.events or event != self.events[-1]:
                self.events.append(event)

    def save(self):
        (self.directory/'interfaces.json').write_text(json.dumps(dict(counts=self.counts, latest=self.latest,
            initial_position=self.initial_position, path_length_m=self.path_length, exploration_events=self.events), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.directory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
