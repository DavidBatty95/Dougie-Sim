"""ROS 2 simulation interfaces, explicit ground-truth odometry and virtual scan."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .simulation import JOINTS, rotation


class CommandWatchdog:
    def __init__(self, timeout=.5):
        self.timeout = timeout
        self.command = np.zeros(3)
        self.received = -math.inf

    def update(self, command, now):
        values = np.asarray(command, dtype=float)
        self.command = values.copy() if values.shape == (3,) and np.isfinite(values).all() else np.zeros(3)
        self.received = now

    def value(self, now):
        return self.command.copy() if 0 <= now-self.received <= self.timeout else np.zeros(3)


class Bridge:
    def __init__(self, sim, preset=(0., 0., 0.), *, publish_scan=True):
        rclpy.init()
        self.node = rclpy.create_node('dougie_simulator')
        self.sim = sim
        self.preset = np.asarray(preset, dtype=float)
        self.watchdog = CommandWatchdog()
        self.has_command = False
        self.scan_ranges = []
        self.scan_enabled = publish_scan
        self.pub_clock = self.node.create_publisher(Clock, '/clock', 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.pub_imu = self.node.create_publisher(Imu, '/imu', 10)
        self.pub_joints = self.node.create_publisher(JointState, '/joint_states', 10)
        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 10)
        self.pub_marker = self.node.create_publisher(Marker, '/dougie/body', 10)
        self.tf = TransformBroadcaster(self.node)
        self.sub = self.node.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.node.get_logger().info(f'Go2 ready: 12 joints; simulator-truth odometry; horizontal scan enabled={publish_scan}')

    def receive(self, message):
        self.has_command = True
        self.watchdog.update((message.linear.x, message.linear.y, message.angular.z), time.monotonic())

    def tick(self):
        sim = self.sim
        if sim.counter % 4 == 0:
            rclpy.spin_once(self.node, timeout_sec=0.)
            self.publish_state()
        if self.scan_enabled and sim.counter % 20 == 0:
            self.publish_scan()
        command = self.watchdog.value(time.monotonic()) if self.has_command else self.preset.copy()
        # Simple forward-stop assistance. Nav2 owns routing; this is not a
        # predictive people planner and does not claim all-direction safety.
        if command[0] > 0 and self.scan_ranges:
            n = len(self.scan_ranges)
            if min(self.scan_ranges[n//2-12:n//2+13]) < .65:
                command[0] = 0.
        return command

    def stamp(self):
        from builtin_interfaces.msg import Time
        ns = int(self.sim.data.time * 1e9)
        return Time(sec=ns//1000000000, nanosec=ns%1000000000)

    def publish_state(self):
        sim, stamp = self.sim, self.stamp()
        self.pub_clock.publish(Clock(clock=stamp))
        p, q = sim.data.qpos[:3], sim.data.qpos[3:7]
        R = rotation(q)
        yaw = math.atan2(R[1, 0], R[0, 0])
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(p[0]), float(p[1])
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        vx, vy = sim.data.qvel[:2]
        odom.twist.twist.linear.x = float(math.cos(yaw)*vx+math.sin(yaw)*vy)
        odom.twist.twist.linear.y = float(-math.sin(yaw)*vx+math.cos(yaw)*vy)
        odom.twist.twist.angular.z = float((R @ sim.data.qvel[3:6])[2])
        self.pub_odom.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = float(p[0])
        transform.transform.translation.y = float(p[1])
        transform.transform.rotation = odom.pose.pose.orientation
        scan_tf = TransformStamped()
        scan_tf.header.stamp, scan_tf.header.frame_id, scan_tf.child_frame_id = stamp, 'base_footprint', 'base_scan'
        scan_tf.transform.translation.z = float(p[2]+.12)
        scan_tf.transform.rotation.w = 1.
        imu_tf = TransformStamped()
        imu_tf.header.stamp, imu_tf.header.frame_id, imu_tf.child_frame_id = stamp, 'odom', 'base_link'
        imu_tf.transform.translation.x, imu_tf.transform.translation.y, imu_tf.transform.translation.z = map(float, p)
        imu_tf.transform.rotation.w, imu_tf.transform.rotation.x, imu_tf.transform.rotation.y, imu_tf.transform.rotation.z = map(float, q)
        self.tf.sendTransform([transform, scan_tf, imu_tf])
        joints = JointState()
        joints.header.stamp, joints.name = stamp, list(JOINTS)
        joints.position = sim.data.qpos[sim.qadr].tolist()
        joints.velocity = sim.data.qvel[sim.vadr].tolist()
        joints.effort = sim.data.ctrl[sim.actuators].tolist()
        self.pub_joints.publish(joints)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        imu.orientation = imu_tf.transform.rotation
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, sim.data.sensor('imu_gyro').data)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, sim.data.sensor('imu_acc').data)
        self.pub_imu.publish(imu)
        marker = Marker()
        marker.header.stamp, marker.header.frame_id = stamp, 'base_link'
        marker.ns, marker.id, marker.type = 'dougie', 0, Marker.CUBE
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = .6, .3, .18
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .7, .9, 1.
        self.pub_marker.publish(marker)

    def publish_scan(self):
        self.scan_ranges = self.sim.scan()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.stamp(), 'base_scan'
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/len(self.scan_ranges)
        scan.angle_max = scan.angle_min + (len(self.scan_ranges)-1)*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .05, 8., .1
        scan.ranges = self.scan_ranges
        self.pub_scan.publish(scan)

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
