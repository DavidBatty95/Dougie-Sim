"""LiDAR/IMU/joint-only front end; publishes estimated odometry and level clouds."""
import json
import math
import time
from array import array
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from unitree_hg.msg import LowState

from .lidar_odometry import ScanOdometry, rotation


def axis_rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 0:
        return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    if axis == 1:
        return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


def lidar_extrinsics(rpy, joints):
    """Robot calibration and measured joints, not scene geometry or root pose."""
    body = axis_rotation(1, rpy[1]) @ axis_rotation(0, rpy[0])
    waist = body @ axis_rotation(2, joints.get("waist_yaw_joint", 0.0))
    origin = np.array((0., 0., joints.get("floating_base_joint", 0.793)))
    origin += waist @ np.array((-0.0039635, 0., 0.035))
    waist = waist @ axis_rotation(0, joints.get("waist_roll_joint", 0.0))
    origin += waist @ np.array((0., 0., 0.019))
    waist = waist @ axis_rotation(1, joints.get("waist_pitch_joint", 0.0))
    origin += waist @ np.array((0.0576235, 0., 0.41987))
    return origin, waist @ np.diag((1., -1., -1.))


class OnboardLidarOdometry(Node):
    def __init__(self):
        super().__init__("onboard_lidar_odometry")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.estimator = ScanOdometry()
        self.imu = deque(maxlen=150)
        self.joints = deque(maxlen=150)
        self.previous_stamp = None
        self.previous_xy = np.zeros(2)
        self.velocity = np.zeros(2)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        self.cloud_pub = self.create_publisher(PointCloud2, "/onboard/lidar", qos)
        self.level_pub = self.create_publisher(PointCloud2, "/onboard/lidar_level", qos)
        self.status_pub = self.create_publisher(String, "/onboard/localization_status", 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(LowState, "/lowstate", self._imu, qos)
        self.create_subscription(JointState, "/joint_states", self._joints, qos)
        self.create_subscription(PointCloud2, "/livox/lidar", self._cloud, qos)

    def _imu(self, msg):
        self.imu.append((self.get_clock().now().nanoseconds * 1e-9,
                         tuple(msg.imu_state.rpy), float(msg.imu_state.gyroscope[2])))

    def _joints(self, msg):
        self.joints.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            dict(zip(msg.name, msg.position))))

    def _cloud(self, msg):
        start = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.imu or not self.joints or (self.previous_stamp is not None and stamp <= self.previous_stamp):
            return
        imu = min(self.imu, key=lambda item: abs(item[0] - stamp))
        joints = min(self.joints, key=lambda item: abs(item[0] - stamp))
        if abs(imu[0] - stamp) > 0.15 or abs(joints[0] - stamp) > 0.15:
            return
        if max(abs(imu[1][0]), abs(imu[1][1])) > 0.65:
            return
        # The transport contract is packed local xyz/intensity float32.
        if msg.point_step != 16 or msg.is_bigendian:
            return
        data = np.frombuffer(msg.data, dtype="<f4").reshape(-1, 4).copy()
        origin, attitude = lidar_extrinsics(imu[1], joints[1])
        data[:, :3] = data[:, :3] @ attitude.T + origin
        radius = np.linalg.norm(data[:, :2], axis=1)
        floor_candidates = data[(radius > 1.0) & (radius < 8.0) & (data[:, 2] < 0.35) & (data[:, 2] > -0.35), 2]
        if len(floor_candidates) >= 30:
            ground = float(np.median(floor_candidates))
            if np.quantile(np.abs(floor_candidates - ground), 0.7) < 0.04:
                data[:, 2] -= ground
        usable = ((data[:, 2] > 0.25) & (data[:, 2] < 1.8) & (radius > 1.0) & (radius < 18.0))
        accepted = self.estimator.update(data[usable, :2], imu[1][2])
        status = String()
        status.data = json.dumps({**self.estimator.quality, "source": "lidar_imu_joint_encoders",
                                  "source_stamp": stamp, "input_age_s": self.get_clock().now().nanoseconds * 1e-9 - stamp,
                                  "cloud_points": len(data), "floor_points": len(floor_candidates),
                                  "processing_ms": (time.monotonic() - start) * 1000})
        self.status_pub.publish(status)
        self.get_logger().info(f"Registration {status.data}", throttle_duration_sec=3.0)
        if not accepted:
            return  # No fresh odometry/TF on failure: downstream freshness stops.
        xy, yaw = self.estimator.xy, self.estimator.yaw
        if self.previous_stamp is not None:
            dt = stamp - self.previous_stamp
            alpha = dt / (0.12 + dt)
            self.velocity += alpha * ((xy - self.previous_xy) / dt - self.velocity)
        self.previous_xy, self.previous_stamp = xy.copy(), stamp
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = msg.header.stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, xy)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        velocity = rotation(yaw).T @ self.velocity
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = map(float, velocity)
        odom.twist.twist.angular.z = imu[2]
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 0.02
            odom.twist.covariance[index] = 0.04
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = map(float, xy)
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        msg.header.frame_id = "base_footprint"
        msg.data = array("B", data.astype("<f4").tobytes())
        self.level_pub.publish(msg)
        data[:, :2] = data[:, :2] @ rotation(yaw).T + xy
        msg.header.frame_id = "odom"
        msg.data = array("B", data.astype("<f4").tobytes())
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OnboardLidarOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
