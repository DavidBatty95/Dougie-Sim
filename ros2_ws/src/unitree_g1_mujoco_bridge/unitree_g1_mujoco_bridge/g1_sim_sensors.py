import math
import os
import json
import socket
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from unitree_go.msg import SportModeState

from .sensor_packets import decode_camera_packet, decode_lidar_packet


class G1SimSensors(Node):
    def __init__(self) -> None:
        super().__init__("g1_sim_sensors")
        self.declare_parameter("camera_frame", "mujoco_d435i_color_optical_frame")
        self.declare_parameter("depth_frame", "mujoco_d435i_color_optical_frame")
        self.declare_parameter("imu_frame", "mujoco_d435i_color_optical_frame")
        self.declare_parameter("lidar_frame", "mujoco_livox_frame")
        self.declare_parameter("lidar_output_frame", "mujoco_livox_frame")
        self.declare_parameter("camera_width", 160)
        self.declare_parameter("camera_height", 120)
        self.declare_parameter("publish_camera", False)
        self.declare_parameter("use_mujoco_raycast_camera", True)
        self.declare_parameter("mujoco_camera_socket", "/tmp/unitree_g1_mujoco_camera.sock")
        self.declare_parameter("camera_rate_hz", 5.0)
        self.declare_parameter("pointcloud_rate_hz", 0.2)
        self.declare_parameter("publish_depth_pointcloud", False)
        self.declare_parameter("imu_rate_hz", 100.0)
        self.declare_parameter("lidar_rate_hz", 10.0)
        self.declare_parameter("livox_point_rate", 200000)
        self.declare_parameter("livox_scan_points", 2400)
        self.declare_parameter("camera_render_step", 16)
        self.declare_parameter("livox_object_surface_samples", 0)
        self.declare_parameter("livox_floor_samples", 0)
        self.declare_parameter("livox_wall_horizontal_samples", 0)
        self.declare_parameter("livox_wall_vertical_samples", 0)
        self.declare_parameter("livox_min_range", 0.1)
        self.declare_parameter("livox_max_range", 40.0)
        self.declare_parameter("livox_horizontal_min_deg", 0.0)
        self.declare_parameter("livox_horizontal_max_deg", 360.0)
        self.declare_parameter("livox_vertical_min_deg", -12.0)
        self.declare_parameter("livox_vertical_max_deg", 18.0)
        self.declare_parameter("use_mujoco_raycast_lidar", True)
        self.declare_parameter("mujoco_lidar_socket", "/tmp/unitree_g1_mujoco_lidar.sock")
        self.declare_parameter("mujoco_pedestrian_socket", "/tmp/unitree_g1_mujoco_pedestrians.sock")
        self.declare_parameter("publish_lidar_tf", True)
        self.declare_parameter("use_mujoco_lidar_pose", True)
        self.declare_parameter("use_cmd_vel_lidar_fallback", False)
        self.declare_parameter("require_exact_lidar_pose", False)
        self.declare_parameter("suppress_lidar_when_fallen", True)
        self.declare_parameter("fallen_tilt_threshold", 0.95)
        self.declare_parameter("upright_tilt_threshold", 0.45)
        self.declare_parameter("lidar_resume_delay", 2.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("mujoco_state_topic", "rt/sportmodestate")
        self.declare_parameter("livox_base_offset_x", 0.10)
        self.declare_parameter("livox_base_offset_y", 0.0)
        self.declare_parameter("livox_base_offset_z", 0.46)
        self.declare_parameter("pelvis_to_livox_z", 0.34)
        self.declare_parameter("mujoco_scene", "scene_29dof.xml")
        self.declare_parameter("mujoco_robot_dir", "/workspace/ros2_ws/src/unitree_mujoco/unitree_robots/g1")
        self.declare_parameter("robot_head_top_height_m", 1.32)
        self.declare_parameter("livox_height_above_ground_m", 1.25318)

        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.depth_frame = str(self.get_parameter("depth_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.lidar_output_frame = str(self.get_parameter("lidar_output_frame").value)
        self.publish_camera = bool(self.get_parameter("publish_camera").value)
        self.use_mujoco_raycast_camera = bool(self.get_parameter("use_mujoco_raycast_camera").value)
        self.mujoco_camera_socket = str(self.get_parameter("mujoco_camera_socket").value)
        self.width = int(self.get_parameter("camera_width").value)
        self.height = int(self.get_parameter("camera_height").value)
        self.pointcloud_rate_hz = float(self.get_parameter("pointcloud_rate_hz").value)
        self.publish_depth_pointcloud = bool(self.get_parameter("publish_depth_pointcloud").value)
        self.lidar_rate_hz = float(self.get_parameter("lidar_rate_hz").value)
        self.livox_point_rate = int(self.get_parameter("livox_point_rate").value)
        self.camera_render_step = max(1, int(self.get_parameter("camera_render_step").value))
        self.livox_object_surface_samples = max(0, int(self.get_parameter("livox_object_surface_samples").value))
        self.livox_floor_samples = max(0, int(self.get_parameter("livox_floor_samples").value))
        self.livox_wall_horizontal_samples = max(0, int(self.get_parameter("livox_wall_horizontal_samples").value))
        self.livox_wall_vertical_samples = max(0, int(self.get_parameter("livox_wall_vertical_samples").value))
        self.livox_min_range = float(self.get_parameter("livox_min_range").value)
        self.livox_max_range = float(self.get_parameter("livox_max_range").value)
        self.livox_horizontal_min = math.radians(float(self.get_parameter("livox_horizontal_min_deg").value))
        self.livox_horizontal_max = math.radians(float(self.get_parameter("livox_horizontal_max_deg").value))
        self.livox_vertical_min = math.radians(float(self.get_parameter("livox_vertical_min_deg").value))
        self.livox_vertical_max = math.radians(float(self.get_parameter("livox_vertical_max_deg").value))
        self.use_mujoco_raycast_lidar = bool(self.get_parameter("use_mujoco_raycast_lidar").value)
        self.mujoco_lidar_socket = str(self.get_parameter("mujoco_lidar_socket").value)
        self.mujoco_pedestrian_socket = str(self.get_parameter("mujoco_pedestrian_socket").value)
        self.publish_lidar_tf = bool(self.get_parameter("publish_lidar_tf").value)
        self.use_mujoco_lidar_pose = bool(self.get_parameter("use_mujoco_lidar_pose").value)
        self.use_cmd_vel_lidar_fallback = bool(self.get_parameter("use_cmd_vel_lidar_fallback").value)
        self.require_exact_lidar_pose = bool(self.get_parameter("require_exact_lidar_pose").value)
        self.suppress_lidar_when_fallen = bool(self.get_parameter("suppress_lidar_when_fallen").value)
        self.fallen_tilt_threshold = float(self.get_parameter("fallen_tilt_threshold").value)
        self.upright_tilt_threshold = float(self.get_parameter("upright_tilt_threshold").value)
        self.lidar_resume_delay = float(self.get_parameter("lidar_resume_delay").value)
        self.livox_base_offset = (
            float(self.get_parameter("livox_base_offset_x").value),
            float(self.get_parameter("livox_base_offset_y").value),
            float(self.get_parameter("livox_base_offset_z").value),
        )
        self.pelvis_to_livox_z = float(self.get_parameter("pelvis_to_livox_z").value)
        self.mujoco_scene = str(self.get_parameter("mujoco_scene").value)
        self.mujoco_robot_dir = Path(str(self.get_parameter("mujoco_robot_dir").value))
        self.robot_head_top_height_m = float(self.get_parameter("robot_head_top_height_m").value)
        self.livox_height_above_ground_m = float(self.get_parameter("livox_height_above_ground_m").value)
        self.livox_points_per_packet = max(1, int(self.get_parameter("livox_scan_points").value))
        self.livox_scan_rays = self._build_livox_scan_rays(self.livox_points_per_packet)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_sensor_poses = {}
        self.last_mujoco_pose = None
        self.last_mujoco_pose_time = None
        self.last_base_pose = ((0.0, 0.0, 0.0), self._identity_matrix())
        self.last_base_yaw = 0.0
        self.last_odom_pose = None
        self.last_odom_time = None
        self.current_odom_twist = (0.0, 0.0, 0.0)
        self.mapping_lidar_fallen = False
        self.mapping_lidar_resume_time = None
        self.mapping_lidar_suppressed_count = 0
        self.mujoco_pose_count = 0
        self.cmd_vel_count = 0
        self.mujoco_lidar_count = 0
        self.mujoco_lidar_socket_rx = None
        self.mujoco_camera_socket_rx = None
        self.mujoco_pedestrian_socket_rx = None
        self.latest_mujoco_lidar_packet = None
        self.latest_mujoco_lidar_world_packet = None
        self.latest_mujoco_lidar_time = None
        self.latest_mujoco_lidar_sim_time = None
        self.latest_mujoco_lidar_acquisition_stamp = None
        self.dropped_mujoco_lidar_packets = 0
        self.latest_mujoco_color_data = None
        self.latest_mujoco_depth_data = None
        self.latest_mujoco_camera_time = None
        self.latest_mujoco_camera_sim_time = None
        self.latest_mujoco_camera_acquisition_stamp = None
        self.camera_data_stamp = None
        self.lidar_dead_reckon_x = 0.0
        self.lidar_dead_reckon_y = 0.0
        self.lidar_dead_reckon_yaw = 0.0
        self.robot_base_height = self.livox_height_above_ground_m - self.pelvis_to_livox_z
        self.last_lidar_pose_time = self.get_clock().now()
        self.current_cmd_vel = [0.0, 0.0, 0.0]
        self.scene_boxes, self.scene_planes = self._load_mujoco_scene_geometry()
        if self.use_mujoco_raycast_lidar:
            self._open_mujoco_lidar_socket()
        self._open_mujoco_pedestrian_socket()
        if self.publish_camera and self.use_mujoco_raycast_camera:
            self._open_mujoco_camera_socket()
        if self.publish_camera:
            data = self._latest_mujoco_camera_images()
            if data is not None:
                self.color_data, self.depth_data, self.camera_data_stamp = data
            else:
                self.color_data, self.depth_data = self._render_camera(self.camera_frame)
                self.camera_data_stamp = self.get_clock().now().to_msg()
        else:
            self.color_data = b""
            self.depth_data = b""
        self.depth_pointcloud_data = b""
        self.depth_pointcloud_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.livox_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.color_pub = self.create_publisher(Image, "/camera/camera/color/image_raw", sensor_qos)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", info_qos)
        self.depth_pub = self.create_publisher(Image, "/camera/camera/depth/image_rect_raw", sensor_qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/camera/depth/camera_info", info_qos)
        self.aligned_depth_pub = self.create_publisher(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", sensor_qos
        )
        self.aligned_depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", info_qos
        )
        self.depth_points_pub = self.create_publisher(PointCloud2, "/camera/camera/depth/color/points", sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "/camera/camera/imu", sensor_qos)
        self.accel_pub = self.create_publisher(Imu, "/camera/camera/accel/sample", sensor_qos)
        self.gyro_pub = self.create_publisher(Imu, "/camera/camera/gyro/sample", sensor_qos)
        self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", sensor_qos)
        self.lidar_world_pub = self.create_publisher(PointCloud2, "/livox/lidar_world", sensor_qos)
        self.livox_imu_pub = self.create_publisher(Imu, "/livox/imu", sensor_qos)
        self.livox_head_height_pub = self.create_publisher(Float32, "/livox/ground_to_head_top", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.mujoco_pedestrians_pub = self.create_publisher(String, "/mujoco/pedestrians", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, sensor_qos)
        if self.use_mujoco_lidar_pose:
            self.create_subscription(
                SportModeState,
                str(self.get_parameter("mujoco_state_topic").value),
                self._mujoco_state_callback,
                sensor_qos,
            )
        if self.use_cmd_vel_lidar_fallback:
            self.create_subscription(
                Twist,
                str(self.get_parameter("cmd_vel_topic").value),
                self._cmd_vel_callback,
                sensor_qos,
            )

        if self.publish_camera:
            self.create_timer(1.0 / float(self.get_parameter("camera_rate_hz").value), self._publish_camera)
        if self.publish_depth_pointcloud and self.pointcloud_rate_hz > 0.0:
            self.create_timer(1.0 / self.pointcloud_rate_hz, self._publish_depth_pointcloud)
        self.create_timer(1.0 / float(self.get_parameter("imu_rate_hz").value), self._publish_imu)
        self.create_timer(1.0 / self.lidar_rate_hz, self._publish_lidar)
        self.create_timer(1.0 / 20.0, self._publish_mujoco_pedestrians)
        self.create_timer(1.0 / 30.0, self._publish_base_tf)
        self.create_timer(2.0, self._diagnostics)
        self.get_logger().info(
            f"Publishing low-latency Livox with {self.livox_points_per_packet} rays/scan; camera={self.publish_camera}"
        )

    def _open_mujoco_lidar_socket(self) -> None:
        try:
            if os.path.exists(self.mujoco_lidar_socket):
                os.unlink(self.mujoco_lidar_socket)
            self.mujoco_lidar_socket_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.mujoco_lidar_socket_rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 262144)
            self.mujoco_lidar_socket_rx.bind(self.mujoco_lidar_socket)
            self.mujoco_lidar_socket_rx.setblocking(False)
            self.get_logger().info(f"Receiving MuJoCo raycast LiDAR on {self.mujoco_lidar_socket}")
        except OSError as exc:
            self.mujoco_lidar_socket_rx = None
            self.get_logger().warning(f"Could not open MuJoCo LiDAR socket: {exc}")

    def _open_mujoco_camera_socket(self) -> None:
        try:
            if os.path.exists(self.mujoco_camera_socket):
                os.unlink(self.mujoco_camera_socket)
            self.mujoco_camera_socket_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.mujoco_camera_socket_rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1048576)
            self.mujoco_camera_socket_rx.bind(self.mujoco_camera_socket)
            self.mujoco_camera_socket_rx.setblocking(False)
            self.get_logger().info(f"Receiving MuJoCo raycast camera on {self.mujoco_camera_socket}")
        except OSError as exc:
            self.mujoco_camera_socket_rx = None
            self.get_logger().warning(f"Could not open MuJoCo camera socket: {exc}")

    def _open_mujoco_pedestrian_socket(self) -> None:
        try:
            if os.path.exists(self.mujoco_pedestrian_socket):
                os.unlink(self.mujoco_pedestrian_socket)
            self.mujoco_pedestrian_socket_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.mujoco_pedestrian_socket_rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            self.mujoco_pedestrian_socket_rx.bind(self.mujoco_pedestrian_socket)
            self.mujoco_pedestrian_socket_rx.setblocking(False)
            self.get_logger().info(f"Receiving MuJoCo pedestrian states on {self.mujoco_pedestrian_socket}")
        except OSError as exc:
            self.mujoco_pedestrian_socket_rx = None
            self.get_logger().warning(f"Could not open MuJoCo pedestrian socket: {exc}")

    def destroy_node(self) -> bool:
        if self.mujoco_lidar_socket_rx is not None:
            self.mujoco_lidar_socket_rx.close()
            self.mujoco_lidar_socket_rx = None
        if self.mujoco_camera_socket_rx is not None:
            self.mujoco_camera_socket_rx.close()
            self.mujoco_camera_socket_rx = None
        if self.mujoco_pedestrian_socket_rx is not None:
            self.mujoco_pedestrian_socket_rx.close()
            self.mujoco_pedestrian_socket_rx = None
        if self.use_mujoco_raycast_lidar and os.path.exists(self.mujoco_lidar_socket):
            try:
                os.unlink(self.mujoco_lidar_socket)
            except OSError:
                pass
        if self.use_mujoco_raycast_camera and os.path.exists(self.mujoco_camera_socket):
            try:
                os.unlink(self.mujoco_camera_socket)
            except OSError:
                pass
        if os.path.exists(self.mujoco_pedestrian_socket):
            try:
                os.unlink(self.mujoco_pedestrian_socket)
            except OSError:
                pass
        return super().destroy_node()

    def _publish_mujoco_pedestrians(self) -> None:
        if self.mujoco_pedestrian_socket_rx is None:
            return
        latest = None
        while True:
            try:
                latest = self.mujoco_pedestrian_socket_rx.recv(8192)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f"MuJoCo pedestrian socket read failed: {exc}", throttle_duration_sec=2.0)
                return
        if not latest:
            return
        if len(latest) < 16:
            return
        magic, count, sim_time = struct.unpack_from("<IId", latest, 0)
        if magic == 0x32444550:
            if len(latest) < 24:
                return
            acquisition_time_ns = struct.unpack_from("<q", latest, 16)[0]
            offset = 24
            stamp = acquisition_time_ns * 1e-9
        elif magic == 0x31444550:
            offset = 16
            stamp = self.get_clock().now().nanoseconds * 1e-9
        else:
            return
        people = []
        entry_size = struct.calcsize("<iddddd")
        for _ in range(min(count, 64)):
            if offset + entry_size > len(latest):
                break
            person_id, x, y, vx, vy, radius = struct.unpack_from("<iddddd", latest, offset)
            offset += entry_size
            people.append({"id": person_id, "x": x, "y": y, "vx": vx, "vy": vy, "radius_m": radius})
        msg = String()
        msg.data = json.dumps({"stamp": stamp, "sim_time": sim_time, "people": people})
        self.mujoco_pedestrians_pub.publish(msg)

    def _mujoco_state_callback(self, msg: SportModeState) -> None:
        self.mujoco_pose_count += 1
        quat = msg.imu_state.quaternion
        rotation = self._quat_wxyz_to_matrix((quat[0], quat[1], quat[2], quat[3]))
        origin = (
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        )
        self.last_mujoco_pose = self._lidar_pose_from_base_pose(origin, rotation)
        self.last_mujoco_pose_time = self.get_clock().now()
        self.last_base_pose = (origin, rotation)
        self._update_odometry_state(origin, rotation)
        self._update_mapping_lidar_state(rotation)
        if self.publish_lidar_tf:
            self._publish_lidar_tf(*self.last_mujoco_pose)

    def _publish_base_tf(self) -> None:
        self._publish_base_footprint_tf(*self.last_base_pose)
        self._publish_odometry(*self.last_base_pose)

    def _update_odometry_state(self, base_origin: tuple, base_rotation: tuple) -> None:
        now = self.get_clock().now()
        yaw = self._stable_base_yaw(base_rotation)
        if self.last_odom_pose is not None and self.last_odom_time is not None:
            dt = (now - self.last_odom_time).nanoseconds * 1e-9
            if dt > 1e-4:
                prev_x, prev_y, prev_yaw = self.last_odom_pose
                dx = base_origin[0] - prev_x
                dy = base_origin[1] - prev_y
                world_vx = dx / dt
                world_vy = dy / dt
                body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
                body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
                wz = self._wrap_pi(yaw - prev_yaw) / dt
                self.current_odom_twist = (body_vx, body_vy, wz)
        self.last_odom_pose = (base_origin[0], base_origin[1], yaw)
        self.last_odom_time = now

    def _publish_odometry(self, base_origin: tuple, base_rotation: tuple) -> None:
        yaw = self._stable_base_yaw(base_rotation)
        yaw_rotation = self._yaw_to_matrix(yaw)
        qx, qy, qz, qw = self._matrix_to_quat_xyzw(yaw_rotation)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = float(base_origin[0])
        odom.pose.pose.position.y = float(base_origin[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.current_odom_twist[0]
        odom.twist.twist.linear.y = self.current_odom_twist[1]
        odom.twist.twist.angular.z = self.current_odom_twist[2]
        self.odom_pub.publish(odom)

    def _publish_base_footprint_tf(self, base_origin: tuple, base_rotation: tuple, stamp=None) -> None:
        yaw = self._stable_base_yaw(base_rotation)
        yaw_rotation = self._yaw_to_matrix(yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = base_origin[0]
        transform.transform.translation.y = base_origin[1]
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = self._matrix_to_quat_xyzw(yaw_rotation)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _stable_base_yaw(self, base_rotation: tuple) -> float:
        if not self.mapping_lidar_fallen and self._base_tilt(base_rotation) < self.fallen_tilt_threshold:
            self.last_base_yaw = math.atan2(base_rotation[1][0], base_rotation[0][0])
        return self.last_base_yaw

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def _update_mapping_lidar_state(self, base_rotation: tuple) -> None:
        if not self.suppress_lidar_when_fallen:
            self.mapping_lidar_fallen = False
            self.mapping_lidar_resume_time = None
            return
        tilt = self._base_tilt(base_rotation)
        now = self.get_clock().now()
        if not self.mapping_lidar_fallen and tilt > self.fallen_tilt_threshold:
            self.mapping_lidar_fallen = True
            self.mapping_lidar_resume_time = None
            self._drop_cached_lidar_packets()
            self.get_logger().warn(
                f"Robot tilt {tilt:.2f} rad; pausing LiDAR mapping to protect SLAM occupancy grid",
                throttle_duration_sec=1.0,
            )
            return
        if self.mapping_lidar_fallen and tilt < self.upright_tilt_threshold:
            self.mapping_lidar_fallen = False
            self.mapping_lidar_resume_time = now + Duration(seconds=self.lidar_resume_delay)
            self._drop_cached_lidar_packets()
            self.get_logger().info(
                f"Robot upright again; waiting {self.lidar_resume_delay:.1f}s before resuming LiDAR mapping"
            )

    def _mapping_lidar_suppressed(self) -> bool:
        if not self.suppress_lidar_when_fallen:
            return False
        if self.mapping_lidar_fallen:
            return True
        if self.mapping_lidar_resume_time is None:
            return False
        if (self.mapping_lidar_resume_time - self.get_clock().now()).nanoseconds > 0:
            return True
        self.mapping_lidar_resume_time = None
        self._drop_cached_lidar_packets()
        self.get_logger().info("Resuming LiDAR mapping after fall recovery stabilization")
        return False

    def _drop_cached_lidar_packets(self) -> None:
        self.latest_mujoco_lidar_packet = None
        self.latest_mujoco_lidar_world_packet = None
        self.latest_mujoco_lidar_time = None
        self.latest_mujoco_lidar_acquisition_stamp = None

    @staticmethod
    def _base_tilt(base_rotation: tuple) -> float:
        up_z = max(-1.0, min(1.0, base_rotation[2][2]))
        return math.acos(up_z)

    def _lidar_pose_from_base_pose(self, base_origin: tuple, base_rotation: tuple) -> tuple:
        offset = self._rotate(base_rotation, self.livox_base_offset)
        origin = (
            base_origin[0] + offset[0],
            base_origin[1] + offset[1],
            base_origin[2] + offset[2],
        )
        rotation = self._matmul(base_rotation, self._livox_default_rotation())
        return origin, rotation

    def _base_pose_from_lidar_pose(self, lidar_origin: tuple, lidar_rotation: tuple) -> tuple:
        base_rotation = self._matmul(lidar_rotation, self._transpose(self._livox_default_rotation()))
        offset = self._rotate(base_rotation, self.livox_base_offset)
        base_origin = (
            lidar_origin[0] - offset[0],
            lidar_origin[1] - offset[1],
            lidar_origin[2] - offset[2],
        )
        return base_origin, base_rotation

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_count += 1
        self.current_cmd_vel = [
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        ]

    def _joint_state_callback(self, msg: JointState) -> None:
        try:
            index = msg.name.index("floating_base_joint")
        except ValueError:
            return
        if index < len(msg.position):
            self.robot_base_height = float(msg.position[index])

    def _diagnostics(self) -> None:
        if self.use_mujoco_raycast_lidar and self.mujoco_lidar_count == 0:
            self.get_logger().warn(
                "No MuJoCo raycast LiDAR packets received; LiDAR is using fallback geometry.",
                throttle_duration_sec=5.0,
            )
        self.get_logger().info(
            f"LiDAR pose status: raycast_packets={self.mujoco_lidar_count} dropped_stale={self.dropped_mujoco_lidar_packets} suppressed={self.mapping_lidar_suppressed_count} fallen={self.mapping_lidar_fallen} sport_pose={self.mujoco_pose_count} cmd_vel={self.cmd_vel_count} frame={self.lidar_frame}",
            throttle_duration_sec=5.0,
        )

    def _publish_lidar_tf(self, origin: tuple, rotation: tuple, stamp=None) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = self.lidar_frame
        transform.transform.translation.x = origin[0]
        transform.transform.translation.y = origin[1]
        transform.transform.translation.z = origin[2]
        qx, qy, qz, qw = self._matrix_to_quat_xyzw(rotation)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _header(self, frame_id: str, stamp=None) -> Header:
        header = Header()
        header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _camera_info(self, header: Header) -> CameraInfo:
        info = CameraInfo()
        info.header = header
        info.width = self.width
        info.height = self.height
        fov_y = math.radians(58.0)
        fx = fy = (self.height * 0.5) / math.tan(fov_y * 0.5)
        cx = (self.width - 1) * 0.5
        cy = (self.height - 1) * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = "plumb_bob"
        return info

    def _publish_camera(self) -> None:
        data = self._latest_mujoco_camera_images()
        if data is not None:
            self.color_data, self.depth_data, stamp = data
        else:
            self.color_data, self.depth_data = self._render_camera(self.camera_frame)
            stamp = self.get_clock().now().to_msg()
        self.camera_data_stamp = stamp
        color_header = Header()
        color_header.stamp = stamp
        color_header.frame_id = self.camera_frame
        depth_header = Header()
        depth_header.stamp = stamp
        depth_header.frame_id = self.depth_frame

        color = Image()
        color.header = color_header
        color.width = self.width
        color.height = self.height
        color.encoding = "rgb8"
        color.is_bigendian = False
        color.step = self.width * 3
        color.data = self.color_data

        depth = Image()
        depth.header = depth_header
        depth.width = self.width
        depth.height = self.height
        depth.encoding = "16UC1"
        depth.is_bigendian = False
        depth.step = self.width * 2
        depth.data = self.depth_data

        aligned_depth = Image()
        aligned_depth.header = color_header
        aligned_depth.width = self.width
        aligned_depth.height = self.height
        aligned_depth.encoding = "16UC1"
        aligned_depth.is_bigendian = False
        aligned_depth.step = self.width * 2
        aligned_depth.data = self.depth_data

        self.color_pub.publish(color)
        self.color_info_pub.publish(self._camera_info(color_header))
        self.depth_pub.publish(depth)
        self.depth_info_pub.publish(self._camera_info(depth_header))
        self.aligned_depth_pub.publish(aligned_depth)
        self.aligned_depth_info_pub.publish(self._camera_info(color_header))

    def _latest_mujoco_camera_images(self):
        if self.mujoco_camera_socket_rx is None:
            return None

        latest_packet = None
        while True:
            try:
                packet = self.mujoco_camera_socket_rx.recv(1048576)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f"MuJoCo camera socket receive failed: {exc}", throttle_duration_sec=2.0)
                return None

            latest_packet = packet

        if latest_packet is not None:
            decoded = self._decode_mujoco_camera_packet(latest_packet)
            if decoded is not None:
                color, depth, acquisition_stamp = decoded
                self.latest_mujoco_color_data = color
                self.latest_mujoco_depth_data = depth
                self.latest_mujoco_camera_acquisition_stamp = acquisition_stamp
                self.latest_mujoco_camera_time = self.get_clock().now()

        if self.latest_mujoco_color_data is None or self.latest_mujoco_camera_time is None:
            return None

        age = (self.get_clock().now() - self.latest_mujoco_camera_time).nanoseconds * 1e-9
        if age > 0.5:
            return None
        return (
            self.latest_mujoco_color_data,
            self.latest_mujoco_depth_data,
            self.latest_mujoco_camera_acquisition_stamp,
        )

    def _decode_mujoco_camera_packet(self, packet: bytes):
        decoded = decode_camera_packet(packet)
        if decoded is None:
            return None
        if decoded.width != self.width or decoded.height != self.height:
            self.get_logger().warning(
                f"Ignoring MuJoCo camera frame {decoded.width}x{decoded.height}; ROS camera is {self.width}x{self.height}",
                throttle_duration_sec=5.0,
            )
            return None
        color_size = self.width * self.height * 3
        depth_size = self.width * self.height * 2
        receipt_time = self.get_clock().now()
        acquisition_stamp = self._acquisition_stamp(decoded.acquisition_time_ns, receipt_time)
        if decoded.origin:
            rotation = self._quat_wxyz_to_matrix(decoded.quaternion_wxyz)
            if self.latest_mujoco_camera_sim_time is not None and decoded.simulation_time + 1e-9 < self.latest_mujoco_camera_sim_time:
                self.get_logger().info("MuJoCo camera sim time reset detected; using newest camera pose")
            self.latest_mujoco_camera_sim_time = decoded.simulation_time
            self._publish_camera_tf(decoded.origin, rotation, acquisition_stamp)
        color = decoded.payload[:color_size]
        depth = decoded.payload[color_size:color_size + depth_size]
        return color, depth, acquisition_stamp

    def _publish_camera_tf(self, origin: tuple, rotation: tuple, stamp=None) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = origin[0]
        transform.transform.translation.y = origin[1]
        transform.transform.translation.z = origin[2]
        qx, qy, qz, qw = self._matrix_to_quat_xyzw(rotation)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_depth_pointcloud(self) -> None:
        if not self.depth_data or not self.color_data:
            return
        self.depth_pointcloud_data = self._build_depth_pointcloud_from_depth()
        cloud = PointCloud2()
        cloud.header = self._header(self.camera_frame, self.camera_data_stamp)
        cloud.height = self.height
        cloud.width = self.width
        cloud.fields = self.depth_pointcloud_fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = self.depth_pointcloud_data
        self.depth_points_pub.publish(cloud)

    def _publish_imu(self) -> None:
        imu = Imu()
        imu.header = self._header(self.imu_frame)
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration.z = 9.80665
        imu.linear_acceleration_covariance[0] = 0.05
        imu.linear_acceleration_covariance[4] = 0.05
        imu.linear_acceleration_covariance[8] = 0.05
        self.imu_pub.publish(imu)

        accel = Imu()
        accel.header = imu.header
        accel.orientation_covariance[0] = -1.0
        accel.linear_acceleration.z = 9.80665
        accel.linear_acceleration_covariance[0] = 0.05
        accel.linear_acceleration_covariance[4] = 0.05
        accel.linear_acceleration_covariance[8] = 0.05
        self.accel_pub.publish(accel)

        gyro = Imu()
        gyro.header = imu.header
        gyro.orientation_covariance[0] = -1.0
        gyro.angular_velocity_covariance[0] = 0.01
        gyro.angular_velocity_covariance[4] = 0.01
        gyro.angular_velocity_covariance[8] = 0.01
        self.gyro_pub.publish(gyro)

        livox_imu = Imu()
        livox_imu.header = self._header(self.lidar_frame)
        livox_imu.orientation.w = 1.0
        livox_imu.orientation_covariance[0] = 0.01
        livox_imu.orientation_covariance[4] = 0.01
        livox_imu.orientation_covariance[8] = 0.01
        livox_imu.angular_velocity_covariance[0] = 0.01
        livox_imu.angular_velocity_covariance[4] = 0.01
        livox_imu.angular_velocity_covariance[8] = 0.01
        livox_imu.linear_acceleration.z = 9.80665
        livox_imu.linear_acceleration_covariance[0] = 0.05
        livox_imu.linear_acceleration_covariance[4] = 0.05
        livox_imu.linear_acceleration_covariance[8] = 0.05
        self.livox_imu_pub.publish(livox_imu)

    def _publish_lidar(self) -> None:
        started = self.get_clock().now()
        cloud_data = self._latest_mujoco_lidar_packet()
        acquisition_stamp = self.latest_mujoco_lidar_acquisition_stamp if cloud_data is not None else None
        if self._mapping_lidar_suppressed():
            self.mapping_lidar_suppressed_count += 1
            self._drop_cached_lidar_packets()
            self.get_logger().warn(
                "LiDAR mapping paused during fall/get-up recovery; not publishing scan source clouds",
                throttle_duration_sec=2.0,
            )
            return
        if cloud_data is None and self.use_mujoco_lidar_pose and self.last_mujoco_pose is None:
            if self.require_exact_lidar_pose:
                self.get_logger().warn(
                    "Skipping LiDAR packet until exact Mujoco Livox pose is available.",
                    throttle_duration_sec=2.0,
                )
                return
            if self.use_cmd_vel_lidar_fallback:
                self._update_dead_reckoned_lidar_pose(started)
        if cloud_data is None:
            cloud_data = self._build_livox_mid360_packet(self.livox_points_per_packet)
        cloud = PointCloud2()
        cloud.header = self._header(self.lidar_output_frame, acquisition_stamp)
        cloud.height = 1
        cloud.width = len(cloud_data) // 16
        cloud.fields = self.livox_fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = cloud_data
        self.lidar_pub.publish(cloud)
        if (
            acquisition_stamp is not None
            and self.latest_mujoco_lidar_world_packet is not None
            and self.lidar_world_pub.get_subscription_count() > 0
        ):
            world_cloud = PointCloud2()
            world_cloud.header = self._header("world", acquisition_stamp)
            world_cloud.height = 1
            world_cloud.width = len(self.latest_mujoco_lidar_world_packet) // 16
            world_cloud.fields = self.livox_fields
            world_cloud.is_bigendian = False
            world_cloud.point_step = 16
            world_cloud.row_step = world_cloud.point_step * world_cloud.width
            world_cloud.is_dense = True
            world_cloud.data = self.latest_mujoco_lidar_world_packet
            self.lidar_world_pub.publish(world_cloud)

        height = Float32()
        height.data = self.robot_head_top_height_m
        self.livox_head_height_pub.publish(height)
        elapsed = (self.get_clock().now() - started).nanoseconds * 1e-9
        period = 1.0 / max(self.lidar_rate_hz, 1e-6)
        if elapsed > period:
            self.get_logger().warn(
                f"Livox packet generation took {elapsed:.3f}s > {period:.3f}s period; reduce livox_scan_points",
                throttle_duration_sec=2.0,
            )

    def _latest_mujoco_lidar_packet(self):
        if self.mujoco_lidar_socket_rx is None:
            return None

        latest_packet = None
        drained_packets = 0
        while True:
            try:
                packet = self.mujoco_lidar_socket_rx.recv(262144)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f"MuJoCo LiDAR socket receive failed: {exc}", throttle_duration_sec=2.0)
                return None

            latest_packet = packet
            drained_packets += 1

        if latest_packet is not None:
            cloud_data = self._decode_mujoco_lidar_packet(latest_packet)
            if cloud_data is not None:
                self.latest_mujoco_lidar_packet = cloud_data
                self.latest_mujoco_lidar_time = self.get_clock().now()
                self.mujoco_lidar_count += 1
                if drained_packets > 1:
                    self.dropped_mujoco_lidar_packets += drained_packets - 1

        if self.latest_mujoco_lidar_packet is None or self.latest_mujoco_lidar_time is None:
            return None

        age = (self.get_clock().now() - self.latest_mujoco_lidar_time).nanoseconds * 1e-9
        if age > max(0.5, 3.0 / max(self.lidar_rate_hz, 1e-6)):
            return None
        return self.latest_mujoco_lidar_packet

    def _decode_mujoco_lidar_packet(self, packet: bytes):
        decoded = decode_lidar_packet(packet)
        if decoded is None:
            return None
        payload = decoded.payload
        origin = decoded.origin
        ox, oy, oz = origin
        rotation = self._quat_wxyz_to_matrix(decoded.quaternion_wxyz)
        receipt_time = self.get_clock().now()
        acquisition_stamp = self._acquisition_stamp(decoded.acquisition_time_ns, receipt_time)
        if self.latest_mujoco_lidar_sim_time is not None and decoded.simulation_time + 1e-9 < self.latest_mujoco_lidar_sim_time:
            self.latest_mujoco_lidar_world_packet = None
            self.get_logger().info("MuJoCo LiDAR sim time reset detected; dropped stale world-frame packet")
        self.latest_mujoco_lidar_sim_time = decoded.simulation_time
        self.latest_mujoco_lidar_acquisition_stamp = acquisition_stamp
        self.last_mujoco_pose = (origin, rotation)
        self.last_mujoco_pose_time = receipt_time
        self.last_base_pose = self._base_pose_from_lidar_pose(origin, rotation)
        self._update_mapping_lidar_state(self.last_base_pose[1])
        if self.publish_lidar_tf:
            self._publish_lidar_tf(origin, rotation, acquisition_stamp)
            self._publish_base_footprint_tf(*self.last_base_pose, acquisition_stamp)
        publish_world = self.lidar_output_frame == "world" or self.lidar_world_pub.get_subscription_count() > 0
        if publish_world:
            world_packet = bytearray(decoded.count * 16)
            for index in range(decoded.count):
                offset = index * 16
                local_x, local_y, local_z, intensity = struct.unpack_from("<ffff", payload, offset)
                world_x, world_y, world_z = self._rotate(rotation, (local_x, local_y, local_z))
                struct.pack_into(
                    "<ffff",
                    world_packet,
                    offset,
                    world_x + ox,
                    world_y + oy,
                    world_z + oz,
                    intensity,
                )
            self.latest_mujoco_lidar_world_packet = bytes(world_packet)
            if self.lidar_output_frame == "world":
                return self.latest_mujoco_lidar_world_packet
        else:
            self.latest_mujoco_lidar_world_packet = None
        return payload

    @staticmethod
    def _acquisition_stamp(acquisition_time_ns, receipt_time: Time):
        if acquisition_time_ns is None or acquisition_time_ns <= 0:
            return receipt_time.to_msg()
        return Time(nanoseconds=int(acquisition_time_ns)).to_msg()

    def _build_livox_mid360_packet(self, count: int) -> bytes:
        # MID-360 is a non-repetitive scanner. This deterministic low-discrepancy
        # pattern gives full 360 x 59 deg coverage without ring artifacts, then
        # casts those rays into a simple room so RViz shows wall/object returns.
        packet = bytearray()
        origin, rotation = self._lookup_sensor_pose(self.lidar_frame)
        for index, local_direction in enumerate(self.livox_scan_rays):
            world_direction = self._rotate(rotation, local_direction)
            hit = self._raycast_livox_scene(origin, world_direction, self.scene_boxes, self.scene_planes)
            if hit is None:
                continue

            radius, world_x, world_y, world_z = hit
            if radius < self.livox_min_range or radius > self.livox_max_range:
                continue

            if self.lidar_output_frame == "world":
                x, y, z = world_x, world_y, world_z
            else:
                x, y, z = self._world_to_sensor((world_x, world_y, world_z), origin, rotation)

            intensity = 30.0 + 70.0 * (1.0 - min(radius, self.livox_max_range) / self.livox_max_range)
            packet.extend(struct.pack("<ffff", x, y, z, intensity))

        packet.extend(self._build_livox_object_surface_returns(origin, rotation, start_index=len(packet) // 16))
        packet.extend(self._build_livox_floor_returns(origin, rotation, start_index=len(packet) // 16))
        packet.extend(self._build_livox_wall_returns(origin, rotation, start_index=len(packet) // 16))
        return bytes(packet)

    def _build_livox_scan_rays(self, count: int) -> list:
        rays = []
        golden_ratio = (1.0 + math.sqrt(5.0)) / 2.0
        horizontal_span = self.livox_horizontal_max - self.livox_horizontal_min
        vertical_span = self.livox_vertical_max - self.livox_vertical_min
        for index in range(count):
            azimuth = self.livox_horizontal_min + horizontal_span * self._fract(index / golden_ratio)
            vertical = self.livox_vertical_min + vertical_span * ((index + 0.5) / count)
            cos_v = math.cos(vertical)
            rays.append((cos_v * math.cos(azimuth), cos_v * math.sin(azimuth), math.sin(vertical)))
        return rays

    def _load_mujoco_scene_geometry(self) -> tuple:
        scene_path = self.mujoco_robot_dir / self.mujoco_scene
        if not scene_path.exists():
            self.get_logger().warning(f"Mujoco scene not found for Livox geometry: {scene_path}")
            return [], self._default_floor_planes()

        boxes = []
        planes = []
        try:
            root = ET.parse(scene_path).getroot()
        except ET.ParseError as exc:
            self.get_logger().warning(f"Could not parse Mujoco scene for Livox geometry: {exc}")
            return [], self._default_floor_planes()

        worldbody = root.find("worldbody")
        if worldbody is None:
            return [], self._default_floor_planes()

        for geom in worldbody.findall("geom"):
            geom_type = geom.get("type", "sphere")
            pos = self._parse_vec(geom.get("pos"), (0.0, 0.0, 0.0))
            size = self._parse_vec(geom.get("size"), (0.0, 0.0, 0.0))

            if geom_type == "plane":
                z = pos[2]
                planes.append(("z", z, (-20.0, -20.0, z), (20.0, 20.0, z)))
            elif geom_type == "box":
                boxes.append(self._box_from_mujoco(pos, size))
                wall_plane = self._wall_plane_from_mujoco_box(pos, size)
                if wall_plane is not None:
                    planes.append(wall_plane)
            elif geom_type == "cylinder":
                radius = size[0]
                half_height = size[1] if len(size) > 1 else radius
                boxes.append(self._box_from_mujoco(pos, (radius, radius, half_height)))

        return boxes, planes or self._default_floor_planes()

    def _default_floor_planes(self) -> list:
        z = 0.0
        return [("z", z, (-20.0, -20.0, z), (20.0, 20.0, z))]

    def _box_from_mujoco(self, pos: tuple, size: tuple) -> tuple:
        x, y, z = pos
        sx, sy, sz = size[:3]
        return ((x - sx, y - sy, z - sz), (x + sx, y + sy, z + sz))

    def _wall_plane_from_mujoco_box(self, pos: tuple, size: tuple):
        x, y, z = pos
        sx, sy, sz = size[:3]
        if sz < 0.5 or min(sx, sy) > 0.08:
            return None

        if sx <= sy:
            # Thin in X: expose the side facing the lidar origin.
            value = x - sx if x >= 0.0 else x + sx
            return ("x", value, (value, y - sy, z - sz), (value, y + sy, z + sz))

        value = y - sy if y >= 0.0 else y + sy
        return ("y", value, (x - sx, value, z - sz), (x + sx, value, z + sz))

    @staticmethod
    def _parse_vec(value: str, default: tuple) -> tuple:
        if not value:
            return default
        parts = [float(part) for part in value.split()]
        while len(parts) < len(default):
            parts.append(0.0)
        return tuple(parts)

    def _raycast_livox_scene(self, origin: tuple, direction: tuple, boxes: list, planes: list):
        best = None
        for box_min, box_max in boxes:
            distance = self._intersect_box(origin, direction, box_min, box_max)
            if distance is not None and (best is None or distance < best):
                best = distance

        for axis, value, bounds_min, bounds_max in planes:
            distance = self._intersect_plane(origin, direction, axis, value, bounds_min, bounds_max)
            if distance is not None and (best is None or distance < best):
                best = distance

        if best is None:
            return None

        x = origin[0] + direction[0] * best
        y = origin[1] + direction[1] * best
        z = origin[2] + direction[2] * best
        return best, x, y, z

    def _intersect_plane(self, origin: tuple, direction: tuple, axis: str, value: float, bounds_min: tuple, bounds_max: tuple):
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        axis_direction = direction[axis_index]
        if abs(axis_direction) < 1e-6:
            return None

        distance = (value - origin[axis_index]) / axis_direction
        if distance <= 0.0:
            return None

        point = (origin[0] + direction[0] * distance, origin[1] + direction[1] * distance, origin[2] + direction[2] * distance)
        if any(point[i] < bounds_min[i] - 1e-6 or point[i] > bounds_max[i] + 1e-6 for i in range(3)):
            return None
        return distance

    @staticmethod
    def _intersect_box(origin: tuple, direction: tuple, box_min: tuple, box_max: tuple):
        t_min = -math.inf
        t_max = math.inf
        for axis in range(3):
            component = direction[axis]
            if abs(component) < 1e-6:
                if box_min[axis] <= origin[axis] <= box_max[axis]:
                    continue
                return None

            t1 = (box_min[axis] - origin[axis]) / component
            t2 = (box_max[axis] - origin[axis]) / component
            near = min(t1, t2)
            far = max(t1, t2)
            t_min = max(t_min, near)
            t_max = min(t_max, far)
            if t_min > t_max:
                return None

        if t_max <= 0.0:
            return None
        return t_min if t_min > 0.0 else t_max

    def _build_livox_object_surface_returns(self, origin: tuple, rotation: tuple, start_index: int = 0) -> bytes:
        if self.livox_object_surface_samples <= 0:
            return b""
        packet = bytearray()
        samples = self.livox_object_surface_samples * self.livox_object_surface_samples
        point_index = start_index
        for box_min, box_max in self.scene_boxes:
            x0, y0, z0 = box_min
            x1, y1, z1 = box_max
            faces = [
                ("x", x0, y0, y1, z0, z1),
                ("x", x1, y0, y1, z0, z1),
                ("y", y0, x0, x1, z0, z1),
                ("y", y1, x0, x1, z0, z1),
                ("z", z1, x0, x1, y0, y1),
            ]
            for axis, fixed, a0, a1, b0, b1 in faces:
                for sample in range(samples):
                    a_fraction, b_fraction = self._surface_sample(sample + point_index)
                    a = a0 + (a1 - a0) * a_fraction
                    b = b0 + (b1 - b0) * b_fraction
                    if axis == "x":
                        x, y, z = fixed, a, b
                    elif axis == "y":
                        x, y, z = a, fixed, b
                    else:
                        x, y, z = a, b, fixed
                    radius = self._distance(origin, (x, y, z))
                    if radius < self.livox_min_range or radius > self.livox_max_range:
                        continue
                    if not self._has_line_of_sight(origin, (x, y, z)):
                        continue
                    if self.lidar_output_frame == "world":
                        local_x, local_y, local_z = x, y, z
                    else:
                        local_x, local_y, local_z = self._world_to_sensor((x, y, z), origin, rotation)
                    intensity = 120.0
                    packet.extend(struct.pack("<ffff", local_x, local_y, local_z, intensity))
                    point_index += 1
        return bytes(packet)

    def _build_livox_floor_returns(self, origin: tuple, rotation: tuple, start_index: int = 0) -> bytes:
        if self.livox_floor_samples <= 0:
            return b""
        packet = bytearray()
        point_index = start_index
        for axis, value, bounds_min, bounds_max in self.scene_planes:
            if axis != "z" or value > -0.1:
                continue

            x_min = max(bounds_min[0], -1.0)
            x_max = min(bounds_max[0], 5.0)
            y_min = max(bounds_min[1], -3.0)
            y_max = min(bounds_max[1], 3.0)
            samples = self.livox_floor_samples * self.livox_floor_samples
            for sample in range(samples):
                x_fraction, y_fraction = self._surface_sample(sample)
                x = x_min + (x_max - x_min) * x_fraction
                y = y_min + (y_max - y_min) * y_fraction
                point = (x, y, value)
                radius = self._distance(origin, point)
                if radius < self.livox_min_range or radius > self.livox_max_range:
                    continue
                if not self._has_line_of_sight(origin, point):
                    continue
                if self.lidar_output_frame == "world":
                    local_x, local_y, local_z = point
                else:
                    local_x, local_y, local_z = self._world_to_sensor(point, origin, rotation)
                intensity = 85.0
                packet.extend(struct.pack("<ffff", local_x, local_y, local_z, intensity))
                point_index += 1
        return bytes(packet)

    def _build_livox_wall_returns(self, origin: tuple, rotation: tuple, start_index: int = 0) -> bytes:
        if self.livox_wall_horizontal_samples <= 0 or self.livox_wall_vertical_samples <= 0:
            return b""
        packet = bytearray()
        point_index = start_index
        for axis, value, bounds_min, bounds_max in self.scene_planes:
            if axis == "z":
                continue

            samples = self.livox_wall_horizontal_samples * self.livox_wall_vertical_samples
            for sample in range(samples):
                h_fraction, z_fraction = self._surface_sample(sample + point_index)
                z = bounds_min[2] + (bounds_max[2] - bounds_min[2]) * z_fraction
                if axis == "x":
                    y = bounds_min[1] + (bounds_max[1] - bounds_min[1]) * h_fraction
                    point = (value, y, z)
                else:
                    x = bounds_min[0] + (bounds_max[0] - bounds_min[0]) * h_fraction
                    point = (x, value, z)

                radius = self._distance(origin, point)
                if radius < self.livox_min_range or radius > self.livox_max_range:
                    continue
                if not self._has_line_of_sight(origin, point):
                    continue
                if self.lidar_output_frame == "world":
                    local_x, local_y, local_z = point
                else:
                    local_x, local_y, local_z = self._world_to_sensor(point, origin, rotation)
                intensity = 95.0
                packet.extend(struct.pack("<ffff", local_x, local_y, local_z, intensity))
                point_index += 1
        return bytes(packet)

    @staticmethod
    def _fract(value: float) -> float:
        return value - math.floor(value)

    def _surface_sample(self, index: int) -> tuple:
        return self._fract(index * 0.7548776662466927 + 0.5), self._fract(index * 0.5698402909980532 + 0.5)

    def _has_line_of_sight(self, origin: tuple, point: tuple) -> bool:
        radius = self._distance(origin, point)
        if radius <= 1e-6:
            return False
        direction = ((point[0] - origin[0]) / radius, (point[1] - origin[1]) / radius, (point[2] - origin[2]) / radius)
        hit = self._raycast_livox_scene(origin, direction, self.scene_boxes, self.scene_planes)
        if hit is None:
            return False
        hit_radius, hit_x, hit_y, hit_z = hit
        if abs(hit_radius - radius) > 0.03:
            return False
        distance_error = math.sqrt((hit_x - point[0]) ** 2 + (hit_y - point[1]) ** 2 + (hit_z - point[2]) ** 2)
        return distance_error <= 0.03

    def _render_camera(self, frame_id: str) -> tuple:
        origin, rotation = self._lookup_sensor_pose(frame_id)
        color = bytearray(self.width * self.height * 3)
        depth = bytearray(self.width * self.height * 2)
        fx = fy = 615.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        step = self.camera_render_step
        for v in range(0, self.height, step):
            block_h = min(step, self.height - v)
            for u in range(0, self.width, step):
                block_w = min(step, self.width - u)
                x_right = (u - cx) / fx
                y_down = (v - cy) / fy
                norm = math.sqrt(x_right * x_right + y_down * y_down + 1.0)
                local_direction = (x_right / norm, y_down / norm, 1.0 / norm)
                world_direction = self._camera_optical_to_world(rotation, local_direction)
                hit = self._raycast_livox_scene(origin, world_direction, self.scene_boxes, self.scene_planes)
                if hit is None:
                    red = int(45 + 40 * u / max(1, self.width - 1))
                    green = int(75 + 55 * v / max(1, self.height - 1))
                    blue = 150
                    self._fill_camera_block(color, depth, u, v, block_w, block_h, (red, green, blue), 0)
                    continue

                radius, x, y, z = hit
                if abs(z) < 0.03:
                    shade = max(60, int(180 - radius * 18))
                    pixel = (shade, shade, shade)
                elif z > 0.2 or abs(y) > 1.8 or x > 3.2:
                    shade = max(70, int(210 - radius * 20))
                    pixel = (int(shade * 0.65), int(shade * 0.85), shade)
                else:
                    shade = max(80, int(240 - radius * 25))
                    pixel = (shade, int(shade * 0.65), 40)

                local_point = self._world_to_camera_optical((x, y, z), origin, rotation)
                depth_m = max(0.0, local_point[2])
                depth_mm = min(65535, int(depth_m * 1000.0))
                self._fill_camera_block(color, depth, u, v, block_w, block_h, pixel, depth_mm)

        return bytes(color), bytes(depth)

    def _fill_camera_block(self, color: bytearray, depth: bytearray, u: int, v: int, width: int, height: int, pixel: tuple, depth_mm: int) -> None:
        depth_data = struct.pack("<H", depth_mm)
        color_data = bytes(pixel) * width
        depth_row_data = depth_data * width
        for row in range(v, v + height):
            color_row = row * self.width * 3
            depth_row = row * self.width * 2
            color_offset = color_row + u * 3
            depth_offset = depth_row + u * 2
            color[color_offset:color_offset + width * 3] = color_data
            depth[depth_offset:depth_offset + width * 2] = depth_row_data

    def _build_test_pattern_image(self) -> bytes:
        data = bytearray()
        for v in range(self.height):
            for u in range(self.width):
                checker = 48 if ((u // 40) + (v // 40)) % 2 else 0
                red = int(80 + 175 * u / max(1, self.width - 1))
                green = int(80 + 175 * v / max(1, self.height - 1))
                blue = 40 + checker
                data.extend((red, green, min(255, blue)))
        return bytes(data)

    def _build_depth_pointcloud_from_depth(self) -> bytes:
        packet = bytearray(self.width * self.height * 16)
        fx = fy = 615.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        invalid = struct.pack("<ffff", float("nan"), float("nan"), float("nan"), 0.0)
        for v in range(self.height):
            depth_row = v * self.width * 2
            points_row = v * self.width * 16
            y_factor = (v - cy) / fy
            for u in range(self.width):
                depth_offset = depth_row + u * 2
                depth_mm = struct.unpack_from("<H", self.depth_data, depth_offset)[0]
                points_offset = points_row + u * 16
                if depth_mm == 0:
                    packet[points_offset:points_offset + 16] = invalid
                    continue

                z = depth_mm / 1000.0
                x = (u - cx) * z / fx
                y = y_factor * z
                color_offset = (v * self.width + u) * 3
                red = self.color_data[color_offset]
                green = self.color_data[color_offset + 1]
                blue = self.color_data[color_offset + 2]
                rgb = struct.unpack("<f", struct.pack("<I", (red << 16) | (green << 8) | blue))[0]
                packet[points_offset:points_offset + 16] = struct.pack("<ffff", x, y, z, rgb)
        return bytes(packet)

    def _lookup_sensor_pose(self, frame_id: str) -> tuple:
        if frame_id == self.lidar_frame and self.use_mujoco_lidar_pose and self.last_mujoco_pose is not None:
            return self.last_mujoco_pose
        if frame_id == self.lidar_frame and self.use_mujoco_lidar_pose:
            pose = self._dead_reckoned_lidar_pose()
            self.last_sensor_poses[frame_id] = pose
            return pose
        try:
            transform = self.tf_buffer.lookup_transform("world", frame_id, Time(), timeout=Duration(seconds=0.0))
            translation = transform.transform.translation
            rotation = self._quat_to_matrix(transform.transform.rotation)
            pose = (translation.x, translation.y, translation.z), rotation
            self.last_sensor_poses[frame_id] = pose
            return pose
        except TransformException:
            if frame_id in self.last_sensor_poses:
                return self.last_sensor_poses[frame_id]
            if frame_id == self.lidar_frame:
                return (0.0, 0.0, self.livox_height_above_ground_m), self._livox_default_rotation()
            return (0.0, 0.0, self.livox_height_above_ground_m), self._camera_default_rotation()

    @staticmethod
    def _quat_to_matrix(q) -> tuple:
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    @staticmethod
    def _quat_wxyz_to_matrix(q: tuple) -> tuple:
        w, x, y, z = q
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    @staticmethod
    def _matrix_to_quat_xyzw(rotation: tuple) -> tuple:
        m00, m01, m02 = rotation[0]
        m10, m11, m12 = rotation[1]
        m20, m21, m22 = rotation[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m21 - m12) / s
            qy = (m02 - m20) / s
            qz = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s
        return qx, qy, qz, qw

    @staticmethod
    def _matmul(a: tuple, b: tuple) -> tuple:
        return tuple(
            tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
            for row in range(3)
        )

    @staticmethod
    def _transpose(matrix: tuple) -> tuple:
        return tuple(tuple(matrix[row][col] for row in range(3)) for col in range(3))

    @staticmethod
    def _identity_matrix() -> tuple:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    @staticmethod
    def _livox_default_rotation() -> tuple:
        # Match the head-mounted URDF frame: MID-360 is rolled 180 deg so its
        # -7..52 deg vertical FOV looks from slightly above horizon down to ground.
        return ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))

    def _update_dead_reckoned_lidar_pose(self, now) -> None:
        elapsed = max(0.0, (now - self.last_lidar_pose_time).nanoseconds * 1e-9)
        self.last_lidar_pose_time = now
        if elapsed > 0.25:
            elapsed = 0.25
        vx, vy, wz = self.current_cmd_vel
        yaw = self.lidar_dead_reckon_yaw
        self.lidar_dead_reckon_x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * elapsed
        self.lidar_dead_reckon_y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * elapsed
        self.lidar_dead_reckon_yaw += wz * elapsed
        origin, rotation = self._dead_reckoned_lidar_pose()
        self.last_sensor_poses[self.lidar_frame] = (origin, rotation)
        if self.publish_lidar_tf:
            self._publish_lidar_tf(origin, rotation)
        self.last_base_pose = (
            (self.lidar_dead_reckon_x, self.lidar_dead_reckon_y, 0.0),
            self._yaw_to_matrix(self.lidar_dead_reckon_yaw),
        )

    def _dead_reckoned_lidar_pose(self) -> tuple:
        origin = (
            self.lidar_dead_reckon_x,
            self.lidar_dead_reckon_y,
            self.robot_base_height + self.pelvis_to_livox_z,
        )
        rotation = self._matmul(
            self._yaw_to_matrix(self.lidar_dead_reckon_yaw), self._livox_default_rotation()
        )
        return origin, rotation

    @staticmethod
    def _yaw_to_matrix(yaw: float) -> tuple:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))

    @staticmethod
    def _camera_default_rotation() -> tuple:
        # Optical +Z forward maps to world +X, +X right maps to world -Y, +Y down maps to world -Z.
        return ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))

    @staticmethod
    def _rotate(rotation: tuple, vector: tuple) -> tuple:
        return (
            rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
            rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
            rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
        )

    def _camera_optical_to_world(self, rotation: tuple, vector: tuple) -> tuple:
        return self._rotate(rotation, vector)

    @staticmethod
    def _world_to_sensor(point: tuple, origin: tuple, rotation: tuple) -> tuple:
        dx, dy, dz = point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]
        return (
            rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz,
            rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz,
            rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz,
        )

    def _world_to_camera_optical(self, point: tuple, origin: tuple, rotation: tuple) -> tuple:
        return self._world_to_sensor(point, origin, rotation)

    @staticmethod
    def _distance(a: tuple, b: tuple) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def main() -> None:
    rclpy.init()
    node = G1SimSensors()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
