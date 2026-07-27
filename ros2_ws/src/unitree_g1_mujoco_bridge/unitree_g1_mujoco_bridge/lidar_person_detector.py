import math
import os
import struct
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .pointcloud import iter_xyz


@dataclass
class Detection:
    x: float
    y: float
    z: float
    confidence: float
    radius: float


class LidarPersonDetector(Node):
    """Optional LiDAR-only BEV person detector.

    The node prepares a robot-centred bird's-eye-view tensor from Livox points and
    runs an ONNX detector when a model is provided. Model output is expected to be
    an array of detections in robot-local metres: [x, y, confidence, radius].
    """

    def __init__(self) -> None:
        super().__init__("lidar_person_detector")
        self.declare_parameter("cloud_topic", "/livox/lidar_world")
        self.declare_parameter("detection_topic", "/lidar_person_detections")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("model_path", "")
        self.declare_parameter("grid_size", 10.0)
        self.declare_parameter("grid_resolution", 0.10)
        self.declare_parameter("min_height", 0.05)
        self.declare_parameter("max_height", 2.20)
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("default_radius", 0.45)
        self.declare_parameter("max_detections", 16)
        self.declare_parameter("fallback_detection", True)
        self.declare_parameter("fallback_cluster_distance", 0.32)
        self.declare_parameter("fallback_min_points", 4)
        self.declare_parameter("fallback_max_points", 260)
        self.declare_parameter("fallback_min_height", 0.25)
        self.declare_parameter("fallback_max_height", 2.20)
        self.declare_parameter("fallback_max_extent", 1.20)
        self.declare_parameter("fallback_min_confidence", 0.50)
        self.declare_parameter("fallback_max_detections", 6)
        self.declare_parameter("fallback_nms_radius", 0.75)
        self.declare_parameter("robot_self_filter_radius", 0.60)
        self.declare_parameter("point_accumulation_time", 0.80)
        self.declare_parameter("max_accumulated_points", 12000)
        self.declare_parameter("processing_rate_hz", 4.0)
        self.declare_parameter("downsample_resolution", 0.16)
        self.declare_parameter("max_processed_points", 3500)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.grid_size = float(self.get_parameter("grid_size").value)
        self.grid_resolution = float(self.get_parameter("grid_resolution").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.default_radius = float(self.get_parameter("default_radius").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.fallback_detection = bool(self.get_parameter("fallback_detection").value)
        self.fallback_cluster_distance = float(self.get_parameter("fallback_cluster_distance").value)
        self.fallback_min_points = int(self.get_parameter("fallback_min_points").value)
        self.fallback_max_points = int(self.get_parameter("fallback_max_points").value)
        self.fallback_min_height = float(self.get_parameter("fallback_min_height").value)
        self.fallback_max_height = float(self.get_parameter("fallback_max_height").value)
        self.fallback_max_extent = float(self.get_parameter("fallback_max_extent").value)
        self.fallback_min_confidence = float(self.get_parameter("fallback_min_confidence").value)
        self.fallback_max_detections = int(self.get_parameter("fallback_max_detections").value)
        self.fallback_nms_radius = float(self.get_parameter("fallback_nms_radius").value)
        self.robot_self_filter_radius = float(self.get_parameter("robot_self_filter_radius").value)
        self.point_accumulation_time = float(self.get_parameter("point_accumulation_time").value)
        self.max_accumulated_points = int(self.get_parameter("max_accumulated_points").value)
        self.processing_period = 1.0 / max(0.1, float(self.get_parameter("processing_rate_hz").value))
        self.downsample_resolution = float(self.get_parameter("downsample_resolution").value)
        self.max_processed_points = int(self.get_parameter("max_processed_points").value)
        self.grid_cells = max(1, int(round(self.grid_size / max(1e-3, self.grid_resolution))))
        self.model_path = str(self.get_parameter("model_path").value)
        self.session = self._load_model(self.model_path)
        self.input_name = self.session.get_inputs()[0].name if self.session is not None else ""
        self.recent_points: deque[tuple[float, list[tuple[float, float, float]]]] = deque()
        self.last_processed_stamp = -math.inf
        self.last_detection_count = 0
        self.last_point_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter("detection_topic").value), 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._diagnostics)
        if self.session is None:
            if self.fallback_detection:
                self.get_logger().warn(
                    "LiDAR person detector has no ONNX model; using conservative LiDAR cluster fallback detections. "
                    "Set lidar_person_model_path:=/path/to/model.onnx to enable ML detections."
                )
            else:
                self.get_logger().warn(
                    "LiDAR person detector has no ONNX model and fallback_detection is false; publishing empty detections."
                )
        else:
            self.get_logger().info(f"LiDAR BEV person detector loaded {self.model_path}")

    def _load_model(self, model_path: str):
        if not model_path or not os.path.exists(model_path):
            return None
        try:
            import onnxruntime as ort
        except ImportError:
            self.get_logger().error("onnxruntime is not available; cannot load LiDAR person detector model")
            return None
        return ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    def _cloud_callback(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
        if stamp_seconds - self.last_processed_stamp < self.processing_period:
            return
        self.last_processed_stamp = stamp_seconds
        robot_pose = self._robot_pose()
        if robot_pose is None:
            self.pub.publish(self._detection_cloud([], self.target_frame, stamp))
            return
        robot_x, robot_y, yaw = robot_pose
        transform = None
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
            except TransformException as exc:
                self.get_logger().warn(
                    f"No transform {self.target_frame} <- {msg.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return

        local_points = []
        for x, y, z in iter_xyz(msg):
            if transform is not None:
                x, y, z = self._transform_point(transform, x, y, z)
            if z < self.min_height or z > self.max_height:
                continue
            dx = x - robot_x
            dy = y - robot_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if math.hypot(local_x, local_y) <= self.robot_self_filter_radius:
                continue
            if abs(local_x) > self.grid_size * 0.5 or abs(local_y) > self.grid_size * 0.5:
                continue
            local_points.append((local_x, local_y, z))

        accumulated_points = self._downsample_points(self._accumulated_points(local_points, stamp_seconds))
        if self.session is None:
            detections = self._fallback_detections(accumulated_points, robot_x, robot_y, yaw)
        else:
            features = self._bev_features(accumulated_points)
            detections = self._run_model(features, robot_x, robot_y, yaw)
        self.last_detection_count = len(detections)
        self.last_point_count = len(accumulated_points)
        self.pub.publish(self._detection_cloud(detections, self.target_frame, stamp))

    def _downsample_points(self, points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        if not points:
            return []
        resolution = self.downsample_resolution
        if resolution <= 0.0:
            return points[: self.max_processed_points] if self.max_processed_points > 0 else points
        voxels: dict[tuple[int, int, int], tuple[float, float, float, int]] = {}
        for x, y, z in points:
            key = (
                math.floor(x / resolution),
                math.floor(y / resolution),
                math.floor(z / resolution),
            )
            if key in voxels:
                sx, sy, sz, count = voxels[key]
                voxels[key] = (sx + x, sy + y, sz + z, count + 1)
            else:
                voxels[key] = (x, y, z, 1)
        downsampled = [(sx / count, sy / count, sz / count) for sx, sy, sz, count in voxels.values()]
        if self.max_processed_points > 0 and len(downsampled) > self.max_processed_points:
            step = max(1, len(downsampled) // self.max_processed_points)
            downsampled = downsampled[::step][: self.max_processed_points]
        return downsampled

    def _accumulated_points(
        self, points: list[tuple[float, float, float]], stamp: float
    ) -> list[tuple[float, float, float]]:
        if self.point_accumulation_time <= 0.0:
            return points
        self.recent_points.append((stamp, points))
        min_stamp = stamp - self.point_accumulation_time
        while self.recent_points and self.recent_points[0][0] < min_stamp:
            self.recent_points.popleft()

        accumulated_count = sum(len(entry_points) for _, entry_points in self.recent_points)
        while self.recent_points and accumulated_count > self.max_accumulated_points:
            _, removed_points = self.recent_points.popleft()
            accumulated_count -= len(removed_points)

        accumulated = []
        for _, entry_points in self.recent_points:
            accumulated.extend(entry_points)
        return accumulated

    def _fallback_detections(
        self,
        points: list[tuple[float, float, float]],
        robot_x: float,
        robot_y: float,
        yaw: float,
    ) -> list[Detection]:
        if not self.fallback_detection or len(points) < self.fallback_min_points:
            return []
        clusters = self._clusters(points, max(0.10, self.fallback_cluster_distance))
        detections = []
        for cluster_points in clusters:
            detection = self._cluster_to_detection(cluster_points, robot_x, robot_y, yaw)
            if detection is not None:
                detections.append(detection)
        detections.sort(key=lambda detection: (detection.confidence, -math.hypot(detection.x - robot_x, detection.y - robot_y)), reverse=True)
        selected: list[Detection] = []
        max_detections = max(0, min(self.max_detections, self.fallback_max_detections))
        for detection in detections:
            if any(math.hypot(detection.x - kept.x, detection.y - kept.y) < self.fallback_nms_radius for kept in selected):
                continue
            selected.append(detection)
            if len(selected) >= max_detections:
                break
        return selected

    def _clusters(
        self, points: list[tuple[float, float, float]], cluster_distance: float
    ) -> list[list[tuple[float, float, float]]]:
        grid: dict[tuple[int, int], list[int]] = {}
        for index, (x, y, _) in enumerate(points):
            grid.setdefault((math.floor(x / cluster_distance), math.floor(y / cluster_distance)), []).append(index)

        remaining = set(range(len(points)))
        clusters = []
        while remaining:
            start = remaining.pop()
            queue = deque([start])
            indexes = [start]
            while queue:
                point_index = queue.popleft()
                px, py, _ = points[point_index]
                cx = math.floor(px / cluster_distance)
                cy = math.floor(py / cluster_distance)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for other_index in grid.get((cx + dx, cy + dy), ()):
                            if other_index not in remaining:
                                continue
                            ox, oy, _ = points[other_index]
                            if math.hypot(ox - px, oy - py) > cluster_distance:
                                continue
                            remaining.remove(other_index)
                            queue.append(other_index)
                            indexes.append(other_index)
            if self.fallback_min_points <= len(indexes) <= self.fallback_max_points:
                clusters.append([points[index] for index in indexes])
        return clusters

    def _cluster_to_detection(
        self,
        points: list[tuple[float, float, float]],
        robot_x: float,
        robot_y: float,
        yaw: float,
    ) -> Detection | None:
        count = len(points)
        sum_x = 0.0
        sum_y = 0.0
        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        min_z = math.inf
        max_z = -math.inf
        for x, y, z in points:
            sum_x += x
            sum_y += y
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            min_z = min(min_z, z)
            max_z = max(max_z, z)
        mean_x = sum_x / count
        mean_y = sum_y / count
        width = max_x - min_x
        depth = max_y - min_y
        height = max_z - min_z
        extent = max(width, depth)
        radius = max(math.hypot(x - mean_x, y - mean_y) for x, y, _ in points)

        partial_height = height < self.fallback_min_height
        if height > self.fallback_max_height:
            return None
        if height < self.fallback_min_height or extent < 0.12:
            return None
        if extent > self.fallback_max_extent or radius > self.default_radius * 1.55:
            return None
        if min(width, depth) > 0.05:
            aspect = max(width, depth) / min(width, depth)
            if aspect > 3.0:
                return None

        confidence = 0.25
        if 0.16 <= radius <= 0.58:
            confidence += 0.22
        if 0.22 <= extent <= 0.95:
            confidence += 0.18
        if 0.50 <= height <= 2.05:
            confidence += 0.22
        if count >= self.fallback_min_points:
            confidence += min(0.20, 0.018 * count)
        if min(width, depth) > 0.05:
            aspect = max(width, depth) / min(width, depth)
            if aspect <= 3.5:
                confidence += 0.10
        confidence = min(0.90, confidence)
        if confidence < self.fallback_min_confidence:
            return None

        world_x = robot_x + math.cos(yaw) * mean_x - math.sin(yaw) * mean_y
        world_y = robot_y + math.sin(yaw) * mean_x + math.cos(yaw) * mean_y
        return Detection(world_x, world_y, 0.6, confidence, max(0.25, min(0.80, radius + 0.15)))

    def _diagnostics(self) -> None:
        self.get_logger().info(
            f"LiDAR person detector: points={self.last_point_count} detections={self.last_detection_count} "
            f"mode={'onnx' if self.session is not None else 'cluster_fallback'}",
            throttle_duration_sec=2.0,
        )

    def _bev_features(self, points: list[tuple[float, float, float]]) -> np.ndarray:
        cells = self.grid_cells
        half = self.grid_size * 0.5
        count = np.zeros((cells, cells), dtype=np.float32)
        max_z = np.full((cells, cells), self.min_height, dtype=np.float32)
        mean_z = np.zeros((cells, cells), dtype=np.float32)
        for x, y, z in points:
            col = int((x + half) / self.grid_resolution)
            row = int((y + half) / self.grid_resolution)
            if row < 0 or row >= cells or col < 0 or col >= cells:
                continue
            count[row, col] += 1.0
            max_z[row, col] = max(max_z[row, col], z)
            mean_z[row, col] += z
        nonzero = count > 0.0
        mean_z[nonzero] /= count[nonzero]
        occupancy = nonzero.astype(np.float32)
        density = np.minimum(count, 12.0) / 12.0
        height = np.clip((max_z - self.min_height) / max(1e-3, self.max_height - self.min_height), 0.0, 1.0)
        mean_height = np.clip((mean_z - self.min_height) / max(1e-3, self.max_height - self.min_height), 0.0, 1.0)
        return np.stack((occupancy, density, height, mean_height), axis=0)[None, :, :, :]

    def _run_model(self, features: np.ndarray, robot_x: float, robot_y: float, yaw: float) -> list[Detection]:
        outputs = self.session.run(None, {self.input_name: features.astype(np.float32)})
        raw = np.asarray(outputs[0]).reshape(-1, outputs[0].shape[-1])
        detections = []
        for row in raw[: self.max_detections * 4]:
            if row.shape[0] < 3:
                continue
            local_x = float(row[0])
            local_y = float(row[1])
            confidence = float(row[2])
            if confidence < self.min_confidence:
                continue
            radius = float(row[3]) if row.shape[0] >= 4 and math.isfinite(float(row[3])) else self.default_radius
            if not math.isfinite(local_x) or not math.isfinite(local_y):
                continue
            if abs(local_x) > self.grid_size * 0.5 or abs(local_y) > self.grid_size * 0.5:
                continue
            world_x = robot_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
            world_y = robot_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
            detections.append(Detection(world_x, world_y, 0.6, confidence, max(0.15, min(1.2, radius))))
            if len(detections) >= self.max_detections:
                break
        return detections

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self._yaw_from_quat(transform.transform.rotation),
        )

    @staticmethod
    def _transform_point(transform, x: float, y: float, z: float) -> tuple[float, float, float]:
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        uvx = q.y * z - q.z * y
        uvy = q.z * x - q.x * z
        uvz = q.x * y - q.y * x
        uuvx = q.y * uvz - q.z * uvy
        uuvy = q.z * uvx - q.x * uvz
        uuvz = q.x * uvy - q.y * uvx
        rx = x + 2.0 * (q.w * uvx + uuvx)
        ry = y + 2.0 * (q.w * uvy + uuvy)
        rz = z + 2.0 * (q.w * uvz + uuvz)
        return rx + tx, ry + ty, rz + tz

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _detection_cloud(detections: list[Detection], frame_id: str, stamp) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = stamp
        cloud.height = 1
        cloud.width = len(detections)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="confidence", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="radius", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 20
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        data = bytearray(cloud.row_step)
        for index, detection in enumerate(detections):
            struct.pack_into(
                "<fffff",
                data,
                index * cloud.point_step,
                detection.x,
                detection.y,
                detection.z,
                detection.confidence,
                detection.radius,
            )
        cloud.data = bytes(data)
        return cloud


def main() -> None:
    rclpy.init()
    node = LidarPersonDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
