import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

from .pointcloud import iter_xyz


class LivoxToScan(Node):
    def __init__(self) -> None:
        super().__init__("livox_to_scan")
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 25.0)
        self.declare_parameter("min_height", -1.10)
        self.declare_parameter("max_height", 0.65)
        self.declare_parameter("min_floor_range", 1.0)
        self.declare_parameter("fill_empty_bins", True)
        self.declare_parameter("max_fill_gap_deg", 12.0)
        self.declare_parameter("scan_time", 0.1)

        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.min_floor_range = float(self.get_parameter("min_floor_range").value)
        self.fill_empty_bins = bool(self.get_parameter("fill_empty_bins").value)
        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")
        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be greater than angle_min")
        if self.range_max <= self.range_min:
            raise ValueError("range_max must be greater than range_min")
        if self.max_height < self.min_height:
            raise ValueError("max_height must be greater than or equal to min_height")
        self.max_fill_gap_bins = max(
            1,
            int(math.radians(float(self.get_parameter("max_fill_gap_deg").value)) / self.angle_increment),
        )
        self.scan_time = float(self.get_parameter("scan_time").value)
        if self.scan_time <= 0.0:
            raise ValueError("scan_time must be positive")
        self.bin_count = max(1, int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)))

        self.pub = self.create_publisher(LaserScan, str(self.get_parameter("scan_topic").value), 10)
        self.last_debug_time = self.get_clock().now()
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Projecting Livox PointCloud2 to /scan for 2D SLAM")

    def _cloud_callback(self, msg: PointCloud2) -> None:
        ranges = [math.inf] * self.bin_count
        for x, y, z in iter_xyz(msg):
            if z < self.min_height or z > self.max_height:
                continue
            distance = math.hypot(x, y)
            if distance < self.range_min or distance > self.range_max:
                continue
            # Keep low street furniture while rejecting near-field ground/floor returns.
            if z < -1.05 and distance > self.min_floor_range:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle >= self.angle_max:
                continue
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.bin_count and distance < ranges[index]:
                ranges[index] = distance

        if self.fill_empty_bins:
            ranges = self._fill_circular_gaps(ranges)
        self._debug_coverage(ranges)

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (self.bin_count - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.pub.publish(scan)

    def _fill_circular_gaps(self, ranges: list[float]) -> list[float]:
        if not ranges or all(not math.isfinite(value) for value in ranges):
            return ranges
        filled = list(ranges)
        count = len(ranges)
        finite_indexes = [index for index, value in enumerate(ranges) if math.isfinite(value)]
        if len(finite_indexes) < 2:
            return ranges
        for start_pos, start_index in enumerate(finite_indexes):
            end_index = finite_indexes[(start_pos + 1) % len(finite_indexes)]
            gap = (end_index - start_index) % count
            if gap <= 1 or gap > self.max_fill_gap_bins:
                continue
            start_range = ranges[start_index]
            end_range = ranges[end_index]
            for step in range(1, gap):
                t = step / gap
                filled[(start_index + step) % count] = (1.0 - t) * start_range + t * end_range
        return filled

    def _debug_coverage(self, ranges: list[float]) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds * 1e-9 < 3.0:
            return
        self.last_debug_time = now
        quadrants = [0, 0, 0, 0]
        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue
            angle = self.angle_min + index * self.angle_increment
            if -math.pi <= angle < -math.pi / 2:
                quadrants[0] += 1
            elif -math.pi / 2 <= angle < 0:
                quadrants[1] += 1
            elif 0 <= angle < math.pi / 2:
                quadrants[2] += 1
            else:
                quadrants[3] += 1
        self.get_logger().info(
            "360 scan bins populated by quadrant "
            f"rear_right={quadrants[0]} front_right={quadrants[1]} "
            f"front_left={quadrants[2]} rear_left={quadrants[3]}",
            throttle_duration_sec=3.0,
        )


def main() -> None:
    rclpy.init()
    node = LivoxToScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
