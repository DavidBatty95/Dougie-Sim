import math
import struct
import unittest

from sensor_msgs.msg import PointCloud2, PointField

from unitree_g1_mujoco_bridge.pointcloud import iter_xyz


def _field(name: str, offset: int) -> PointField:
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


class PointCloudTests(unittest.TestCase):
    def test_iter_xyz_yields_only_finite_points(self) -> None:
        msg = PointCloud2()
        msg.fields = [_field("x", 0), _field("y", 4), _field("z", 8)]
        msg.point_step = 12
        msg.data = b"".join(
            struct.pack("<fff", *point)
            for point in [(1.0, 2.0, 3.0), (math.nan, 0.0, 0.0), (-1.0, -2.0, -3.0)]
        )

        self.assertEqual(list(iter_xyz(msg)), [(1.0, 2.0, 3.0), (-1.0, -2.0, -3.0)])

    def test_iter_xyz_ignores_invalid_layouts(self) -> None:
        msg = PointCloud2()
        msg.fields = [_field("x", 0), _field("y", 4)]
        msg.point_step = 8
        msg.data = struct.pack("<ff", 1.0, 2.0)

        self.assertEqual(list(iter_xyz(msg)), [])


if __name__ == "__main__":
    unittest.main()
