import math
import unittest

from sensor_msgs.msg import LaserScan

from unitree_g1_mujoco_bridge.scan_dynamic_filter import ScanDynamicFilter


class ScanDynamicFilterTests(unittest.TestCase):
    def test_remove_dynamic_return_clears_nearby_bins_only(self) -> None:
        scan = LaserScan()
        scan.angle_min = -math.pi / 4.0
        scan.angle_max = math.pi / 4.0
        scan.angle_increment = math.pi / 8.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, 2.0, 2.05, 2.1, 5.0, 5.0]

        node = object.__new__(ScanDynamicFilter)
        node.filter_radius = 0.15
        node.range_padding = 0.10
        node.min_filter_range = 0.2

        removed = ScanDynamicFilter._remove_dynamic_return(node, scan, 2.0, 0.0)

        self.assertGreaterEqual(removed, 1)
        self.assertEqual(scan.ranges[0], 5.0)
        self.assertTrue(math.isinf(scan.ranges[3]))
        self.assertEqual(scan.ranges[-1], 5.0)

    def test_remove_dynamic_return_ignores_invalid_ranges(self) -> None:
        scan = LaserScan()
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 0.5
        scan.range_min = 0.5
        scan.range_max = 5.0
        scan.ranges = [1.0, 1.0, 1.0, 1.0]

        node = object.__new__(ScanDynamicFilter)
        node.filter_radius = 0.2
        node.range_padding = 0.0
        node.min_filter_range = 0.5

        self.assertEqual(ScanDynamicFilter._remove_dynamic_return(node, scan, 0.1, 0.0), 0)
        self.assertEqual(ScanDynamicFilter._remove_dynamic_return(node, scan, 6.0, 0.0), 0)


if __name__ == "__main__":
    unittest.main()
