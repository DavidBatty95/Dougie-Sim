import math
import unittest

from dynamic_obstacle_benchmarks.geometry import Pose2D, Velocity2D, circle_clearance, polygon_circle_clearance, time_to_collision


class GeometryTests(unittest.TestCase):
    def test_circle_clearance(self) -> None:
        self.assertAlmostEqual(circle_clearance(Pose2D(0, 0), 0.5, Pose2D(2, 0), 0.5), 1.0)

    def test_polygon_circle_clearance(self) -> None:
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        self.assertLess(polygon_circle_clearance(square, Pose2D(0, 0), 0.2), 0.0)
        self.assertAlmostEqual(polygon_circle_clearance(square, Pose2D(2, 0), 0.2), 0.8)

    def test_ttc_approaching_crossing_and_moving_away(self) -> None:
        robot = Pose2D(0, 0)
        self.assertAlmostEqual(
            time_to_collision(robot, Velocity2D(1, 0), Pose2D(2, 0), Velocity2D(0, 0), 0.5, 5.0),
            2.0,
        )
        self.assertIsNone(time_to_collision(robot, Velocity2D(-1, 0), Pose2D(2, 0), Velocity2D(0, 0), 0.5, 5.0))
        self.assertIsNone(time_to_collision(robot, Velocity2D(0, 0), Pose2D(2, 0), Velocity2D(0, 0), 0.5, 5.0))


if __name__ == "__main__":
    unittest.main()
