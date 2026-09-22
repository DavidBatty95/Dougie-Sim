import math
import unittest

from dynamic_obstacle_benchmarks.geometry import (
    Pose2D,
    Velocity2D,
    circle_clearance,
    closest_approach,
    polygon_circle_clearance,
    time_to_collision,
)


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
            1.5,
        )
        self.assertIsNone(time_to_collision(robot, Velocity2D(-1, 0), Pose2D(2, 0), Velocity2D(0, 0), 0.5, 5.0))
        self.assertIsNone(time_to_collision(robot, Velocity2D(0, 0), Pose2D(2, 0), Velocity2D(0, 0), 0.5, 5.0))

    def test_ttc_handles_overlap_horizon_and_tangent_contact(self) -> None:
        robot = Pose2D(0, 0)
        self.assertEqual(
            time_to_collision(robot, Velocity2D(1, 0), Pose2D(0.25, 0), Velocity2D(), 0.5, 5.0),
            0.0,
        )
        self.assertIsNone(
            time_to_collision(robot, Velocity2D(1, 0), Pose2D(2, 0), Velocity2D(), 0.5, 1.49)
        )
        self.assertAlmostEqual(
            time_to_collision(robot, Velocity2D(1, 0), Pose2D(2, 0), Velocity2D(), 0.5, 1.5),
            1.5,
        )
        self.assertAlmostEqual(
            time_to_collision(robot, Velocity2D(1, 0), Pose2D(2, 0.5), Velocity2D(), 0.5, 5.0),
            2.0,
        )

    def test_closest_approach_reports_horizon_limited_tcpa_and_dcpa(self) -> None:
        self.assertEqual(
            closest_approach(Pose2D(0, 0), Velocity2D(), Pose2D(2, 0), Velocity2D(), 5.0),
            (0.0, 2.0),
        )
        tcpa, dcpa = closest_approach(
            Pose2D(0, 0), Velocity2D(1, 0), Pose2D(2, 1), Velocity2D(), 5.0
        )
        self.assertAlmostEqual(tcpa, 2.0)
        self.assertAlmostEqual(dcpa, 1.0)
        self.assertEqual(
            closest_approach(Pose2D(0, 0), Velocity2D(-1, 0), Pose2D(2, 0), Velocity2D(), 5.0),
            (0.0, 2.0),
        )


if __name__ == "__main__":
    unittest.main()
