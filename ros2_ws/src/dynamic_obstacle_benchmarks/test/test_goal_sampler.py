import math
import random
import unittest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig, Bounds
from dynamic_obstacle_benchmarks.geometry import Pose2D, distance_xy
from dynamic_obstacle_benchmarks.goal_sampler import GoalSampler


class GoalSamplerTests(unittest.TestCase):
    def test_local_goal_bounds_and_distance_leave_room_for_avoidance(self) -> None:
        cfg = BenchmarkConfig(bounds=Bounds(-10, 10, -10, 10), goal_bounds=Bounds(-3, 3, -3, 3),
                              goals_count=8, min_goal_distance_m=1.0, max_goal_distance_m=2.0)
        cfg.validate()
        previous = Pose2D(0, 0)
        for goal in GoalSampler(cfg, random.Random(42)).sample(previous, []):
            self.assertGreaterEqual(distance_xy(previous, goal.pose), 1.0)
            self.assertLessEqual(distance_xy(previous, goal.pose), 2.0)
            self.assertLessEqual(abs(goal.pose.x), 2.5)
            self.assertLessEqual(abs(goal.pose.y), 2.5)
            previous = goal.pose
        # Escaping outside the sampling area must not prevent a return goal.
        goal = GoalSampler(cfg, random.Random(42)).sample_random_goal(0, Pose2D(7, 0), [])
        self.assertLessEqual(abs(goal.pose.x), 2.5)

    def test_goal_bounds_and_distance_configuration_are_validated(self) -> None:
        with self.assertRaises(ValueError):
            BenchmarkConfig(goal_bounds=Bounds(-20, 20, -1, 1)).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(min_goal_distance_m=2.0, max_goal_distance_m=1.0).validate()

    def test_random_goals_are_deterministic_and_inside_bounds(self) -> None:
        cfg = BenchmarkConfig(goals_count=5, people_count=0)
        goals_a = GoalSampler(cfg, random.Random(7)).sample(Pose2D(0, 0), [])
        goals_b = GoalSampler(cfg, random.Random(7)).sample(Pose2D(0, 0), [])
        self.assertEqual([g.as_dict() for g in goals_a], [g.as_dict() for g in goals_b])
        for goal in goals_a:
            self.assertGreaterEqual(goal.pose.x, cfg.bounds.x_min + cfg.min_distance_from_walls_m)
            self.assertLessEqual(goal.pose.x, cfg.bounds.x_max - cfg.min_distance_from_walls_m)
            self.assertGreaterEqual(goal.pose.y, cfg.bounds.y_min + cfg.min_distance_from_walls_m)
            self.assertLessEqual(goal.pose.y, cfg.bounds.y_max - cfg.min_distance_from_walls_m)

    def test_random_goals_follow_the_nonholonomic_heading_limit(self) -> None:
        cfg = BenchmarkConfig(
            goals_count=5,
            people_count=0,
            min_goal_distance_m=0.1,
            max_goal_heading_change_rad=1.4,
            max_sampling_attempts=1000,
        )
        previous = Pose2D(0.0, 0.0, 0.0)
        goals = GoalSampler(cfg, random.Random(12)).sample(previous, [])

        for goal in goals:
            expected_yaw = math.atan2(
                goal.pose.y - previous.y,
                goal.pose.x - previous.x,
            )
            heading_change = math.atan2(
                math.sin(goal.pose.yaw - previous.yaw),
                math.cos(goal.pose.yaw - previous.yaw),
            )
            self.assertAlmostEqual(goal.pose.yaw, expected_yaw)
            self.assertLessEqual(abs(heading_change), 1.4)
            previous = goal.pose

    def test_fixed_goal_mode_requires_enough_goals(self) -> None:
        cfg = BenchmarkConfig(goal_mode="fixed", goals_count=2, fixed_goals=[{"x": 1, "y": 2}])
        with self.assertRaises(ValueError):
            GoalSampler(cfg, random.Random(1)).sample(Pose2D(0, 0), [])


if __name__ == "__main__":
    unittest.main()
