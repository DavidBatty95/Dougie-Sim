import random
import unittest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D, distance_xy
from dynamic_obstacle_benchmarks.goal_sampler import GoalSampler


class GoalSamplerTests(unittest.TestCase):
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

    def test_fixed_goal_mode_requires_enough_goals(self) -> None:
        cfg = BenchmarkConfig(goal_mode="fixed", goals_count=2, fixed_goals=[{"x": 1, "y": 2}])
        with self.assertRaises(ValueError):
            GoalSampler(cfg, random.Random(1)).sample(Pose2D(0, 0), [])


if __name__ == "__main__":
    unittest.main()
