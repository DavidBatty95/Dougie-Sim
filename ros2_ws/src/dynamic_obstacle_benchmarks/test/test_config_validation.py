import unittest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig, Bounds


class ConfigValidationTests(unittest.TestCase):
    def test_invalid_bounds_fail(self) -> None:
        cfg = BenchmarkConfig(bounds=Bounds(1, -1, -1, 1))
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_invalid_thresholds_fail(self) -> None:
        cfg = BenchmarkConfig(robot_radius_m=-1.0)
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_negative_near_miss_duration_fails(self) -> None:
        cfg = BenchmarkConfig(near_miss_min_duration_s=-0.01)
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_invalid_goal_sampling_limits_fail(self) -> None:
        with self.assertRaises(ValueError):
            BenchmarkConfig(max_sampling_attempts=0).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(path_validation_timeout_s=0.0).validate()


if __name__ == "__main__":
    unittest.main()
