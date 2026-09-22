import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from dynamic_obstacle_benchmarks.config import BenchmarkConfig, Bounds, load_config


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

    def test_invalid_distance_margin_order_fails(self) -> None:
        with self.assertRaises(ValueError):
            BenchmarkConfig(critical_margin_m=-0.01).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(critical_margin_m=0.35, near_miss_margin_m=0.35).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(near_miss_margin_m=0.75, warning_margin_m=0.75).validate()

    def test_invalid_ttc_threshold_order_fails(self) -> None:
        with self.assertRaises(ValueError):
            BenchmarkConfig(ttc_critical_s=0.0).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(ttc_critical_s=2.0, ttc_warning_s=1.5).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(ttc_warning_s=4.0, ttc_horizon_s=3.0).validate()

    def test_invalid_data_health_contract_fails(self) -> None:
        with self.assertRaises(ValueError):
            BenchmarkConfig(odom_stale_timeout_s=0.0).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(pedestrian_stale_timeout_s=-1.0).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(scan_stale_timeout_s=0.0).validate()
        with self.assertRaises(ValueError):
            BenchmarkConfig(control_people_source="").validate()

    def test_quoted_boolean_is_not_silently_coerced(self) -> None:
        with TemporaryDirectory() as directory:
            path = Path(directory) / "benchmark.yaml"
            path.write_text('data_health:\n  require_exact_people_count: "false"\n')
            with self.assertRaisesRegex(ValueError, "must be a boolean"):
                load_config(str(path))


if __name__ == "__main__":
    unittest.main()
