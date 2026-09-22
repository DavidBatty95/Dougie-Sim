import unittest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.metrics import BenchmarkMetrics
from dynamic_obstacle_benchmarks.scoring import score_benchmark


class ScoringTests(unittest.TestCase):
    def test_collision_is_failure(self) -> None:
        cfg = BenchmarkConfig(goals_count=10)
        metrics = BenchmarkMetrics(goals_requested=10, goals_reached=10, total_time_s=50, actual_path_length_m=20, ideal_path_length_m=20, collisions=1)
        self.assertEqual(score_benchmark(metrics, cfg).final_score, 0.0)
        metrics.collisions = 2
        self.assertEqual(score_benchmark(metrics, cfg).final_score, 0.0)

    def test_no_movement_is_failure(self) -> None:
        cfg = BenchmarkConfig(goals_count=5)
        metrics = BenchmarkMetrics(goals_requested=5, total_time_s=100, no_movement=True)
        self.assertEqual(score_benchmark(metrics, cfg).final_score, 0.0)

    def test_fall_is_failure(self) -> None:
        cfg = BenchmarkConfig(goals_count=5)
        metrics = BenchmarkMetrics(goals_requested=5, goals_reached=5, total_time_s=100, actual_path_length_m=10, ideal_path_length_m=10, fallen=True)
        self.assertEqual(score_benchmark(metrics, cfg).final_score, 0.0)

    def test_safe_complete_run_scores_well(self) -> None:
        cfg = BenchmarkConfig(goals_count=2)
        metrics = BenchmarkMetrics(goals_requested=2, goals_reached=2, total_time_s=20, actual_path_length_m=10, ideal_path_length_m=9)
        self.assertGreater(score_benchmark(metrics, cfg).final_score, 70.0)

    def test_invalid_trial_is_not_scored(self) -> None:
        cfg = BenchmarkConfig(goals_count=1)
        metrics = BenchmarkMetrics(goals_requested=1, valid_trial=False, invalid_reason="nav2_not_ready")
        score = score_benchmark(metrics, cfg)
        self.assertEqual(score.final_score, 0.0)
        self.assertEqual(score.safety_score, 0.0)

    def test_proximity_duration_penalty_uses_wall_time(self) -> None:
        cfg = BenchmarkConfig(goals_count=1)
        sparse = BenchmarkMetrics(
            goals_requested=1,
            warning_distance_wall_time_s=1.0,
            warning_distance_person_time_s=1.0,
            time_below_warning_distance_s=1.0,
        )
        crowded = BenchmarkMetrics(
            goals_requested=1,
            warning_distance_wall_time_s=1.0,
            warning_distance_person_time_s=10.0,
            time_below_warning_distance_s=10.0,
        )
        self.assertEqual(
            score_benchmark(sparse, cfg).safety_score,
            score_benchmark(crowded, cfg).safety_score,
        )


if __name__ == "__main__":
    unittest.main()
