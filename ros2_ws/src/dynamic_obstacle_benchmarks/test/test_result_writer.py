import json
import tempfile
import unittest
from pathlib import Path

import yaml

from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.goal_sampler import Goal
from dynamic_obstacle_benchmarks.geometry import Pose2D
from dynamic_obstacle_benchmarks.metrics import BenchmarkMetrics
from dynamic_obstacle_benchmarks.result_writer import ResultWriter
from dynamic_obstacle_benchmarks.scoring import ScoreBreakdown


class ResultWriterTests(unittest.TestCase):
    def test_writes_json_and_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cfg = BenchmarkConfig(output_dir=tmp, goals_count=1, people_count=0)
            writer = ResultWriter(cfg, str(Path(tmp) / "result.json"))
            goals = [Goal(0, Pose2D(1, 0))]
            scenario = writer.write_scenario(cfg, goals, [])
            metrics = BenchmarkMetrics(
                goals_requested=1, yield_episodes=4, resume_episodes=4, yield_time_s=3.5
            )
            path = writer.write(cfg, metrics, ScoreBreakdown(100, 0, 0, 0, 0, 65), goals, [], scenario)
            self.assertTrue(path.exists())
            result = json.loads(path.read_text())
            self.assertEqual(result["schema_version"], 2)
            self.assertTrue(result["run_id"])
            self.assertEqual(result["benchmark_name"], "person_avoidance_benchmark")
            self.assertIn("minimum_center_distance_m", result)
            self.assertIn("minimum_body_clearance_m", result)
            self.assertIn("stale_events", result)
            self.assertIn("speed_limit_violations", result)
            self.assertIn("left_valid_area", result)
            self.assertEqual(result["people_requested"], 0)
            self.assertEqual(result["people_observed"], 0)
            self.assertEqual(result["people_active_initial"], 0)
            self.assertEqual(
                result["data_sources"]["requested"]["control_people"],
                "mock_goal_follower_no_people_input",
            )
            self.assertEqual(result["data_sources"]["requested"]["scoring_people"], "synthetic_ground_truth")
            self.assertTrue(result["data_health"]["require_exact_people_count"])
            self.assertEqual(result["data_sources"]["observed"]["scan"], "not_observed")
            self.assertEqual(result["artifacts"]["scenario"]["path"], "result_scenario.yaml")
            scenario_data = yaml.safe_load(scenario.read_text())
            self.assertEqual(scenario_data["run_id"], result["run_id"])
            self.assertEqual(result["data_health"]["max_scan_sample_stall_s"], 0.0)
            self.assertEqual(result["yield_episodes"], 4)
            self.assertEqual(result["resume_episodes"], 4)
            self.assertEqual(result["yield_time_s"], 3.5)
            self.assertTrue(path.with_name("result_goals.csv").exists())
            self.assertTrue(path.with_name("result_events.csv").exists())

    def test_invalid_trial_has_explicit_status_and_reason(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cfg = BenchmarkConfig(output_dir=tmp, goals_count=1, people_count=0)
            writer = ResultWriter(cfg, str(Path(tmp) / "invalid.json"))
            metrics = BenchmarkMetrics(
                goals_requested=1,
                valid_trial=False,
                invalid_reason="no_cmd_vel_before_scoring_start",
            )
            scenario = writer.write_scenario(cfg, [], [])
            path = writer.write(cfg, metrics, ScoreBreakdown(0, 0, 0, 0, 0, 0), [], [], scenario)
            result = json.loads(path.read_text())
            self.assertFalse(result["valid_trial"])
            self.assertFalse(result["success"])
            self.assertEqual(result["invalid_reason"], "no_cmd_vel_before_scoring_start")
            self.assertEqual(result["failure_reason"], "no_cmd_vel_before_scoring_start")


if __name__ == "__main__":
    unittest.main()
