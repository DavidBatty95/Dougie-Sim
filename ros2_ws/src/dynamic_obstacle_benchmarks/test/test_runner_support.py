import argparse
import random
import unittest

from dynamic_obstacle_benchmarks.benchmark_runner import LiveEventEpisodes
from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D
from dynamic_obstacle_benchmarks.goal_sampler import GoalSampler
from dynamic_obstacle_benchmarks.person_avoidance_benchmark import apply_overrides


class RunnerSupportTests(unittest.TestCase):
    def test_live_debug_states_are_counted_as_episodes(self) -> None:
        episodes = LiveEventEpisodes()
        emergency = {"status": "SAFETY_FILTER_STOPPING"}
        normal = {"status": "MOVING_OR_READY"}
        stale = {"mode": "stale_input", "reason": "stale_input"}
        safety_normal = {"mode": "pass_through", "reason": "pass_through"}
        for payload in (emergency, emergency, normal, emergency, stale, stale, safety_normal, stale):
            episodes.update(payload)
        self.assertEqual(episodes.emergency_stops, 2)
        self.assertEqual(episodes.stale_events, 2)

    def test_goal_sampler_resamples_when_live_validator_rejects(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, people_count=0)
        attempts = []

        def validator(start, goal):
            attempts.append((start, goal))
            return len(attempts) >= 2

        goals = GoalSampler(cfg, random.Random(4), validator).sample(Pose2D(0, 0), [])
        self.assertEqual(len(goals), 1)
        self.assertEqual(len(attempts), 2)

    def test_random_goal_is_sampled_from_current_robot_pose(self) -> None:
        cfg = BenchmarkConfig(goals_count=3, people_count=0, min_goal_distance_m=1.0)
        starts = []

        def validator(start, goal):
            starts.append(start)
            return True

        sampler = GoalSampler(cfg, random.Random(4), validator)
        current = Pose2D(1.25, -0.75, 0.0)
        goal = sampler.sample_random_goal(2, current, [])
        self.assertEqual(goal.index, 2)
        self.assertEqual(starts, [current])

    def test_mode_values_come_from_loaded_preset_before_cli_overrides(self) -> None:
        cfg = BenchmarkConfig()
        cfg.mode_presets = {
            "stress": {
                "people": 17,
                "goals": 13,
                "behaviours": ["custom"],
                "goal_mode": "curriculum",
                "max_goal_time_s": 44,
            }
        }
        args = argparse.Namespace(
            mode="stress", backend=None, seed=None, people=3, goals=None, headless=False,
            record_rosbag=False, output=None, scenario_file=None,
        )
        apply_overrides(cfg, args)
        self.assertEqual(cfg.people_count, 3)
        self.assertEqual(cfg.goals_count, 13)
        self.assertEqual(cfg.behaviours, ["custom"])
        self.assertEqual(cfg.goal_mode, "curriculum")
        self.assertEqual(cfg.max_goal_time_s, 44)


if __name__ == "__main__":
    unittest.main()
