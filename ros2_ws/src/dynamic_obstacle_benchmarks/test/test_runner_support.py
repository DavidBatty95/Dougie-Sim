import argparse
import random
import unittest
from types import SimpleNamespace

from dynamic_obstacle_benchmarks.benchmark_runner import BenchmarkRosNode, LiveEventEpisodes, velocity_from_pose_delta
from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D, Velocity2D
from dynamic_obstacle_benchmarks.goal_sampler import GoalSampler
from dynamic_obstacle_benchmarks.person_avoidance_benchmark import apply_overrides


class RunnerSupportTests(unittest.TestCase):
    def test_goal_path_validation_fails_closed_without_action_server(self) -> None:
        class UnavailablePathClient:
            def wait_for_server(self, timeout_sec):
                return False

        node = object.__new__(BenchmarkRosNode)
        node.path_client = UnavailablePathClient()

        self.assertFalse(node.goal_has_path(Pose2D(0, 0), Pose2D(1, 0), timeout_s=0.01))

    def test_cancel_nav_goal_releases_active_handle(self) -> None:
        class CompletedFuture:
            def done(self):
                return True

        class GoalHandle:
            cancel_count = 0

            def cancel_goal_async(self):
                self.cancel_count += 1
                return CompletedFuture()

            def get_result_async(self):
                return CompletedFuture()

        node = object.__new__(BenchmarkRosNode)
        handle = GoalHandle()
        node.nav_goal_handle = handle

        node.cancel_nav_goal()

        self.assertIsNone(node.nav_goal_handle)
        self.assertEqual(handle.cancel_count, 1)

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

        for payload in ({"mode": "override"}, {"mode": "override"}, {"mode": "pass_through"}, {"mode": "override"}):
            episodes.update(payload)
        self.assertEqual(episodes.avoidance_interventions, 2)

    def test_goal_sampler_resamples_when_live_validator_rejects(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, people_count=0)
        attempts = []

        def validator(start, goal):
            attempts.append((start, goal))
            return len(attempts) >= 2

        goals = GoalSampler(cfg, random.Random(4), validator).sample(Pose2D(0, 0), [])
        self.assertEqual(len(goals), 1)
        self.assertEqual(len(attempts), 2)

    def test_pose_delta_velocity_recovers_missing_odom_twist(self) -> None:
        velocity = velocity_from_pose_delta(Pose2D(0.0, 0.0, 0.0), Pose2D(0.2, -0.1, 0.1), 0.5)
        self.assertIsNotNone(velocity)
        self.assertAlmostEqual(velocity.vx, 0.4)
        self.assertAlmostEqual(velocity.vy, -0.2)
        self.assertAlmostEqual(velocity.wz, 0.2)
        self.assertIsNone(velocity_from_pose_delta(Pose2D(0.0, 0.0), Pose2D(1.0, 0.0), 1.0))

    def test_missing_odom_twist_uses_windowed_pose_velocity(self) -> None:
        def odom(x, stamp):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp), nanosec=int(stamp % 1 * 1e9))),
                pose=SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )),
                twist=SimpleNamespace(twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0, y=0.0),
                    angular=SimpleNamespace(z=0.0),
                )),
            )

        node = object.__new__(BenchmarkRosNode)
        node.config = BenchmarkConfig(max_speed_mps=0.5)
        node.latest_robot_velocity = Velocity2D()
        node._last_odom_pose = None
        node._last_odom_stamp = None
        node._derived_robot_velocity = node.latest_robot_velocity
        node._odom_callback(odom(0.0, 1.0))
        node._odom_callback(odom(0.08, 1.1))
        self.assertAlmostEqual(node.latest_robot_velocity.vx, 0.0)
        node._odom_callback(odom(0.09, 1.3))
        self.assertAlmostEqual(node.latest_robot_velocity.vx, 0.3)

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
            }
        }
        cfg.max_goal_time_s = 77
        args = argparse.Namespace(
            mode="stress", backend=None, seed=None, people=3, goals=None, headless=False,
            record_rosbag=False, output=None, scenario_file=None,
        )
        apply_overrides(cfg, args)
        self.assertEqual(cfg.people_count, 3)
        self.assertEqual(cfg.goals_count, 13)
        self.assertEqual(cfg.behaviours, ["custom"])
        self.assertEqual(cfg.goal_mode, "curriculum")
        self.assertEqual(cfg.max_goal_time_s, 77)


if __name__ == "__main__":
    unittest.main()
