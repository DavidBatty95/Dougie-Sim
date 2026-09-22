import unittest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig, Bounds
from dynamic_obstacle_benchmarks.geometry import Pose2D, Velocity2D
from dynamic_obstacle_benchmarks.metrics import MetricsMonitor
from dynamic_obstacle_benchmarks.person_motion_models import PersonState


def person(
    x: float,
    velocity_x: float = 0.0,
    person_id: int = 1,
    radius_m: float = 0.30,
) -> PersonState:
    return PersonState(
        person_id,
        Pose2D(x, 0.0),
        Velocity2D(velocity_x, 0.0),
        radius_m,
        "test",
        abs(velocity_x),
    )


class MetricsTests(unittest.TestCase):
    def test_reports_center_distance_clearance_and_legacy_alias(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        monitor.update(0.05, 0.05, Pose2D(0, 0), Velocity2D(), [person(1.2)])

        self.assertAlmostEqual(monitor.metrics.minimum_separation_m, 1.2)
        self.assertAlmostEqual(monitor.metrics.minimum_center_distance_m, 1.2)
        self.assertAlmostEqual(monitor.metrics.minimum_body_clearance_m, 0.55)
        self.assertAlmostEqual(monitor.metrics.mean_center_distance_m, 1.2)
        self.assertAlmostEqual(monitor.metrics.mean_body_clearance_m, 0.55)

    def test_near_miss_requires_minimum_duration_and_counts_new_episode(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, near_miss_min_duration_s=0.10)
        monitor = MetricsMonitor(cfg)
        close = [person(0.9)]
        monitor.update(0.04, 0.04, Pose2D(0, 0), Velocity2D(), close)
        monitor.update(0.08, 0.04, Pose2D(0, 0), Velocity2D(), close)
        self.assertEqual(monitor.metrics.near_misses, 0)

        monitor.update(0.12, 0.04, Pose2D(0, 0), Velocity2D(), close)
        monitor.update(0.16, 0.04, Pose2D(0, 0), Velocity2D(), close)
        self.assertEqual(monitor.metrics.near_misses, 1)
        event = next(event for event in monitor.metrics.events if event.type == "near_miss")
        self.assertAlmostEqual(event.time_s, 0.04)
        self.assertAlmostEqual(event.center_distance_m, 0.9)
        self.assertAlmostEqual(event.body_clearance_m, 0.25)

        monitor.update(0.20, 0.04, Pose2D(0, 0), Velocity2D(), [person(1.1)])
        for now in (0.24, 0.28, 0.32):
            monitor.update(now, 0.04, Pose2D(0, 0), Velocity2D(), close)
        self.assertEqual(monitor.metrics.near_misses, 2)

    def test_short_critical_distance_excursion_is_not_an_episode(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, near_miss_min_duration_s=0.10)
        monitor = MetricsMonitor(cfg)
        monitor.update(0.05, 0.05, Pose2D(0, 0), Velocity2D(), [person(0.70)])
        monitor.update(0.10, 0.05, Pose2D(0, 0), Velocity2D(), [person(0.80)])
        self.assertEqual(monitor.metrics.critical_near_misses, 0)

    def test_critical_ttc_is_one_event_per_contiguous_episode(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, near_miss_min_duration_s=0.0)
        monitor = MetricsMonitor(cfg)
        approaching = [person(1.0)]
        monitor.update(0.05, 0.05, Pose2D(0, 0), Velocity2D(2.0, 0.0), approaching)
        monitor.update(0.10, 0.05, Pose2D(0, 0), Velocity2D(2.0, 0.0), approaching)
        self.assertEqual(sum(event.type == "critical_ttc" for event in monitor.metrics.events), 1)

        monitor.update(0.15, 0.05, Pose2D(0, 0), Velocity2D(-2.0, 0.0), approaching)
        monitor.update(0.20, 0.05, Pose2D(0, 0), Velocity2D(2.0, 0.0), approaching)
        self.assertEqual(sum(event.type == "critical_ttc" for event in monitor.metrics.events), 2)

        monitor.update(0.25, 0.05, Pose2D(0, 0), Velocity2D(), [])
        monitor.update(0.30, 0.05, Pose2D(0, 0), Velocity2D(2.0, 0.0), approaching)
        self.assertEqual(sum(event.type == "critical_ttc" for event in monitor.metrics.events), 3)

    def test_ttc_is_first_contact_and_tcpa_accompanies_minimum_dcpa(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        monitor.update(
            0.05,
            0.05,
            Pose2D(0, 0),
            Velocity2D(2.0, 0.0),
            [person(1.0)],
        )
        self.assertAlmostEqual(monitor.metrics.minimum_ttc_s, 0.175)
        self.assertAlmostEqual(monitor.metrics.minimum_dcpa_m, 0.0)
        self.assertAlmostEqual(monitor.metrics.tcpa_at_minimum_dcpa_s, 0.5)
        event = next(event for event in monitor.metrics.events if event.type == "critical_ttc")
        self.assertAlmostEqual(event.ttc_s, 0.175)
        self.assertAlmostEqual(event.tcpa_s, 0.5)
        self.assertAlmostEqual(event.dcpa_m, 0.0)

    def test_proximity_reports_wall_time_and_person_time(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        close_people = [person(0.9), person(0.9, person_id=2)]
        monitor.update(0.1, 0.1, Pose2D(0, 0), Velocity2D(), close_people)

        self.assertAlmostEqual(monitor.metrics.warning_distance_wall_time_s, 0.1)
        self.assertAlmostEqual(monitor.metrics.near_miss_distance_wall_time_s, 0.1)
        self.assertAlmostEqual(monitor.metrics.critical_distance_wall_time_s, 0.0)
        self.assertAlmostEqual(monitor.metrics.warning_distance_person_time_s, 0.2)
        self.assertAlmostEqual(monitor.metrics.near_miss_distance_person_time_s, 0.2)
        self.assertAlmostEqual(monitor.metrics.critical_distance_person_time_s, 0.0)
        self.assertAlmostEqual(monitor.metrics.time_below_warning_distance_s, 0.2)
        self.assertAlmostEqual(monitor.metrics.time_below_near_miss_distance_s, 0.2)

    def test_person_radius_is_used_for_clearance_thresholds_and_ttc(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        monitor.update(
            0.1,
            0.1,
            Pose2D(0, 0),
            Velocity2D(1.0, 0.0),
            [person(1.0, radius_m=0.50)],
        )
        self.assertAlmostEqual(monitor.metrics.minimum_body_clearance_m, 0.15)
        self.assertAlmostEqual(monitor.metrics.minimum_ttc_s, 0.15)
        self.assertAlmostEqual(monitor.metrics.near_miss_distance_wall_time_s, 0.1)

    def test_mean_clearance_is_time_weighted(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        monitor.update(1.0, 1.0, Pose2D(0, 0), Velocity2D(), [person(1.0)])
        monitor.update(4.0, 3.0, Pose2D(0, 0), Velocity2D(), [person(2.0)])
        self.assertAlmostEqual(monitor.metrics.mean_center_distance_m, 1.75)
        self.assertAlmostEqual(monitor.metrics.mean_body_clearance_m, 1.10)

    def test_speed_and_bounds_are_measured_without_people(self) -> None:
        cfg = BenchmarkConfig(goals_count=1, max_speed_mps=1.0, bounds=Bounds(-1, 1, -1, 1))
        monitor = MetricsMonitor(cfg)
        monitor.update(0.1, 0.1, Pose2D(2, 0), Velocity2D(1.2, 0), [])
        monitor.update(0.2, 0.1, Pose2D(2, 0), Velocity2D(1.3, 0), [])
        self.assertEqual(monitor.metrics.speed_limit_violations, 1)
        self.assertTrue(monitor.metrics.left_valid_area)

    def test_moving_pedestrian_time_counts_wall_time_not_people(self) -> None:
        monitor = MetricsMonitor(BenchmarkConfig(goals_count=1))
        moving_people = [
            person(2.0, 0.5),
            PersonState(2, Pose2D(3.0, 0.0), Velocity2D(-0.4, 0.0), 0.30, "test", 0.4),
        ]
        monitor.update(0.1, 0.1, Pose2D(0.0, 0.0), Velocity2D(), moving_people)
        monitor.update(0.2, 0.1, Pose2D(0.0, 0.0), Velocity2D(), moving_people)
        self.assertAlmostEqual(monitor.metrics.moving_pedestrian_time_s, 0.2)


if __name__ == "__main__":
    unittest.main()
