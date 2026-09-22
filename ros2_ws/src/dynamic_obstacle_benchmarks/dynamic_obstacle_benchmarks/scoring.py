from __future__ import annotations

from dataclasses import dataclass

from .config import BenchmarkConfig
from .metrics import BenchmarkMetrics


@dataclass(frozen=True)
class ScoreBreakdown:
    safety_score: float
    task_score: float
    speed_score: float
    efficiency_score: float
    hard_penalties: float
    final_score: float

    def as_dict(self) -> dict:
        return {
            "safety_score": self.safety_score,
            "task_score": self.task_score,
            "speed_score": self.speed_score,
            "efficiency_score": self.efficiency_score,
            "hard_penalties": self.hard_penalties,
            "final_score": self.final_score,
        }


def clamp_score(value: float) -> float:
    return max(0.0, min(100.0, value))


def score_benchmark(metrics: BenchmarkMetrics, config: BenchmarkConfig) -> ScoreBreakdown:
    if not metrics.valid_trial:
        return ScoreBreakdown(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    safety = 100.0
    safety -= 50.0 * metrics.collisions
    safety -= 25.0 * metrics.critical_near_misses
    safety -= 10.0 * metrics.near_misses
    safety -= 0.5 * metrics.warning_distance_wall_time_s
    safety -= 2.0 * metrics.near_miss_distance_wall_time_s
    safety -= 5.0 * metrics.critical_distance_wall_time_s
    if metrics.minimum_ttc_s <= config.ttc_critical_s:
        safety -= 10.0
    safety -= 5.0 * metrics.stale_events
    safety = clamp_score(safety)

    task = clamp_score(100.0 * metrics.goals_reached / max(1, metrics.goals_requested))
    reference_time = metrics.ideal_path_length_m / max(config.nominal_speed_mps, 1e-6)
    speed = 0.0
    if metrics.total_time_s > 1e-6 and reference_time > 1e-6:
        speed = 100.0 * min(1.5, reference_time / metrics.total_time_s) / 1.5
    speed = clamp_score(speed)
    efficiency = clamp_score(100.0 * metrics.efficiency)

    hard_penalties = 0.0
    if metrics.left_valid_area:
        hard_penalties += 30.0
    if metrics.speed_limit_violations:
        hard_penalties += 20.0

    if metrics.collisions > 0 or metrics.no_movement or metrics.fallen:
        return ScoreBreakdown(
            round(safety, 3),
            round(task, 3),
            round(speed, 3),
            round(efficiency, 3),
            round(hard_penalties, 3),
            0.0,
        )

    final = 0.65 * safety + 0.15 * task + 0.10 * speed + 0.10 * efficiency - hard_penalties
    return ScoreBreakdown(round(safety, 3), round(task, 3), round(speed, 3), round(efficiency, 3), round(hard_penalties, 3), round(clamp_score(final), 3))
