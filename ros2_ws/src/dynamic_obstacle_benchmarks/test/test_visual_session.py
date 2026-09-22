import json

import pytest

from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D
from dynamic_obstacle_benchmarks.goal_sampler import Goal
from dynamic_obstacle_benchmarks.metrics import BenchmarkMetrics
from dynamic_obstacle_benchmarks.person_avoidance_benchmark import apply_overrides, build_parser
from dynamic_obstacle_benchmarks.result_writer import ResultWriter
from dynamic_obstacle_benchmarks.scoring import ScoreBreakdown


def test_visual_session_has_long_defaults_and_explicit_short_overrides():
    config = BenchmarkConfig(backend="mujoco", onboard_only=True)
    apply_overrides(config, build_parser().parse_args(["--visual-session"]))
    assert config.visual_session and config.total_timeout_s == 1800.
    assert config.goals_count == 1_000_000
    apply_overrides(config, build_parser().parse_args(["--visual-session", "--duration-seconds", "60", "--goals", "3"]))
    assert config.total_timeout_s == 60. and config.goals_count == 3


@pytest.mark.parametrize("collisions", [0, 2])
def test_visual_session_records_contacts_and_cannot_claim_benchmark_success(tmp_path, collisions):
    config = BenchmarkConfig(backend="mujoco", onboard_only=True, visual_session=True,
        total_timeout_s=60., goals_count=1, output_dir=str(tmp_path))
    writer = ResultWriter(config, str(tmp_path / "session.json"))
    goals = [Goal(0, Pose2D(1., 0.))]
    scenario = writer.write_scenario(config, goals, [])
    metrics = BenchmarkMetrics(goals_requested=1, goals_reached=1, collisions=collisions, total_time_s=60.)
    result = json.loads(writer.write(config, metrics, ScoreBreakdown(100, 100, 100, 100, 100, 100), goals, [], scenario).read_text())
    assert result["visual_session"] and not result["success"]
    assert result["collisions"] == collisions
    assert result["session_end_reason"] == "duration_elapsed"


def test_visual_session_requires_onboard_live_mode_and_finite_duration():
    with pytest.raises(ValueError, match="onboard MuJoCo"):
        apply_overrides(BenchmarkConfig(), build_parser().parse_args(["--visual-session"]))
    with pytest.raises(ValueError, match="finite and positive"):
        apply_overrides(BenchmarkConfig(), build_parser().parse_args(["--duration-seconds", "nan"]))
