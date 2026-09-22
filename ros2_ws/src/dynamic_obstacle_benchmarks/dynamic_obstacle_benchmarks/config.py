from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import math

import yaml


@dataclass
class Bounds:
    x_min: float = -5.0
    x_max: float = 5.0
    y_min: float = -5.0
    y_max: float = 5.0


@dataclass
class BenchmarkConfig:
    name: str = "person_avoidance_benchmark"
    mode: str = "standard"
    seed: int = 42
    backend: str = "mock"
    headless: bool = True
    use_sim_time: bool = True
    output_dir: str = "results/person_avoidance"
    output: str | None = None
    scenario_file: str | None = None
    record_rosbag: bool = False
    map_frame: str = "map"
    odom_frame: str = "odom"
    robot_frame: str = "base_link"
    bounds: Bounds = field(default_factory=Bounds)
    goal_bounds: Bounds | None = None
    people_count: int = 8
    goals_count: int = 10
    goal_mode: str = "random"
    fixed_goals: list[dict[str, float]] = field(default_factory=list)
    robot_radius_m: float = 0.35
    person_radius_m: float = 0.30
    nominal_speed_mps: float = 0.5
    max_speed_mps: float = 1.0
    goal_tolerance_m: float = 0.25
    max_goal_time_s: float = 60.0
    total_timeout_s: float = 600.0
    min_goal_distance_m: float = 2.0
    max_goal_distance_m: float | None = None
    max_goal_heading_change_rad: float = 3.141592653589793
    min_distance_from_walls_m: float = 0.5
    max_sampling_attempts: int = 1000
    path_validation_timeout_s: float = 5.0
    people_speed_min_mps: float = 0.2
    people_speed_max_mps: float = 1.2
    pedestrian_scenario: str = "waypoints"
    behaviours: list[str] = field(default_factory=lambda: ["straight_line_crossing", "random_waypoint", "opposing_flow", "stop_start"])
    spawn_min_distance_from_robot_m: float = 1.5
    spawn_min_distance_from_goal_m: float = 1.0
    spawn_min_distance_between_people_m: float = 0.8
    warning_margin_m: float = 0.75
    near_miss_margin_m: float = 0.35
    critical_margin_m: float = 0.10
    near_miss_min_duration_s: float = 0.10
    ttc_horizon_s: float = 3.0
    ttc_warning_s: float = 1.50
    ttc_critical_s: float = 0.75
    odom_stale_timeout_s: float = 1.0
    pedestrian_stale_timeout_s: float = 1.0
    scan_stale_timeout_s: float = 1.0
    require_exact_people_count: bool = True
    control_people_source: str = "auto"
    scoring_people_source: str = "auto"
    scan_source: str = "auto"
    onboard_only: bool = False
    visual_session: bool = False
    dt_s: float = 0.05
    mode_presets: dict[str, dict[str, Any]] = field(default_factory=dict, repr=False)

    @property
    def collision_distance_m(self) -> float:
        return self.robot_radius_m + self.person_radius_m

    @property
    def critical_distance_m(self) -> float:
        return self.collision_distance_m + self.critical_margin_m

    @property
    def near_miss_distance_m(self) -> float:
        return self.collision_distance_m + self.near_miss_margin_m

    @property
    def warning_distance_m(self) -> float:
        return self.collision_distance_m + self.warning_margin_m

    def pose_in_valid_area(self, pose) -> bool:
        return self.bounds.x_min <= pose.x <= self.bounds.x_max and self.bounds.y_min <= pose.y <= self.bounds.y_max

    def validate(self) -> None:
        if self.visual_session and (self.backend != "mujoco" or not self.onboard_only):
            raise ValueError("visual_session requires the onboard MuJoCo configuration")
        if self.people_count < 0 or self.goals_count < 1:
            raise ValueError("people_count must be >= 0 and goals_count must be >= 1")
        if self.max_sampling_attempts < 1:
            raise ValueError("max_sampling_attempts must be >= 1")
        if self.max_goal_distance_m is not None and (not math.isfinite(self.max_goal_distance_m) or self.max_goal_distance_m <= self.min_goal_distance_m):
            raise ValueError("maximum goal distance must exceed minimum goal distance")
        if not 0.0 < self.max_goal_heading_change_rad <= 3.141592653589793:
            raise ValueError("max_goal_heading_change_rad must be in (0, pi]")
        if self.bounds.x_min >= self.bounds.x_max or self.bounds.y_min >= self.bounds.y_max:
            raise ValueError("environment bounds are invalid")
        if self.goal_bounds is not None:
            b = self.goal_bounds
            if not all(math.isfinite(v) for v in (b.x_min, b.x_max, b.y_min, b.y_max)) or not (
                self.bounds.x_min <= b.x_min < b.x_max <= self.bounds.x_max
                and self.bounds.y_min <= b.y_min < b.y_max <= self.bounds.y_max
            ):
                raise ValueError("goal bounds must be finite and inside environment bounds")
        positive = {
            "robot_radius_m": self.robot_radius_m,
            "person_radius_m": self.person_radius_m,
            "nominal_speed_mps": self.nominal_speed_mps,
            "max_speed_mps": self.max_speed_mps,
            "goal_tolerance_m": self.goal_tolerance_m,
            "max_goal_time_s": self.max_goal_time_s,
            "total_timeout_s": self.total_timeout_s,
            "dt_s": self.dt_s,
            "path_validation_timeout_s": self.path_validation_timeout_s,
            "odom_stale_timeout_s": self.odom_stale_timeout_s,
            "pedestrian_stale_timeout_s": self.pedestrian_stale_timeout_s,
            "scan_stale_timeout_s": self.scan_stale_timeout_s,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.near_miss_distance_m <= self.collision_distance_m:
            raise ValueError("near_miss_distance must be greater than collision_distance")
        if self.critical_margin_m < 0.0:
            raise ValueError("critical_margin_m must be non-negative")
        if self.near_miss_margin_m <= self.critical_margin_m:
            raise ValueError("near_miss_margin_m must be greater than critical_margin_m")
        if self.warning_distance_m <= self.near_miss_distance_m:
            raise ValueError("warning_distance must be greater than near_miss_distance")
        if self.near_miss_min_duration_s < 0.0:
            raise ValueError("near_miss_min_duration_s must be non-negative")
        if not 0.0 < self.ttc_critical_s <= self.ttc_warning_s <= self.ttc_horizon_s:
            raise ValueError(
                "TTC thresholds must satisfy 0 < critical <= warning <= horizon"
            )
        for name in ("control_people_source", "scoring_people_source", "scan_source"):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"{name} must not be empty")


def load_config(path: str | None = None) -> BenchmarkConfig:
    data: dict[str, Any] = {}
    if path:
        config_path = Path(path)
        if not config_path.exists():
            raise FileNotFoundError(f"Benchmark config not found: {path}")
        data = yaml.safe_load(config_path.read_text()) or {}
    cfg = BenchmarkConfig()
    _apply_mapping(cfg, data)
    presets_path = (Path(path).parent if path else Path(__file__).parents[1] / "config") / "person_scenarios.yaml"
    if presets_path.exists():
        cfg.mode_presets = yaml.safe_load(presets_path.read_text()) or {}
    cfg.validate()
    return cfg


def _apply_mapping(cfg: BenchmarkConfig, data: dict[str, Any]) -> None:
    benchmark = data.get("benchmark", {})
    environment = data.get("environment", {})
    robot = data.get("robot", {})
    people = data.get("people", {})
    goals = data.get("goals", {})
    safety = data.get("safety", {})
    simulation = data.get("simulation", {})
    data_health = data.get("data_health", {})
    data_sources = data.get("data_sources", {})

    cfg.name = benchmark.get("name", cfg.name)
    cfg.mode = str(benchmark.get("mode", cfg.mode))
    cfg.seed = int(benchmark.get("seed", cfg.seed))
    cfg.backend = simulation.get("backend", benchmark.get("backend", cfg.backend))
    cfg.headless = _boolean(benchmark, "headless", cfg.headless)
    cfg.output_dir = str(benchmark.get("output_dir", cfg.output_dir))
    cfg.record_rosbag = _boolean(benchmark, "record_rosbag", cfg.record_rosbag)
    cfg.total_timeout_s = float(benchmark.get("total_timeout_s", cfg.total_timeout_s))
    cfg.use_sim_time = _boolean(benchmark, "use_sim_time", cfg.use_sim_time)
    cfg.onboard_only = _boolean(benchmark, "onboard_only", cfg.onboard_only)
    cfg.visual_session = _boolean(benchmark, "visual_session", cfg.visual_session)
    cfg.map_frame = environment.get("map_frame", cfg.map_frame)
    cfg.odom_frame = environment.get("odom_frame", cfg.odom_frame)
    cfg.robot_frame = environment.get("robot_frame", cfg.robot_frame)
    if "bounds" in environment:
        b = environment["bounds"]
        cfg.bounds = Bounds(float(b["x_min"]), float(b["x_max"]), float(b["y_min"]), float(b["y_max"]))
    if "bounds" in goals:
        b = goals["bounds"]
        cfg.goal_bounds = Bounds(float(b["x_min"]), float(b["x_max"]), float(b["y_min"]), float(b["y_max"]))
    cfg.robot_radius_m = float(robot.get("radius_m", cfg.robot_radius_m))
    cfg.nominal_speed_mps = float(robot.get("nominal_speed_mps", cfg.nominal_speed_mps))
    cfg.max_speed_mps = float(robot.get("max_speed_mps", cfg.max_speed_mps))
    cfg.goal_tolerance_m = float(robot.get("goal_tolerance_m", cfg.goal_tolerance_m))
    cfg.max_goal_time_s = float(robot.get("max_goal_time_s", cfg.max_goal_time_s))
    cfg.people_count = int(people.get("count", cfg.people_count))
    cfg.person_radius_m = float(people.get("radius_m", cfg.person_radius_m))
    cfg.people_speed_min_mps = float(people.get("speed_min_mps", cfg.people_speed_min_mps))
    cfg.people_speed_max_mps = float(people.get("speed_max_mps", cfg.people_speed_max_mps))
    cfg.pedestrian_scenario = str(people.get("scenario", cfg.pedestrian_scenario))
    cfg.behaviours = list(people.get("behaviours", cfg.behaviours))
    cfg.spawn_min_distance_from_robot_m = float(
        people.get("spawn_min_distance_from_robot_m", cfg.spawn_min_distance_from_robot_m)
    )
    cfg.spawn_min_distance_from_goal_m = float(
        people.get("spawn_min_distance_from_goal_m", cfg.spawn_min_distance_from_goal_m)
    )
    cfg.spawn_min_distance_between_people_m = float(
        people.get("spawn_min_distance_between_people_m", cfg.spawn_min_distance_between_people_m)
    )
    cfg.goals_count = int(goals.get("count", cfg.goals_count))
    cfg.goal_mode = str(goals.get("mode", cfg.goal_mode))
    cfg.fixed_goals = list(goals.get("fixed", cfg.fixed_goals))
    cfg.min_goal_distance_m = float(goals.get("min_distance_between_goals_m", cfg.min_goal_distance_m))
    if "max_distance_between_goals_m" in goals:
        cfg.max_goal_distance_m = float(goals["max_distance_between_goals_m"])
    cfg.max_goal_heading_change_rad = float(
        goals.get("max_heading_change_rad", cfg.max_goal_heading_change_rad)
    )
    cfg.min_distance_from_walls_m = float(goals.get("min_distance_from_walls_m", cfg.min_distance_from_walls_m))
    cfg.max_sampling_attempts = int(goals.get("max_sampling_attempts", cfg.max_sampling_attempts))
    cfg.path_validation_timeout_s = float(
        goals.get("path_validation_timeout_s", cfg.path_validation_timeout_s)
    )
    cfg.warning_margin_m = float(safety.get("warning_margin_m", cfg.warning_margin_m))
    cfg.near_miss_margin_m = float(safety.get("near_miss_margin_m", cfg.near_miss_margin_m))
    cfg.critical_margin_m = float(safety.get("critical_margin_m", cfg.critical_margin_m))
    cfg.near_miss_min_duration_s = float(safety.get("near_miss_min_duration_s", cfg.near_miss_min_duration_s))
    cfg.ttc_horizon_s = float(safety.get("ttc_horizon_s", cfg.ttc_horizon_s))
    cfg.ttc_warning_s = float(safety.get("ttc_warning_s", cfg.ttc_warning_s))
    cfg.ttc_critical_s = float(safety.get("ttc_critical_s", cfg.ttc_critical_s))
    cfg.odom_stale_timeout_s = float(
        data_health.get("odom_stale_timeout_s", cfg.odom_stale_timeout_s)
    )
    cfg.pedestrian_stale_timeout_s = float(
        data_health.get("pedestrian_stale_timeout_s", cfg.pedestrian_stale_timeout_s)
    )
    cfg.scan_stale_timeout_s = float(
        data_health.get("scan_stale_timeout_s", cfg.scan_stale_timeout_s)
    )
    cfg.require_exact_people_count = _boolean(
        data_health, "require_exact_people_count", cfg.require_exact_people_count
    )
    cfg.control_people_source = str(
        data_sources.get("control_people", cfg.control_people_source)
    )
    cfg.scoring_people_source = str(
        data_sources.get("scoring_people", cfg.scoring_people_source)
    )
    cfg.scan_source = str(data_sources.get("scan", cfg.scan_source))
    cfg.dt_s = float(simulation.get("dt_s", cfg.dt_s))


def _boolean(values: dict[str, Any], key: str, default: bool) -> bool:
    value = values.get(key, default)
    if type(value) is not bool:
        raise ValueError(f"{key} must be a boolean")
    return value
