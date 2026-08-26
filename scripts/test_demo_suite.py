from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest


def load_suite() -> ModuleType:
    path = Path(__file__).with_name("demo_suite.py")
    spec = importlib.util.spec_from_file_location("demo_suite", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_presets_are_scored_and_deduplicated(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=7, headless=True)
    selected = suite.select_cases(
        catalog, "locomotion", ["fixed-forward", "mock-avoidance"]
    )

    assert [case.name for case in selected] == ["mock-avoidance", "fixed-forward"]
    assert all(case.scored and not case.interactive for case in selected)


def test_interactive_case_cannot_be_mixed_with_batch_case(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=False)

    with pytest.raises(ValueError, match="interactive cases must be run alone"):
        suite.select_cases(catalog, None, ["waypoint-visual", "mock-avoidance"])


def test_official_policy_cases_pin_profile_and_disable_lateral(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=True)

    fixed = catalog["fixed-forward"]
    waypoint = catalog["waypoint-live"]
    social = catalog["social-live"]
    endurance = catalog["social-endurance"]
    visual = catalog["social-visual"]
    assert "--validate" in fixed.command
    assert "unitree_rl_lab_g1_29dof" in fixed.command
    assert "policy_profile:=unitree_rl_lab_g1_29dof" in social.command
    assert "max_lin_vel_y:=0.0" in social.command
    assert "max_lin_vel_x:=0.45" in waypoint.command
    assert "max_lin_vel_x:=0.45" in social.command
    assert "--startup-seconds" not in social.command
    assert any(arg.endswith("demo_suite_social_benchmark.yaml") for arg in social.command)
    assert "mujoco_pedestrian_start_delay:=45.0" in social.command
    assert "tracker_use_sensor_input:=false" in social.command
    assert "tracker_local_tracking_radius:=0.0" in social.command
    assert "livox_scan_points:=1200" in social.command
    assert any(arg.endswith("social_navigation_oracle_v1.yaml") for arg in social.command)
    assert "--require-avoidance" in endurance.command
    assert "mujoco_pedestrian_scenario:=demo_endurance" in endurance.command
    assert "mujoco_pedestrian_start_delay:=30.0" in endurance.command
    assert "max_lin_vel_y:=0.0" in endurance.command
    assert any(arg.endswith("demo_suite_social_endurance_benchmark.yaml") for arg in endurance.command)
    assert "--no-rviz" not in visual.command
    assert "--require-avoidance" in visual.command
    assert "mujoco_pedestrian_scenario:=demo_endurance" in visual.command
    assert "tracker_use_sensor_input:=false" in visual.command
    assert any(arg.endswith("social_navigation_oracle_v1.yaml") for arg in visual.command)


def test_dry_run_catalog_uses_headless_live_benchmarks(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=True)

    assert "--no-rviz" in catalog["waypoint-live"].command
    assert "--no-rviz" in catalog["social-live"].command
    assert "--no-rviz" in catalog["social-endurance"].command
