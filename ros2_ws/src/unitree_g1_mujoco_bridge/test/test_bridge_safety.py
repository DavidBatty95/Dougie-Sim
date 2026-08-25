import math

import pytest

from unitree_g1_mujoco_bridge.bridge_safety import (
    blocked_motion_state,
    clamp_velocity,
    no_unrequested_forward,
    relative_motion_metrics,
    scan_is_fresh,
    stop_velocity,
    trajectory_increases_clearance,
    trajectory_makes_target_progress,
    trajectory_static_cost,
    ttc_is_relevant,
)


def test_approaching_person_has_future_relevant_ttc():
    metrics = relative_motion_metrics(0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0)
    assert metrics["closing"] is True
    assert metrics["tcpa"] == pytest.approx(2.0)
    assert metrics["dcpa"] == pytest.approx(0.0)
    assert ttc_is_relevant(metrics, 3.0, 1.0)


@pytest.mark.parametrize(
    "robot_velocity,human_position",
    [((-1.0, 0.0), (2.0, 0.0)), ((1.0, 0.0), (-2.0, 0.0))],
)
def test_diverging_or_past_cpa_has_no_future_ttc(
    robot_velocity, human_position
):
    metrics = relative_motion_metrics(
        0.0, 0.0, *robot_velocity, *human_position, 0.0, 0.0
    )
    assert metrics["closing"] is False
    assert math.isinf(metrics["tcpa"])
    assert not ttc_is_relevant(metrics, 3.0, 1.0)


def test_zero_relative_velocity_has_no_ttc():
    metrics = relative_motion_metrics(0.0, 0.0, 0.4, 0.0, 2.0, 0.0, 0.4, 0.0)
    assert not metrics["closing"]
    assert math.isinf(metrics["tcpa"])


def test_ttc_requires_relevant_dcpa():
    metrics = relative_motion_metrics(0.0, 0.0, 1.0, 0.0, 2.0, 2.0, 0.0, 0.0)
    assert metrics["closing"]
    assert not ttc_is_relevant(metrics, 3.0, 1.0)


@pytest.mark.parametrize("requested", [0.0, -0.2])
def test_no_unrequested_forward_motion(requested):
    assert no_unrequested_forward(0.18, requested) == 0.0
    assert no_unrequested_forward(-0.1, requested) == -0.1


def test_static_obstacle_rejects_escape_trajectory():
    trajectory = [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.6, 0.0, 0.0)]
    cost, rejected = trajectory_static_cost(
        trajectory, [(0.55, 0.0)], 0.2, 1.0
    )
    assert rejected
    assert math.isinf(cost)


def test_clear_escape_trajectory_is_allowed():
    trajectory = [(0.0, 0.0, 0.0), (0.0, 0.3, 0.0)]
    _, rejected = trajectory_static_cost(trajectory, [(1.0, 0.0)], 0.2, 1.5)
    assert not rejected


def test_missing_and_stale_scan_are_not_fresh():
    assert not scan_is_fresh(None, 10.0, 0.5)
    assert not scan_is_fresh(9.0, 10.0, 0.5)
    assert scan_is_fresh(9.7, 10.0, 0.5)


def test_safe_patch_motion_must_increase_clearance_without_initial_dip():
    risk = [(1.0, 0.0)]
    assert trajectory_increases_clearance(-0.10, 0.0, 0.0, risk, 1.0, 0.1)
    assert not trajectory_increases_clearance(0.10, 0.0, 0.0, risk, 1.0, 0.1)
    assert not trajectory_increases_clearance(0.0, 0.0, 0.0, risk, 1.0, 0.1)


def test_safe_patch_command_must_make_progress_toward_stamped_target():
    assert trajectory_makes_target_progress(0.10, 0.0, 0.0, 1.0, 0.0, 1.0, 0.1)
    assert not trajectory_makes_target_progress(-0.10, 0.0, 0.0, 1.0, 0.0, 1.0, 0.1)
    assert not trajectory_makes_target_progress(0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.1)
    assert trajectory_makes_target_progress(0.0, 0.0, 0.3, 0.0, 1.0, 1.0, 0.1)
    assert not trajectory_makes_target_progress(0.0, 0.0, -0.3, 0.0, 1.0, 1.0, 0.1)


def test_stop_semantics_are_true_zero_velocity():
    assert stop_velocity() == (0.0, 0.0, 0.0)


def test_yield_transitions_to_and_remains_in_deadlock_recovery():
    assert (
        blocked_motion_state("YIELD_TO_HUMAN", 3.9, 8.0, 4.0, 6.0) ==
        "YIELD_TO_HUMAN"
    )
    assert (
        blocked_motion_state("YIELD_TO_HUMAN", 4.0, 6.0, 4.0, 6.0) ==
        "DEADLOCK_RECOVERY"
    )
    assert (
        blocked_motion_state("DEADLOCK_RECOVERY", 0.1, 6.1, 4.0, 6.0) ==
        "DEADLOCK_RECOVERY"
    )


def test_final_velocity_clamps():
    assert clamp_velocity(2.0, -3.0, 4.0, 0.2, 0.1, 0.15, 0.5) == (
        0.2, -0.15, 0.5
    )
    assert clamp_velocity(-2.0, 3.0, -4.0, 0.2, 0.1, 0.15, 0.5) == (
        -0.1, 0.15, -0.5
    )
