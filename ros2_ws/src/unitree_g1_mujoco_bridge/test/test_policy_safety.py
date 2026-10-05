import math

import pytest

from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS
from unitree_g1_mujoco_bridge.policy_safety import (
    G1_JOINT_LIMITS,
    advance_gait_phase,
    apply_command_deadband,
    axis_isolated_command,
    cleared_motion_state,
    clamp_joint_targets,
    ensure_forward_turning_arc,
    filter_command,
    pending_target,
    sample_age,
    startup_active,
    validate_metadata_joint_order,
)


def test_gait_phase_advances_and_resets_with_motion_command():
    phase = advance_gait_phase(0.0, (0.15, 0.0, 0.0), 0.02, 0.8)
    assert phase == pytest.approx(0.025)
    assert advance_gait_phase(phase, (0.0, 0.0, 0.0), 0.02, 0.8) == 0.0
    assert advance_gait_phase(0.99, (0.0, 0.0, 0.2), 0.02, 0.8) == pytest.approx(
        0.015
    )


LOWER_BODY_JOINTS = G1_29DOF_JOINTS[:12]


def test_sample_freshness_handles_missing_stale_and_clock_regression():
    assert math.isinf(sample_age(10.0, None))
    assert sample_age(10.0, 9.8) == pytest.approx(0.2)
    assert sample_age(9.0, 10.0) == 0.0


def test_reset_motion_state_clears_base_velocity_and_sensor_timestamps():
    velocity, lowstate_arrival, odom_arrival = cleared_motion_state()
    assert velocity == [0.0, 0.0, 0.0]
    assert lowstate_arrival is None
    assert odom_arrival is None


def test_deadband_hysteresis_prevents_low_speed_chatter():
    command, stopped = apply_command_deadband([0.05, 0.0, 0.0], False, 0.05, 0.01)
    assert stopped and command == [0.0, 0.0, 0.0]
    command, stopped = apply_command_deadband([0.055, 0.0, 0.0], stopped, 0.05, 0.01)
    assert stopped and command == [0.0, 0.0, 0.0]
    command, stopped = apply_command_deadband([0.061, 0.0, 0.0], stopped, 0.05, 0.01)
    assert not stopped and command[0] == pytest.approx(0.061)


def test_zero_or_timeout_bypasses_acceleration_filter():
    assert filter_command(
        [0.3, -0.1, 0.2], [0.0, 0.0, 0.0], [0.1] * 3, 0.01, True
    ) == [0.0] * 3
    ramped = filter_command(
        [0.0] * 3, [0.3, 0.0, 0.0], [0.1] * 3, 0.1, False
    )
    assert ramped[0] == pytest.approx(0.01)


def test_axis_isolated_commands_rotate_then_translate():
    command, turning = axis_isolated_command([-0.05, 0.0, -0.10], 0.05)
    assert turning
    assert command == [0.0, 0.0, -0.10]
    command, turning = axis_isolated_command([0.05, 0.0, -0.04], 0.05)
    assert not turning
    assert command == [0.05, 0.0, 0.0]


def test_disabled_turning_arc_preserves_reverse_and_turn_requests():
    assert ensure_forward_turning_arc([-.3,.1,.35],.05,0.,0.) == [-.3,.1,.35]


def test_turning_arc_adds_forward_floor_only_while_turning():
    assert ensure_forward_turning_arc([0.0, 0.0, -0.10], 0.05, 0.10) == [
        0.10,
        0.0,
        -0.10,
    ]
    assert ensure_forward_turning_arc([0.12, 0.0, -0.10], 0.05, 0.10) == [
        0.12,
        0.0,
        -0.10,
    ]
    assert ensure_forward_turning_arc([0.0, 0.0, -0.04], 0.05, 0.10) == [
        0.0,
        0.0,
        -0.04,
    ]


def test_turning_arc_promotes_small_active_yaw_to_policy_minimum():
    assert ensure_forward_turning_arc([0.0, 0.0, -0.04], 0.03, 0.06, 0.12) == [
        0.06,
        0.0,
        -0.12,
    ]
    assert ensure_forward_turning_arc([0.0, 0.0, 0.20], 0.03, 0.06, 0.12) == [
        0.06,
        0.0,
        0.20,
    ]


def test_joint_targets_are_clamped_inside_mjcf_soft_limits():
    targets = clamp_joint_targets([100.0] * 29)
    for target, (lower, upper) in zip(targets, G1_JOINT_LIMITS):
        assert lower < target < upper
        expected_upper = 0.5 * (lower + upper) + 0.45 * (upper - lower)
        assert target == pytest.approx(expected_upper)


def test_metadata_joint_order_mismatch_is_rejected():
    validate_metadata_joint_order(
        G1_29DOF_JOINTS, G1_29DOF_JOINTS, LOWER_BODY_JOINTS, 29
    )
    validate_metadata_joint_order(
        LOWER_BODY_JOINTS, G1_29DOF_JOINTS, LOWER_BODY_JOINTS, 12
    )
    validate_metadata_joint_order(
        G1_29DOF_JOINTS, G1_29DOF_JOINTS, LOWER_BODY_JOINTS, 12
    )
    mismatched = list(G1_29DOF_JOINTS)
    mismatched[0], mismatched[1] = mismatched[1], mismatched[0]
    with pytest.raises(ValueError, match="joint_names metadata"):
        validate_metadata_joint_order(mismatched, G1_29DOF_JOINTS, LOWER_BODY_JOINTS)
    with pytest.raises(ValueError, match="joint_names metadata"):
        validate_metadata_joint_order(
            LOWER_BODY_JOINTS, G1_29DOF_JOINTS, LOWER_BODY_JOINTS, 29
        )


def test_startup_never_releases_while_upright_is_missing():
    assert startup_active(100.0, True, None, 0.25, 2.0, 0.5)
    assert not startup_active(100.0, False, None, 0.25, 2.0, 0.5)
    assert startup_active(3.0, True, 0.2, 0.25, 2.0, 0.5)
    assert not startup_active(3.0, True, 0.6, 0.25, 2.0, 0.5)


def test_pending_inference_refreshes_last_safe_target():
    assert pending_target([1.0, 2.0], [3.0, 4.0]) == [1.0, 2.0]
    assert pending_target(None, [3.0, 4.0]) == [3.0, 4.0]
