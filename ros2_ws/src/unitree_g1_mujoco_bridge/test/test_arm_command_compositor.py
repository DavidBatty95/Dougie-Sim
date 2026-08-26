import math

import pytest

from unitree_g1_mujoco_bridge.arm_command_compositor import (
    ARM_JOINTS,
    ArmCommandCompositorCore,
    TrajectoryError,
    TrajectoryPointData,
    interpolate_trajectory,
    parse_trajectory,
)


def body(value=0.0):
    return [float(value)] * 29


def acquired_core():
    core = ArmCommandCompositorCore(input_timeout=0.5)
    measured = body()
    measured[15:] = [index * 0.01 for index in range(14)]
    assert core.update_measured(measured, 1.0)
    assert core.update_locomotion(body(0.2), 1.0)
    assert core.acquire(1.1)[0]
    return core, measured


def test_transparent_pass_through_and_release() -> None:
    core = ArmCommandCompositorCore(input_timeout=0.25)
    policy = [index / 10.0 for index in range(29)]
    core.update_locomotion(policy, 0.0)
    assert core.output(0.1) == policy
    core.update_measured(body(), 0.1)
    assert core.acquire(0.1)[0]
    core.release()
    assert core.output(0.2) == policy


def test_pass_through_does_not_replay_a_stale_policy_command() -> None:
    core = ArmCommandCompositorCore(input_timeout=0.25)
    core.update_locomotion(body(0.2), 1.0)
    assert core.output(1.2) == body(0.2)
    assert core.output(1.3) is None


def test_acquisition_requires_fresh_state_and_seeds_measured_arm() -> None:
    core = ArmCommandCompositorCore(input_timeout=0.2)
    assert not core.acquire(0.0)[0]
    measured = body()
    measured[15:] = [0.1] * 14
    core.update_measured(measured, 1.0)
    assert not core.acquire(1.3)[0]
    assert core.acquire(1.1)[0]
    assert core.arm_hold == [0.1] * 14


def test_named_mapping_is_order_independent_and_interpolates() -> None:
    initial = [0.0] * 14
    trajectory = parse_trajectory(
        ["right_elbow_joint", "left_shoulder_pitch_joint"],
        [TrajectoryPointData((0.6, -0.4), 1.0)],
        initial,
    )
    halfway = interpolate_trajectory(trajectory, initial, 0.5)
    assert halfway[ARM_JOINTS.index("right_elbow_joint")] == pytest.approx(0.3)
    assert halfway[ARM_JOINTS.index("left_shoulder_pitch_joint")] == pytest.approx(-0.2)


def test_subset_trajectory_holds_unmentioned_joints() -> None:
    initial = [index * 0.01 for index in range(14)]
    trajectory = parse_trajectory(
        ["left_elbow_joint"], [TrajectoryPointData((0.5,), 1.0)], initial
    )
    output = interpolate_trajectory(trajectory, initial, 0.5)
    for index, value in enumerate(output):
        if ARM_JOINTS[index] == "left_elbow_joint":
            assert value == pytest.approx((initial[index] + 0.5) / 2.0)
        else:
            assert value == initial[index]


@pytest.mark.parametrize(
    "names,points",
    [
        ([ARM_JOINTS[0], ARM_JOINTS[0]], [TrajectoryPointData((0.0, 0.0), 1.0)]),
        (["waist_pitch_joint"], [TrajectoryPointData((0.0,), 1.0)]),
        (["not_a_joint"], [TrajectoryPointData((0.0,), 1.0)]),
        ([ARM_JOINTS[0]], []),
        ([ARM_JOINTS[0]], [TrajectoryPointData((), 1.0)]),
        ([ARM_JOINTS[0]], [TrajectoryPointData((math.nan,), 1.0)]),
        ([ARM_JOINTS[0]], [TrajectoryPointData((0.0,), math.inf)]),
        (
            [ARM_JOINTS[0]],
            [TrajectoryPointData((0.0,), 1.0), TrajectoryPointData((0.1,), 1.0)],
        ),
        ([ARM_JOINTS[0]], [TrajectoryPointData((0.0,), 1.0, velocities=(0.0, 0.0))]),
        ([ARM_JOINTS[0]], [TrajectoryPointData((0.0,), 1.0, velocities=(2.0,))]),
    ],
)
def test_rejects_malformed_trajectories(names, points) -> None:
    with pytest.raises(TrajectoryError):
        parse_trajectory(names, points, [0.0] * 14)


def test_rejects_position_limits_and_excessive_speed() -> None:
    with pytest.raises(TrajectoryError, match="outside"):
        parse_trajectory(
            ["left_elbow_joint"], [TrajectoryPointData((2.5,), 2.0)], [0.0] * 14
        )
    with pytest.raises(TrajectoryError, match="speed"):
        parse_trajectory(
            ["left_elbow_joint"], [TrajectoryPointData((1.0,), 0.1)], [0.0] * 14
        )


def test_normalizes_small_zero_time_start_drift_and_rejects_jumps() -> None:
    trajectory = parse_trajectory(
        ["left_elbow_joint"],
        [TrajectoryPointData((0.04,), 0.0), TrajectoryPointData((0.4,), 1.0)],
        [0.0] * 14,
    )
    assert trajectory.points[0].positions == (0.0,)

    with pytest.raises(TrajectoryError, match="instantaneous"):
        parse_trajectory(
            ["left_elbow_joint"],
            [TrajectoryPointData((0.06,), 0.0)],
            [0.0] * 14,
        )


def test_stale_inputs_abort_and_suppress_held_output() -> None:
    core, _ = acquired_core()
    trajectory = parse_trajectory(
        ["left_elbow_joint"], [TrajectoryPointData((0.5,), 1.0)], core.arm_hold
    )
    core.start_trajectory(trajectory, 1.1)
    assert core.output(1.7) is None
    assert core.outcome[0] == "stale"
    assert core.trajectory is None


def test_trajectory_start_is_seeded_from_latest_measured_arm() -> None:
    core, measured = acquired_core()
    elbow_index = ARM_JOINTS.index("left_elbow_joint")
    measured[15 + elbow_index] = 0.2
    core.update_measured(measured, 1.05)
    trajectory = parse_trajectory(
        ["left_elbow_joint"], [TrajectoryPointData((0.4,), 1.0)], measured[15:]
    )
    core.start_trajectory(trajectory, 1.1)
    assert core.trajectory_initial[elbow_index] == 0.2


def test_cancellation_holds_fresh_measured_positions() -> None:
    core, measured = acquired_core()
    trajectory = parse_trajectory(
        ["left_elbow_joint"], [TrajectoryPointData((0.5,), 1.0)], core.arm_hold
    )
    core.start_trajectory(trajectory, 1.1)
    updated = list(measured)
    updated[15:] = [0.25] * 14
    core.update_measured(updated, 1.2)
    core.cancel(1.2)
    assert core.arm_hold == [0.25] * 14
    assert core.trajectory is None
    assert core.outcome[0] == "canceled"


def test_completion_requires_measured_goal_tolerance() -> None:
    core, measured = acquired_core()
    elbow_index = ARM_JOINTS.index("left_elbow_joint")
    trajectory = parse_trajectory(
        ["left_elbow_joint"], [TrajectoryPointData((0.5,), 1.0)], core.arm_hold
    )
    core.start_trajectory(trajectory, 1.1)
    core.update_locomotion(body(), 2.1)
    measured[15 + elbow_index] = 0.5
    core.update_measured(measured, 2.1)
    assert core.output(2.1) is not None
    assert core.outcome[0] == "succeeded"
