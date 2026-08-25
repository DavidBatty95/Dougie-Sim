import numpy as np
import pytest

from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import (
    UNITREE_RL_LAB_G1_ACTION_SCALE,
    UNITREE_RL_LAB_G1_DEFAULT_POS,
    UNITREE_RL_LAB_G1_JOINT_IDS_MAP,
    UNITREE_RL_LAB_G1_OBS_DIM,
    UnitreeRLLabG1PolicyAdapter,
)


def test_joint_permutation_round_trip_and_default_pose():
    adapter = UnitreeRLLabG1PolicyAdapter()
    sdk_values = list(range(29))

    policy_values = adapter.sdk_to_policy(sdk_values)

    assert policy_values == [sdk_values[index] for index in UNITREE_RL_LAB_G1_JOINT_IDS_MAP]
    assert adapter.policy_to_sdk(policy_values) == sdk_values
    assert adapter.sdk_to_policy(adapter.sdk_default_positions) == pytest.approx(
        UNITREE_RL_LAB_G1_DEFAULT_POS
    )


def test_initial_observation_matches_official_term_grouped_history():
    adapter = UnitreeRLLabG1PolicyAdapter()
    sdk_default = adapter.sdk_default_positions

    observation = adapter.build_observation(
        command=(0.1, -0.2, 0.05),
        sdk_joint_pos=sdk_default,
        sdk_joint_vel=[2.0] * 29,
        gyroscope=(1.0, 2.0, 3.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        last_action=[0.5] * 29,
    ).reshape(-1)

    assert observation.shape == (UNITREE_RL_LAB_G1_OBS_DIM,)
    assert observation[0:15] == pytest.approx([0.2, 0.4, 0.6] * 5)
    assert observation[15:30] == pytest.approx([0.0, 0.0, -1.0] * 5)
    assert observation[30:45] == pytest.approx([0.1, -0.2, 0.05] * 5)
    assert observation[45:190] == pytest.approx([0.0] * 145)
    assert observation[190:335] == pytest.approx([0.1] * 145)
    assert observation[335:480] == pytest.approx([0.5] * 145)


def test_history_is_oldest_to_newest_within_each_term_and_reset_refills():
    adapter = UnitreeRLLabG1PolicyAdapter()
    sdk_default = adapter.sdk_default_positions
    zeros = [0.0] * 29

    for command_x in range(1, 7):
        observation = adapter.build_observation(
            (command_x, 0.0, 0.0), sdk_default, zeros, (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0), zeros
        ).reshape(-1)

    assert observation[30:45:3] == pytest.approx([2.0, 3.0, 4.0, 5.0, 6.0])

    adapter.reset()
    reset_observation = adapter.build_observation(
        (9.0, 0.0, 0.0), sdk_default, zeros, (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0), zeros
    ).reshape(-1)
    assert reset_observation[30:45:3] == pytest.approx([9.0] * 5)


def test_actions_use_official_scale_offset_and_sdk_permutation():
    adapter = UnitreeRLLabG1PolicyAdapter()
    action = np.arange(29, dtype=np.float32)

    sdk_targets = adapter.action_targets(action)
    policy_targets = adapter.sdk_to_policy(sdk_targets)

    expected = np.asarray(UNITREE_RL_LAB_G1_DEFAULT_POS) + (
        UNITREE_RL_LAB_G1_ACTION_SCALE * action
    )
    assert policy_targets == pytest.approx(expected)
