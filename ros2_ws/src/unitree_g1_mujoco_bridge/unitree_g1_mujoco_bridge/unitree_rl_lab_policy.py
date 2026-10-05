from collections import deque
from typing import Sequence

import numpy as np


UNITREE_RL_LAB_G1_OBS_DIM = 480
UNITREE_RL_LAB_G1_ACTION_DIM = 29
UNITREE_RL_LAB_G1_HISTORY_LENGTH = 5
UNITREE_RL_LAB_G1_ACTION_SCALE = 0.25

# Policy-order joint index -> Unitree SDK motor index, from the official v0 deploy.yaml.
UNITREE_RL_LAB_G1_JOINT_IDS_MAP = (
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11,
    17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
)
UNITREE_RL_LAB_G1_DEFAULT_POS = (
    -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3,
    0.3, -0.2, -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97,
    0.15, -0.15, 0.0, 0.0, 0.0, 0.0,
)


def _quat_conjugate(quat: Sequence[float]) -> tuple[float, float, float, float]:
    return quat[0], -quat[1], -quat[2], -quat[3]


def _quat_rotate(quat: Sequence[float], vec: Sequence[float]) -> list[float]:
    w, x, y, z = quat
    vx, vy, vz = vec
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    ]


class UnitreeRLLabG1PolicyAdapter:
    """Reproduce the official Unitree RL Lab G1-29DoF v0 deployment contract."""

    _TERM_NAMES = (
        "base_ang_vel",
        "projected_gravity",
        "velocity_commands",
        "joint_pos_rel",
        "joint_vel_rel",
        "last_action",
    )

    def __init__(self) -> None:
        self._history = {
            name: deque(maxlen=UNITREE_RL_LAB_G1_HISTORY_LENGTH)
            for name in self._TERM_NAMES
        }

    @property
    def sdk_default_positions(self) -> list[float]:
        return self.policy_to_sdk(UNITREE_RL_LAB_G1_DEFAULT_POS)

    def reset(self) -> None:
        for history in self._history.values():
            history.clear()

    @staticmethod
    def sdk_to_policy(values: Sequence[float]) -> list[float]:
        if len(values) != UNITREE_RL_LAB_G1_ACTION_DIM:
            raise ValueError(f"Expected 29 SDK-order values, got {len(values)}")
        return [float(values[index]) for index in UNITREE_RL_LAB_G1_JOINT_IDS_MAP]

    @staticmethod
    def policy_to_sdk(values: Sequence[float]) -> list[float]:
        if len(values) != UNITREE_RL_LAB_G1_ACTION_DIM:
            raise ValueError(f"Expected 29 policy-order values, got {len(values)}")
        result = [0.0] * UNITREE_RL_LAB_G1_ACTION_DIM
        for policy_index, sdk_index in enumerate(UNITREE_RL_LAB_G1_JOINT_IDS_MAP):
            result[sdk_index] = float(values[policy_index])
        return result

    def build_observation(
        self,
        command: Sequence[float],
        sdk_joint_pos: Sequence[float],
        sdk_joint_vel: Sequence[float],
        gyroscope: Sequence[float],
        quaternion_wxyz: Sequence[float],
        last_action: Sequence[float],
        external_arms: bool = False,
    ) -> np.ndarray:
        if len(command) != 3 or len(gyroscope) != 3 or len(quaternion_wxyz) != 4:
            raise ValueError("Expected 3D command/gyro and a wxyz quaternion")

        quat = np.asarray(quaternion_wxyz, dtype=np.float64)
        norm = float(np.linalg.norm(quat))
        if norm > 1e-6:
            quat /= norm
        else:
            quat = np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64)

        policy_pos = self.sdk_to_policy(sdk_joint_pos)
        policy_vel = self.sdk_to_policy(sdk_joint_vel)
        previous_action = list(last_action)
        if external_arms:
            for index,sdk_index in enumerate(UNITREE_RL_LAB_G1_JOINT_IDS_MAP):
                if sdk_index >= 15:
                    policy_pos[index] = UNITREE_RL_LAB_G1_DEFAULT_POS[index]
                    policy_vel[index] = 0.
                    previous_action[index] = 0.
        terms = {
            "base_ang_vel": [0.2 * float(value) for value in gyroscope],
            "projected_gravity": _quat_rotate(
                _quat_conjugate(quat), (0.0, 0.0, -1.0)
            ),
            "velocity_commands": [float(value) for value in command],
            "joint_pos_rel": [
                value - default
                for value, default in zip(policy_pos, UNITREE_RL_LAB_G1_DEFAULT_POS)
            ],
            "joint_vel_rel": [0.05 * value for value in policy_vel],
            "last_action": [float(value) for value in previous_action],
        }

        for name in self._TERM_NAMES:
            history = self._history[name]
            if not history:
                history.extend([list(terms[name])] * UNITREE_RL_LAB_G1_HISTORY_LENGTH)
            else:
                history.append(list(terms[name]))

        values = [
            value
            for name in self._TERM_NAMES
            for sample in self._history[name]
            for value in sample
        ]
        if len(values) != UNITREE_RL_LAB_G1_OBS_DIM:
            raise RuntimeError(f"Built {len(values)} Unitree RL Lab observations")
        return np.asarray([values], dtype=np.float32)

    def action_targets(self, action: Sequence[float]) -> list[float]:
        if len(action) != UNITREE_RL_LAB_G1_ACTION_DIM:
            raise ValueError(f"Expected 29 policy actions, got {len(action)}")
        policy_targets = [
            default + UNITREE_RL_LAB_G1_ACTION_SCALE * float(value)
            for default, value in zip(UNITREE_RL_LAB_G1_DEFAULT_POS, action)
        ]
        return self.policy_to_sdk(policy_targets)
