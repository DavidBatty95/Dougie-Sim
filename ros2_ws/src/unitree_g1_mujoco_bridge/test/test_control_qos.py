from rclpy.qos import QoSReliabilityPolicy

from unitree_g1_mujoco_bridge.g1_mujoco_bridge import (
    LATEST_RELIABLE_QOS as BRIDGE_QOS,
)
from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import (
    LATEST_RELIABLE_QOS as POLICY_QOS,
)


def test_control_topics_keep_only_latest_reliable_sample() -> None:
    for profile in (BRIDGE_QOS, POLICY_QOS):
        assert profile.depth == 1
        assert profile.reliability == QoSReliabilityPolicy.RELIABLE
