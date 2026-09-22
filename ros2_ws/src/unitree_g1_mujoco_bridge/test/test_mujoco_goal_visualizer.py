import math
from types import SimpleNamespace as S

import pytest

from unitree_g1_mujoco_bridge.mujoco_goal_visualizer import (
    GOAL_MAGIC, GOAL_PACKET, MujocoGoalVisualizer, encode_goal, relative_goal,
)


def test_goal_packet_is_robot_relative_and_can_be_explicitly_cleared():
    packet = encode_goal(True, 2., -1., .55, stamp_ns=123)
    assert len(packet) == 28
    magic, active, stamp, x, y, radius = GOAL_PACKET.unpack(packet)
    assert (magic, active, stamp, x, y) == (GOAL_MAGIC, 1, 123, 2., -1.)
    assert radius == pytest.approx(.55)
    assert GOAL_PACKET.unpack(encode_goal(False))[1] == 0
    with pytest.raises(ValueError):
        encode_goal(True, math.nan, 0.)


def test_map_goal_uses_estimated_tf_to_robot_frame():
    transform = S(translation=S(x=-1., y=0.), rotation=S(x=0., y=0., z=-math.sqrt(.5), w=math.sqrt(.5)))
    assert relative_goal(S(x=2., y=1.), transform) == pytest.approx((0., -2.))


def test_expired_mission_clears_overlay_without_using_any_pose_source():
    sent = []
    node = S(active=True, heartbeat=-math.inf, goal=object(), _send=lambda active: sent.append(active))
    MujocoGoalVisualizer._tick(node)
    assert sent == [False]
