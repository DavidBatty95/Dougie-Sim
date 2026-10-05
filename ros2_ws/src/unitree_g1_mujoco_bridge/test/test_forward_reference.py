import math

from unitree_g1_mujoco_bridge.forward_reference import forward_reference


def test_forward_goal_uses_full_speed():
    assert forward_reference(3, 0, .65) == (.65, 0., 0.)


def test_side_and_rear_goals_turn_before_translation():
    for x, y in ((0, 3), (-3, 0), (-3, -2)):
        vx, vy, yaw = forward_reference(x, y, .65)
        assert vx == vy == 0
        assert 0 < abs(yaw) <= .35
        assert math.copysign(1, yaw) == math.copysign(1, math.atan2(y, x))


def test_forward_arc_and_bad_input():
    vx, vy, yaw = forward_reference(3, 1, .65)
    assert 0 < vx < .65 and vy == 0 and yaw > 0
    assert forward_reference(float('nan'), 1, .65) == (0., 0., 0.)


def test_directional_compensation_respects_calibrated_limits_and_stops_aligned():
    assert forward_reference(0, -2, .6, .6, .9)[2] == -.9
    assert forward_reference(0, 2, .6, .6, .9)[2] == .6
    assert forward_reference(2, .1, .6, .6, .9)[2] == 0.
    assert forward_reference(2, -.1, .6, .6, .9)[2] == 0.
    vx, vy, yaw = forward_reference(3, -1, .6, .6, .9)
    assert vx > 0 and vy == 0 and -.35 <= yaw < 0
