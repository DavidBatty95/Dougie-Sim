import numpy as np
import pytest

from dougie_sim.odometry import leg_kinematics
from dougie_sim.stance_hold import support_torques


def test_stance_balances_weight_and_virtual_work():
    joints = np.tile([0., .8, -1.5], 4)
    up = np.array([.15, 0., 1.])
    up /= np.linalg.norm(up)
    torque, loads = support_torques(joints, up, 15.)
    feet, _ = leg_kinematics(joints, np.zeros(12))
    forces = loads[:, None]*up
    np.testing.assert_allclose(forces.sum(axis=0), 15.*9.81*up, atol=1e-10)
    np.testing.assert_allclose(np.cross(feet, forces).sum(axis=0), np.zeros(3), atol=1e-10)
    velocity = np.linspace(-.2, .3, 12)
    _, foot_velocity = leg_kinematics(joints, velocity)
    assert torque@velocity == pytest.approx(-np.sum(foot_velocity*forces))
    assert np.min(loads) > 5. and np.max(np.abs(torque)) <= 12.


def test_stance_refuses_unsupported_or_invalid_inputs():
    joints = np.tile([0., .8, -1.5], 4)
    for up, mass in (([1., 0., 0.], 15.), ([0., 0., 0.], 15.), ([0., 0., 1.], -1.), ([0., np.nan, 1.], 15.)):
        with pytest.raises(ValueError):
            support_torques(joints, up, mass)
