import numpy as np
import pytest

from dougie_sim.odometry import leg_kinematics
from dougie_sim.stance_hold import GravityStanceHold, support_torques


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


def measurement(stamp, force=30.):
    from dougie_sim.odometry import Proprioception
    return Proprioception(stamp, np.zeros(3), [0., 0., 9.81],
                          np.tile([0., .8, -1.5], 4), np.zeros(12), np.full(4, force))


def test_stance_controller_blends_load_and_refuses_lost_contact():
    default, scale, entry = np.tile([0., .8, -1.5], 4), np.full(12, .25), np.full(12, .1)
    hold = GravityStanceHold(10., 15., default, scale, 20.)
    assert hold.update(measurement(10.4), np.eye(3), entry, 10.4) is None
    assert hold.update(measurement(10.5), np.eye(3), entry, 10.5) is None
    np.testing.assert_allclose(hold.update(measurement(10.71), np.eye(3), entry, 10.71), entry)
    action = hold.update(measurement(11.22), np.eye(3), entry, 11.22)
    torque, _ = support_torques(default, [0., 0., 1.], 15.)
    np.testing.assert_allclose(default+scale*action, default+torque/20.)
    assert hold.update(measurement(11.23, force=0.), np.eye(3), entry, 11.23) is None
    assert hold.metrics()['phase'] == 'refused'
    assert hold.update(measurement(11.5), np.eye(3), entry, 11.5) is None


def test_stance_entry_deadline_cannot_be_reset_by_waiting():
    hold = GravityStanceHold(0., 15., np.zeros(12), np.ones(12), 20.)
    assert hold.update(measurement(3.1), np.eye(3), np.zeros(12), 3.1) is None
    assert hold.metrics()['reason'] == 'stance_entry_deadline'


def test_simulation_terminal_stance_uses_onboard_measurements_and_retains_refusal():
    from dataclasses import replace
    from dougie_sim.onboard_sensors import OnboardSensors
    from dougie_sim.odometry import FusedOdometry
    from dougie_sim.simulation import Simulation
    sim, sensors, odometry = Simulation(), OnboardSensors(0), FusedOdometry()
    with pytest.raises(ValueError, match='onboard proprioception'):
        sim.request_terminal_stance()
    for step in range(1150):
        observed = sensors.proprioception(sim)
        odometry.update(observed)
        sim.onboard_proprioception = observed, odometry.rotation
        if step == 750:
            sim.request_terminal_stance()
        sim.step()
    metrics = sim.metrics()['terminal_stance']
    assert metrics['phase'] == 'holding' and 5. < metrics['robot_model_mass_kg'] < 30.
    assert metrics['maximum_support_torque_nm'] <= 12.
    assert sim.metrics()['upright'] > .9
    with pytest.raises(ValueError, match='zero velocity command'):
        sim.step((.1, 0., 0.))
    sim.onboard_proprioception = replace(observed, foot_force=np.zeros(4)), odometry.rotation
    sim.step()
    sim.request_terminal_stance()
    assert sim.metrics()['terminal_stance']['phase'] == 'refused'
