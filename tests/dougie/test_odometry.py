from dataclasses import replace

import numpy as np
import pytest

from dougie_sim.odometry import FusedOdometry, Proprioception, leg_kinematics
from dougie_sim.onboard_sensors import OnboardSensors
from dougie_sim.simulation import Simulation, rotation


def test_encoder_forward_kinematics_matches_robot_contract():
    sim = Simulation()
    feet, velocity = leg_kinematics(sim.data.qpos[sim.qadr], sim.data.qvel[sim.vadr])
    measured = np.array([sim.data.body(name+'_foot').xpos for name in ('FR', 'FL', 'RR', 'RL')])
    np.testing.assert_allclose(feet@rotation(sim.data.qpos[3:7]).T+sim.data.qpos[:3], measured, atol=1e-9)
    np.testing.assert_array_equal(velocity, np.zeros((4, 3)))


def test_body_scan_has_no_world_pose_and_no_true_support_labels():
    sim = Simulation('room')
    sensors = OnboardSensors(4)
    packet = sensors.scan(sim)
    np.testing.assert_array_equal(packet.origin, [0., 0., .12])
    assert not packet.support_mask.any()
    sim.data.qpos[:2] = [1., 1.]
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    moved = sensors.scan(sim)
    np.testing.assert_array_equal(packet.directions, moved.directions)
    np.testing.assert_array_equal(packet.origin, moved.origin)
    assert not np.array_equal(packet.distances, moved.distances)


def test_onboard_policy_observation_does_not_read_true_base_pose_or_velocity():
    sim = Simulation()
    measured = OnboardSensors().proprioception(sim)
    sim.onboard_proprioception = measured, np.eye(3)
    before = sim.observation([.1, 0., 0.])
    sim.data.qpos[:7] = [20., -10., 3., .70710678, .70710678, 0., 0.]
    sim.data.qvel[:6] = 12.
    np.testing.assert_array_equal(before, sim.observation([.1, 0., 0.]))


def test_scan_projection_is_an_immutable_capture_time_snapshot():
    sim = Simulation('room')
    estimator = FusedOdometry()
    local = OnboardSensors().scan(sim)
    packet = estimator.world_packet(local)
    points = packet.points.copy()
    estimator.position[:] = 10.
    estimator.rotation[:] = -np.eye(3)
    np.testing.assert_array_equal(packet.points, points)
    estimator.stamp = local.stamp_s+.1
    with pytest.raises(ValueError, match='capture timestamp'):
        estimator.world_packet(local)


def test_sensor_filter_tracks_physical_walking_without_world_pose_input():
    sim = Simulation('room')
    sensors, odometry = OnboardSensors(1), FusedOdometry()
    errors = []
    for i in range(1600):
        observation = sensors.proprioception(sim)
        odometry.update(observation)
        if i % 40 == 0:
            odometry.observe_scan(sensors.scan(sim))
        sim.onboard_proprioception = observation, odometry.rotation.copy()
        sim.step((.2, 0., .08))
        if i > 200:
            errors.append(np.linalg.norm(odometry.position-sim.data.qpos[:3]))
    assert odometry.initialized and odometry.scan_updates > 10
    assert np.percentile(errors, 95) < .15
    assert np.linalg.eigvalsh(odometry.covariance).min() > 0
    assert np.linalg.norm(odometry.position[:2]) > .6
    with pytest.raises(ValueError, match='out-of-order'):
        odometry.update(replace(observation, stamp_s=0.))
