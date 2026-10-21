import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dougie_sim.pose_graph import PoseGraph
from dougie_sim.map_corrections import MapCorrection, ObservationJournal
from dougie_sim.radiation import RadiationSample


def drifted_loop():
    graph = PoseGraph()
    truth = [np.array(p, dtype=float) for p in ((0, 0, 0), (1, 0, .2), (1, 1, .4), (0, 1, .2), (0, 0, 0))]
    for i, point in enumerate(truth[:-1]):
        graph.add_pose(float(i), point+[.025*i, -.01*i, .005*i], Rotation.from_rotvec([.003*i, -.002*i, .005*i]).as_matrix())
    return graph, truth


def test_joint_pose_graph_optimization_closes_3d_loop_without_moving_gauge():
    graph, truth = drifted_loop()
    initial = [(p.position.copy(), p.rotation.copy()) for p in graph.poses]
    current_p, current_R = truth[-1]+[.1, -.04, .02], Rotation.from_rotvec([.012, -.008, .02]).as_matrix()
    event = graph.close_loop(0., 4., current_p, current_R, truth[-1], np.eye(3), 1)
    assert event is not None, graph.last_result
    assert len(event.knots) == 5
    assert graph.last_result['final_cost'] < graph.last_result['initial_cost']
    marginal = np.asarray(graph.last_result['position_marginal_covariance'])
    assert np.linalg.eigvalsh(marginal).min() > 0.
    assert np.max(np.sqrt(np.diag(marginal))) < .1
    assert np.linalg.norm(graph.poses[-1].position) < np.linalg.norm(current_p)/2
    np.testing.assert_array_equal(graph.poses[0].position, initial[0][0])
    np.testing.assert_array_equal(graph.poses[0].rotation, initial[0][1])
    assert any(np.linalg.norm(graph.poses[i].position-initial[i][0]) > .001 for i in (1, 2, 3))
    for i, (p, R) in enumerate([*initial, (current_p, current_R)]):
        delta_R, delta_t = event.transform(float(i))
        np.testing.assert_allclose(delta_R@p+delta_t, graph.poses[i].position, atol=1e-9)
        np.testing.assert_allclose(delta_R@R, graph.poses[i].rotation, atol=1e-9)


def test_outlier_loop_is_transactionally_rejected():
    graph, truth = drifted_loop()
    before = graph.archive()
    event = graph.close_loop(0., 4., truth[-1]+[.1, 0., 0.], np.eye(3), [4., 4., 4.], np.eye(3), 1)
    assert event is None and graph.rejected_loops == 1
    assert graph.archive()['poses'] == before['poses']
    assert graph.archive()['factors'] == before['factors']


def test_graph_correction_json_replay_uses_optimized_knots_not_endpoint_warp():
    from dataclasses import asdict
    graph, truth = drifted_loop()
    event = graph.close_loop(0., 4., [.1, -.04, .02], np.eye(3), truth[-1], np.eye(3), 1)
    assert event is not None
    decoded = MapCorrection(**json.loads(json.dumps(asdict(event))))
    for stamp in (0., .5, 1., 2.5, 4., 5.):
        np.testing.assert_allclose(decoded.point([1., .5, .4], stamp), event.point([1., .5, .4], stamp), atol=1e-12)
    sample = RadiationSample(2., (1., .5, .4), 50, .5, 7.2, 1., .01, .02,
        window_positions=((1., .5, .4), (1.1, .5, .4)), window_durations_s=(.25, .25))
    corrected = decoded.sample(sample)
    assert corrected.counts == sample.counts and corrected.dose_usv == sample.dose_usv
    np.testing.assert_allclose(corrected.position, np.mean(corrected.window_positions, axis=0))
    assert corrected.map_revision == 1


def test_graph_capacity_is_explicit_without_evicting_old_constraints():
    graph = PoseGraph(capacity=2)
    graph.add_pose(0., np.zeros(3), np.eye(3))
    graph.add_pose(1., np.ones(3)*.1, np.eye(3))
    assert graph.add_pose(2., np.ones(3)*.2, np.eye(3)) is None
    assert graph.saturated and len(graph.poses) == 2 and len(graph.factors) == 1


def test_bad_correction_knots_cannot_move_the_initial_frame():
    with pytest.raises(ValueError, match='knots'):
        MapCorrection(1, 0., 1., (0., 0., 0.), (.1, 0., 0.),
                      ((0., 0., 0., 0., .01, 0., 0.), (1., 0., 0., 0., .1, 0., 0.)))


def test_confirmed_onboard_loop_updates_graph_keyframes_and_current_pose_together():
    from dougie_sim.odometry import FusedOdometry
    from dougie_sim.registration import Keyframe
    a, b = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-1., 1., 21))
    points = np.concatenate((np.c_[np.zeros(a.size), a.ravel(), b.ravel()],
        np.c_[a.ravel(), np.zeros(a.size), b.ravel()], np.c_[a.ravel(), b.ravel(), np.zeros(a.size)]))
    normals = np.repeat(np.eye(3), a.size, axis=0)
    odometry = FusedOdometry(slam_backend='pose-graph')
    for i in range(6):
        p = np.array([.08*i/5., 0., 0.])
        odometry.keyframes.append(Keyframe(i*5., p, np.eye(3), points, normals, .001))
        odometry.pose_graph.add_pose(i*5., p, np.eye(3))
    odometry.position = np.array([.08, 0., 0.])
    odometry._try_loop(points, normals, 60.)
    odometry._try_loop(points, normals, 62.)
    assert odometry.map_revision == 1 and odometry.corrections[0].knots
    assert 0. < odometry.position[0] < .06
    for frame, pose in zip(odometry.keyframes, odometry.pose_graph.poses):
        np.testing.assert_allclose(frame.position, pose.position, atol=1e-9)
        np.testing.assert_allclose(frame.rotation, pose.rotation, atol=1e-9)
