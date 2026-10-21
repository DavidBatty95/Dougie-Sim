from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dougie_sim.map_corrections import MapCorrection, ObservationJournal
from dougie_sim.odometry import FusedOdometry
from dougie_sim.registration import Keyframe, register
from dougie_sim.radiation import Detector, RadiationSample
from dougie_sim.sensing import RangePacket, support_mask


def corner_cloud():
    a, b = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-1., 1., 21))
    points = np.concatenate((np.c_[np.zeros(a.size), a.ravel(), b.ravel()],
        np.c_[a.ravel(), np.zeros(a.size), b.ravel()], np.c_[a.ravel(), b.ravel(), np.zeros(a.size)]))
    normals = np.repeat(np.eye(3), a.size, axis=0)
    return points, normals


def floor_packet(stamp=.5):
    x, y = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-1., 1., 21))
    points, origin = np.stack((x, y, np.zeros_like(x)), axis=-1), np.array([0., 0., .4])
    delta = points-origin
    distance = np.linalg.norm(delta, axis=2)
    directions = delta/distance[..., None]
    return RangePacket(stamp, 0, origin, directions, distance, 6., support_mask(origin, directions, distance, 6.))


def test_registration_recovers_transform_and_rejection_is_atomic():
    points, normals = corner_cloud()
    R, t = Rotation.from_rotvec([.01, -.01, .02]).as_matrix(), np.array([.04, -.03, .02])
    result = register(points, normals, points@R.T+t, normals@R.T, np.zeros(3), np.eye(3))
    assert result.accepted and result.observable_rank == 6
    np.testing.assert_allclose(result.position, t, atol=.005)
    np.testing.assert_allclose(result.rotation, R, atol=.005)
    rejected = register(points, normals, points+10., normals, np.zeros(3), np.eye(3))
    assert not rejected.accepted
    np.testing.assert_array_equal(rejected.position, np.zeros(3))
    np.testing.assert_array_equal(rejected.rotation, np.eye(3))


def test_flat_scene_does_not_invent_observable_horizontal_motion():
    points, normals = corner_cloud()
    points, normals = points[-441:], normals[-441:]
    result = register(points, normals, points, normals, np.array([.1, .1, .04]), np.eye(3))
    assert result.accepted and result.reason == 'weak_geometry' and result.observable_rank == 3
    np.testing.assert_allclose(result.position, [.1, .1, 0.], atol=1e-6)


def test_rejected_scan_cannot_poison_submaps_and_loop_requires_confirmation():
    points, normals = corner_cloud()
    odometry = FusedOdometry()
    odometry.initialized = True
    odometry._observe_submaps(points, normals, 0.)
    count = len(odometry.keyframes)
    odometry._observe_submaps(points+10., normals, .2)
    assert len(odometry.keyframes) == count and odometry.rejection_streak == 1
    odometry.keyframes = [Keyframe(i*5., np.array([.08*i/5., 0., 0.]), np.eye(3), points, normals, .001) for i in range(6)]
    odometry.position = np.array([.08, 0., 0.])
    odometry._try_loop(points, normals, 60.)
    assert odometry.map_revision == 0
    odometry._try_loop(points, normals, 62.)
    assert odometry.map_revision == 1
    np.testing.assert_allclose(odometry.position, np.zeros(3), atol=.005)
    np.testing.assert_allclose(odometry.keyframes[0].position, np.zeros(3), atol=1e-9)


def test_count_window_and_scan_share_the_same_revision_without_duplicate_counts():
    event = MapCorrection(1, 0., 1., (0., 0., .1), (.2, 0., 0.))
    detector = Detector(3)
    for i in range(100):
        sample = detector.integrate([0., 0., .4], 10., .005, (i+1)*.005)
    packet = floor_packet()
    journal = ObservationJournal()
    journal.append([sample], packet, [0., 0., 0.], .5)
    assert journal.correct([event])
    assert not journal.correct([event])
    corrected = journal.samples[0]
    assert corrected.counts == sample.counts and corrected.dose_usv == sample.dose_usv
    assert corrected.window_durations_s == sample.window_durations_s
    assert corrected.map_revision == journal.scans[0].map_revision == 1
    R, t = event.transform(.5)
    np.testing.assert_allclose(journal.scans[0].points, packet.points@R.T+t, atol=1e-12)
    np.testing.assert_allclose(corrected.position, np.average(corrected.window_positions, weights=corrected.window_durations_s, axis=0))
    assert len(journal.samples) == 1
    with pytest.raises(ValueError, match='unknown map revision'):
        journal.align_packet(replace(packet, map_revision=2))
    with pytest.raises(ValueError, match='unknown map revision'):
        journal.align_sample(replace(sample, map_revision=2))


def test_partial_detector_window_correction_preserves_physical_counts():
    a, b = Detector(3), Detector(3)
    for i in range(3):
        a.integrate([0., 0., 0.], 10., .1, (i+1)*.1)
        b.integrate([0., 0., 0.], 10., .1, (i+1)*.1)
    a.correct_map(MapCorrection(1, 0., .3, (0., 0., 0.), (.3, 0., 0.)))
    for i in range(3, 5):
        actual = a.integrate([.3, 0., 0.], 10., .1, (i+1)*.1)
        control = b.integrate([0., 0., 0.], 10., .1, (i+1)*.1)
    assert actual.counts == control.counts and actual.integration_s == control.integration_s
    assert actual.position[0] == pytest.approx(.21)
    assert actual.map_revision == 1


def test_worker_rebuild_replays_each_count_once_and_corrects_home():
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'conservative',
                      options=dict(consistent_mapping=True, navigation_profile='robust', path_mode='grid', display_prediction=False))
    sample = RadiationSample(.5, (0., 0., .4), 10, .5, 1.44, .1, .001, .002)
    worker.plan([sample], floor_packet(), np.zeros(3), .5, False)
    event = MapCorrection(1, 0., 1., (0., 0., 0.), (.2, 0., 0.))
    packet = event.packet(floor_packet(1.5))
    _, _, metrics, _ = worker.plan([replace(sample, stamp_s=1.5, position=(.2, 0., .4), map_revision=1)],
        packet, np.array([.2, 0., 0.]), 1.5, False, corrections=(event,))
    result = worker.finalize([])
    assert result.gp.updates == 2 and metrics['map_revision'] == 1
    assert result.calibration.windows == 2  # Replay must not rescore held-in counts.
    np.testing.assert_allclose(result.home, [.1, 0., 0.])
    assert metrics['map_consistency']['retained_count_windows'] == 2


def test_same_graph_cell_still_executes_checked_final_home_approach():
    from types import SimpleNamespace
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    planner = ClusteredDoseExplorer()
    packet = floor_packet(1.)
    planner.terrain.update(packet.points, packet.supports, packet.origin)
    planner.home, planner.home_tolerance, planner.returning = np.array([.01, .01, 0.]), .2, True
    planner.sample = RadiationSample(1., (.24, .24, .4), 10, .5, 1.44, .1, .001, .002)
    planner.gp = SimpleNamespace(predict=lambda p: (np.ones(len(p)), np.full(len(p), .1), np.ones(len(p))))
    planner.replan(np.array([.24, .24, 0.]), 1.)
    assert planner.state == 'returning' and planner.path
    np.testing.assert_allclose(planner.path[-1], planner.home)


def test_sparse_elevation_wall_normals_are_not_discarded_by_fixed_distance_cutoff():
    from dougie_sim.scan_geometry import registration_cloud
    a, e = np.meshgrid(np.linspace(-.4, .4, 41), np.array([-.15, 0., .15, .35, .6]))
    directions = np.stack((np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)), axis=-1)
    distances = 3./directions[:, :, 0]
    packet = RangePacket(1., 0, np.zeros(3), directions, distances, 6., np.zeros(a.shape, bool))
    points, normals = registration_cloud(packet)
    assert len(points) >= 100
    np.testing.assert_allclose(points[:, 0], 3., atol=1e-9)
    assert np.all(np.abs(normals[:, 0]) > .99)


def test_registration_keeps_small_wall_constraints_beside_a_large_floor():
    points, normals = corner_cloud()
    # A large floor's rotational lever arm used to dominate the unscaled rank
    # cutoff, losing valid horizontal wall constraints.
    floor = points[-441:]*5.
    points = np.concatenate((points[:882:12], floor))
    normals = np.concatenate((normals[:882:12], normals[-441:]))
    result = register(points, normals, points, normals, np.array([.04, .03, .02]), np.eye(3))
    assert result.accepted and result.observable_rank == 6
    np.testing.assert_allclose(result.position, np.zeros(3), atol=.005)


def test_pose_allowance_requests_return_before_motion_stop_threshold():
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'conservative',
                      options=dict(consistent_mapping=True, display_prediction=False))
    sample = RadiationSample(.5, (0., 0., .4), 5, .5, .72, .1, .001, .002)
    worker.plan([sample], floor_packet(), np.zeros(3), .5, False)
    _, state, metrics, _ = worker.plan([replace(sample, stamp_s=1.5)], floor_packet(1.5), np.array([.5, 0., 0.]),
        1.5, False, localization=dict(initialized=True, position_std_m=[.19]*3, conditional_position_std_m=[.003]*3))
    assert state.startswith('return')
    assert metrics['decision_reason'] == 'localization_return_reserve'


def test_confirmed_small_loop_revalidates_uncertainty_without_moving_maps():
    points, normals = corner_cloud()
    odometry = FusedOdometry()
    odometry.keyframes = [Keyframe(i*5., np.array([.008*i/5., 0., 0.]), np.eye(3), points, normals, .001) for i in range(6)]
    odometry.position, odometry.drift_variance = np.array([.008, 0., 0.]), .04
    before = odometry.position.copy()
    odometry._try_loop(points, normals, 60.)
    assert odometry.drift_variance == .04
    odometry._try_loop(points, normals, 62.)
    assert odometry.anchor_revalidations == 1 and odometry.map_revision == 0
    assert odometry.drift_variance < .002
    np.testing.assert_array_equal(odometry.position, before)


def test_planned_clearance_reserves_the_followers_tracking_error():
    from dougie_sim.planning_worker import create_explorer
    from dougie_sim.path_motion import TRACKING_ERROR_LIMIT
    explorer = create_explorer((2., 100., 32, 600., 'poisson'), 'clustered', 'conservative',
                               dict(consistent_mapping=True, path_mode='smooth'))
    assert explorer.terrain.base_radius == pytest.approx(.4+TRACKING_ERROR_LIMIT)
    explorer.terrain.set_localization_margin(.02)
    assert explorer.terrain.radius == pytest.approx(.4+TRACKING_ERROR_LIMIT+.02)
