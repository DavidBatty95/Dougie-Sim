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


def test_opposite_observed_faces_cannot_collapse_a_thin_obstacle():
    points, normals = corner_cloud()
    face, normal = points[:441], normals[:441]
    result = register(face+[.05, 0., 0.], normal, face, -normal, np.zeros(3), np.eye(3))
    assert not result.accepted
    np.testing.assert_array_equal(result.position, np.zeros(3))


def test_opposite_face_does_not_invent_rank_in_otherwise_weak_geometry():
    target, normals = corner_cloud()
    source, target_normals = target.copy(), normals.copy()
    source[:441, 0] += .05
    target_normals[:441] *= -1.
    result = register(source, normals, target, target_normals, np.zeros(3), np.eye(3))
    assert result.accepted and result.observable_rank == 5
    np.testing.assert_allclose(result.position, np.zeros(3), atol=1e-6)


def test_registration_normals_face_the_observing_sensor():
    from dougie_sim.scan_geometry import registration_cloud
    for azimuth in (0., np.pi):
        a, e = np.meshgrid(np.linspace(-.4, .4, 41)+azimuth, np.array([-.15, 0., .15, .35, .6]))
        directions = np.stack((np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)), axis=-1)
        distances = 3./np.abs(directions[:, :, 0])
        packet = RangePacket(1., 0, np.zeros(3), directions, distances, 6., np.zeros(a.shape, bool))
        points, normals = registration_cloud(packet)
        assert len(points) >= 100
        assert np.all(np.sum(normals*(points-packet.origin), axis=1) <= 0.)


def test_normal_balanced_sampling_preserves_observed_directions_inside_voxels():
    from dougie_sim.scan_geometry import sample_registration_cloud
    x, y = np.meshgrid(np.arange(0., 3., .05), np.arange(0., 3., .05))
    floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    along = .05+np.arange(7)*.24
    wall_x = np.c_[np.full(7, .01), along, np.full(7, .05)]
    wall_y = np.c_[along, np.full(7, .01), np.full(7, .05)]
    points = np.vstack((floor, wall_x, wall_y))
    normals = np.vstack((np.tile([0., 0., 1.], (len(floor), 1)),
                         np.tile([-1., 0., 0.], (7, 1)), np.tile([0., -1., 0.], (7, 1))))
    spatial, spatial_n = sample_registration_cloud(points, normals, capacity=200)
    selected, selected_n = sample_registration_cloud(points, normals, 'normal-balanced', 200)
    assert len(selected) <= 200
    assert len(set(map(tuple, selected))) == len(selected)
    assert sum(selected_n[:, 0] == -1.) >= 4 and sum(selected_n[:, 1] == -1.) >= 4
    assert set(map(tuple, selected)) <= set(map(tuple, points))
    assert register(spatial, spatial_n, spatial, spatial_n, np.zeros(3), np.eye(3)).observable_rank == 3
    assert register(selected, selected_n, selected, selected_n, np.zeros(3), np.eye(3)).observable_rank == 6


def test_spatial_registration_sampling_retains_historical_order_and_budget():
    from dougie_sim.scan_geometry import sample_registration_cloud
    rng = np.random.default_rng(3)
    points = rng.uniform(-5., 5., (10000, 3))
    normals = np.tile([0., 0., 1.], (len(points), 1))
    _, indices = np.unique(np.floor(points/.12).astype(int), axis=0, return_index=True)
    indices = indices[::int(np.ceil(len(indices)/1500))]
    selected, selected_n = sample_registration_cloud(points, normals)
    np.testing.assert_array_equal(selected, points[indices])
    np.testing.assert_array_equal(selected_n, normals[indices])


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


def test_wider_anchor_eligibility_still_requires_confirmed_full_rank_registration():
    points, normals = corner_cloud()
    for radius, revisions in ((1.25, 0), (3., 1)):
        odometry = FusedOdometry(loop_search_radius=radius)
        odometry.keyframes = [Keyframe(i*5., np.zeros(3), np.eye(3), points, normals, .001) for i in range(5)]
        odometry.position = np.array([2.1, 0., 0.])
        odometry._try_loop(points-[2., 0., 0.], normals, 60.)
        assert odometry.map_revision == 0
        odometry._try_loop(points-[2., 0., 0.], normals, 62.)
        assert odometry.map_revision == revisions
        if revisions:
            np.testing.assert_allclose(odometry.position, [2., 0., 0.], atol=.005)
    weak = FusedOdometry(loop_search_radius=3.)
    floor, up = points[-441:], normals[-441:]
    weak.keyframes = [Keyframe(i*5., np.zeros(3), np.eye(3), floor, up, .001) for i in range(5)]
    weak.position = np.array([2.1, 0., 0.])
    weak._try_loop(floor-[2., 0., 0.], up, 60.)
    weak._try_loop(floor-[2., 0., 0.], up, 62.)
    assert weak.map_revision == 0
    assert weak.loop_search['last_reason'] == 'registration_quality_rejected'


def test_invalid_loop_search_radius_is_rejected():
    for radius in (0., 3.1, float('nan')):
        with pytest.raises(ValueError, match='loop search radius'):
            FusedOdometry(loop_search_radius=radius)


@pytest.mark.parametrize('capacity,used,weak,accepted', [(16, 16, False, False),
    (32, 16, False, True), (32, 16, True, False), (32, 32, False, False)])
def test_bounded_correction_capacity_preserves_confirmation_and_rank_gates(capacity, used, weak, accepted):
    points, normals = corner_cloud()
    if weak:
        points, normals = points[-441:], normals[-441:]
    odometry = FusedOdometry(loop_correction_capacity=capacity)
    odometry.corrections = [MapCorrection(i+1, 0., 1., (0., 0., 0.), (0., 0., 0.)) for i in range(used)]
    odometry.keyframes = [Keyframe(i*5., np.zeros(3), np.eye(3), points, normals, .001) for i in range(6)]
    odometry.position = np.array([.08, 0., 0.])
    odometry._try_loop(points, normals, 60.)
    assert odometry.map_revision == used
    odometry._try_loop(points, normals, 62.)
    assert odometry.map_revision == used+int(accepted)
    if used == capacity:
        assert odometry.loop_search['inhibited_reason'] == 'correction_capacity'
    elif weak:
        assert odometry.loop_search['last_reason'] == 'registration_quality_rejected'


def test_correction_history_capacity_is_discrete_and_does_not_expand_pose_graph():
    for capacity in (0, 8, 64, 16., True, None):
        with pytest.raises(ValueError, match='loop correction capacity'):
            FusedOdometry(loop_correction_capacity=capacity)
    assert FusedOdometry(slam_backend='pose-graph', loop_correction_capacity=32).pose_graph.capacity == 512


@pytest.mark.parametrize('retention,keeps_remote', [('recent', False), ('coverage', True)])
def test_cloud_retention_keeps_coverage_without_discarding_graph_constraints(retention, keeps_remote):
    odometry = FusedOdometry(slam_backend='pose-graph', keyframe_retention=retention)
    points, normals = corner_cloud()
    for i in range(65):
        position = np.array([10. if i == 8 else 0., 0., 0.])
        odometry.keyframes.append(Keyframe(float(i), position, np.eye(3), points, normals, .01))
        odometry.pose_graph.add_pose(float(i), position, np.eye(3))
    odometry.loop_candidate = (9., np.zeros(3), np.zeros(3), 70.)
    odometry._prune_keyframes()
    stamps = {frame.stamp for frame in odometry.keyframes}
    assert len(stamps) == 64 and (8. in stamps) == keeps_remote
    assert set(range(8)) | set(range(61, 65)) <= stamps
    assert 9. in stamps  # A candidate awaiting confirmation must survive coverage pruning.
    assert len(odometry.pose_graph.poses) == 65 and len(odometry.pose_graph.factors) == 64


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


@pytest.mark.parametrize('terrain_profile,fusion', [('conservative', 'maximum'), ('layered', 'bounded-mean')])
def test_worker_rebuild_replays_each_count_once_and_corrects_home(terrain_profile, fusion, tmp_path):
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', terrain_profile,
                      options=dict(consistent_mapping=True, navigation_profile='robust', path_mode='grid',
                                   display_prediction=False, support_fusion=fusion))
    sample = RadiationSample(.5, (0., 0., .4), 10, .5, 1.44, .1, .001, .002)
    worker.plan([sample], floor_packet(), np.zeros(3), .5, False)
    event = MapCorrection(1, 0., 1., (0., 0., 0.), (.2, 0., 0.))
    packet = event.packet(floor_packet(1.5))
    _, _, metrics, _ = worker.plan([replace(sample, stamp_s=1.5, position=(.2, 0., .4), map_revision=1)],
        packet, np.array([.2, 0., 0.]), 1.5, False, corrections=(event,))
    archive = tmp_path/'support.npz' if terrain_profile == 'layered' else None
    result = worker.finalize([], support_archive=archive)
    assert result.gp.updates == 2 and metrics['map_revision'] == 1
    assert result.calibration.windows == 2  # Replay must not rescore held-in counts.
    np.testing.assert_allclose(result.home, [.1, 0., 0.])
    assert metrics['map_consistency']['retained_count_windows'] == 2
    if fusion == 'bounded-mean':
        assert result.terrain.support_fusion == fusion
        assert len(result.terrain._support_moments) == len(result.terrain.support_returns) > 0
        with np.load(archive, allow_pickle=False) as saved:
            assert int(saved['map_revision']) == 1
            np.testing.assert_allclose(saved['points'][:len(floor_packet().points)], event.packet(floor_packet()).points)
        from scripts.replay_dougie_support_fusion import replay
        comparison = replay(archive)
        assert comparison['captures'] == 2
        assert comparison['results'][1]['matches_original_node_count']
        with np.load(archive, allow_pickle=False) as saved:
            incomplete = dict(saved)
        incomplete['journal_saturated'] = True
        np.savez_compressed(tmp_path/'incomplete.npz', **incomplete)
        with pytest.raises(ValueError, match='complete'):
            replay(tmp_path/'incomplete.npz')


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


def test_first_support_failure_capture_is_not_overwritten(tmp_path):
    from dougie_sim import planning_worker as worker
    path = tmp_path/'first_failure.npz'
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'layered',
                      options=dict(consistent_mapping=True, display_prediction=False, support_capture_path=str(path)))
    worker._explorer.home = np.zeros(3)
    sample = RadiationSample(.5, (0., 0., .4), 10, .5, 1.44, .1, .001, .002)
    worker.plan([sample], floor_packet(), np.array([10., 0., 0.]), .5, False)
    original = path.read_bytes()
    with np.load(path, allow_pickle=False) as capture:
        assert str(capture['capture_scope']) == 'first_support_failure'
        assert len(capture['stamps_s']) == 1
    worker.plan([replace(sample, stamp_s=1.5)], floor_packet(1.5), np.array([11., 0., 0.]), 1.5, False)
    assert path.read_bytes() == original


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


def test_rebuild_preserves_observed_height_bands_and_body_visit_neighbours():
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'layered',
                      options=dict(consistent_mapping=True, path_mode='grid', display_prediction=False))
    sample = RadiationSample(.5, (0., 0., .44), 5, .5, .72, .1, .001, .002)
    worker.plan([sample], floor_packet(), np.zeros(3), .5, False)
    upper = floor_packet(1.5)
    upper = replace(upper, origin=upper.origin+[3., 0., .4])
    worker.plan([replace(sample, stamp_s=1.5, position=(3., 0., .84))], upper, np.array([3., 0., .4]), 1.5, False)
    event = MapCorrection(1, 0., 3., (0., 0., 0.), (.1, 0., 0.))
    _, _, metrics, _ = worker.plan([replace(sample, stamp_s=3.5, position=(.1, 0., .44), map_revision=1)],
        event.packet(floor_packet(3.5)), np.array([.1, 0., 0.]), 3.5, False, corrections=(event,))
    assert .4 in metrics['sampled_support_height_bands_m']
    assert metrics['visited_cells'] >= 8
