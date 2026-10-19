import numpy as np
import pytest

from dougie_sim.geometry_gp import GeometryPoissonGP
from dougie_sim.gp import PoissonOnlineGP
from dougie_sim.radiation import Detector
from dougie_sim.sensing import RangePacket
from dougie_sim.spatial_gp import load_gp, SpatialGP


def wall_packet(stamp=0., x=0.):
    y, z = np.meshgrid(np.linspace(-2., 2., 41), np.linspace(0., 2., 21))
    points = np.stack((np.full_like(y, x), y, z), axis=-1)
    origin = np.array([-1., 0., .5])
    delta = points-origin
    distances = np.linalg.norm(delta, axis=2)
    return RangePacket(stamp, 0, origin, delta/distances[..., None], distances, 6., np.zeros_like(y, dtype=bool))


def test_detector_window_preserves_motion_duration_and_single_count_draw():
    detector = Detector(seed=3)
    for i in range(100):
        sample = detector.integrate([i*.01, 0., .4], 10., .005, (i+1)*.005)
    assert sample is not None and detector.samples == 1
    assert sum(sample.window_durations_s) == pytest.approx(sample.integration_s)
    assert len(sample.window_positions) == 8
    np.testing.assert_allclose(np.average(sample.window_positions, weights=sample.window_durations_s, axis=0), sample.position)
    assert sample.window_positions[0][0] < .1 and sample.window_positions[-1][0] > .9


def test_stationary_window_matches_original_scalar_poisson_update():
    a, b = PoissonOnlineGP(16), PoissonOnlineGP(16)
    a.update_counts([0., 0., .4], 30, .5)
    b.update_counts_window([[0., 0., .4]]*8, [.0625]*8, 30)
    np.testing.assert_allclose(a.predict([[0., 0., .4], [1., 0., .4]]), b.predict([[0., 0., .4], [1., 0., .4]]))
    assert b.updates == 1


def test_moving_window_updates_endpoint_rates_without_inventing_midpoint_sample():
    gp = PoissonOnlineGP(32, lengthscales=(.15, .15, .15))
    points = np.array([[-1., 0., .4], [1., 0., .4]])
    gp.update_counts_window(points, [.25, .25], 100, 3600.)
    mean, _, _ = gp.predict([*points, [0., 0., .4]])
    assert gp.updates == 1 and mean[0] > 20. and mean[1] > 20.
    assert mean[2] < 10.  # There was no observation at the arithmetic midpoint.
    assert np.linalg.eigvalsh(gp.cov).min() > -1e-8
    with pytest.raises(ValueError):
        gp.update_counts_window(points, [.25, -.25], 100)


def test_observed_wall_kernel_is_psd_and_retains_uncertainty_across_wall():
    gp = GeometryPoissonGP(32)
    gp.observe_geometry(wall_packet())
    assert len(gp.planes) == 1
    points = np.random.default_rng(3).normal(size=(50, 3))
    assert np.linalg.eigvalsh(gp.kernel(points, points)).min() > -1e-8
    same = gp.kernel([[-.15, 0., .5]], [[-.25, 0., .5]])[0, 0]
    across = gp.kernel([[-.15, 0., .5]], [[.15, 0., .5]])[0, 0]
    assert across < .2*same
    for _ in range(5):
        gp.update_counts([-.15, 0., .5], 100, .5)
    _, sigma, _ = gp.predict([[-.15, 0., .5], [.15, 0., .5]])
    assert sigma[1] > 3*sigma[0]


def test_new_geometry_replays_counts_once_and_archive_roundtrips(tmp_path):
    gp = GeometryPoissonGP(32)
    gp.update_counts_window([[-.3, 0., .5], [-.2, 0., .5]], [.25, .25], 20)
    gp.observe_geometry(wall_packet())
    assert gp.geometry_rebuilds == 1 and gp.updates == 1 and len(gp.records) == 1
    fresh = GeometryPoissonGP(32)
    fresh.observe_geometry(wall_packet())
    fresh.update_counts_window([[-.3, 0., .5], [-.2, 0., .5]], [.25, .25], 20)
    query = [[-.2, 0., .5], [.2, 0., .5]]
    np.testing.assert_allclose(gp.predict(query), fresh.predict(query), atol=1e-9)
    archive = tmp_path/'geometry.npz'
    gp.save(archive)
    loaded = load_gp(archive)
    np.testing.assert_allclose(gp.predict(query), loaded.predict(query), atol=1e-9)
    loaded.observe_geometry(wall_packet(11., 2.))
    assert loaded.updates == 1 and len(loaded.records) == 1


def test_spatial_window_crossing_boundary_is_one_observation_per_expert():
    gp = SpatialGP()
    with pytest.raises(ValueError):
        gp.update_counts_window([[0., 0., .5]], [-.5], 12)
    assert not gp.experts and gp.updates == 0
    gp.update_counts_window([[-.1, .5, .5], [.1, .5, .5]], [.25, .25], 12)
    assert gp.updates == 1
    assert all(expert.updates == 1 for expert in gp.experts.values())
