import math

import numpy as np
import pytest

from dougie_sim.gp import OnlineGP, PoissonOnlineGP
from dougie_sim.spatial_gp import SpatialGP, load_gp


def test_halo_weights_partition_unity_and_are_continuous():
    gp = SpatialGP()
    points = np.random.default_rng(10).uniform(-10, 10, (200, 3))
    _, weights = gp.assignments(points)
    np.testing.assert_allclose(weights.sum(axis=1), 1.)
    assert np.all(weights >= 0) and np.all((weights > 0).sum(axis=1) <= 8)


def test_overlapping_observations_do_not_multiply_confidence():
    gp, single = SpatialGP(), PoissonOnlineGP(32)
    point = [4., 2., .75]
    for _ in range(10):
        gp.update_counts(point, 30, .5)
        single.update_counts(point, 30, .5)
    assert gp.updates == 10 and len(gp.experts) == 2
    assert all(e.updates == 10 for e in gp.experts.values())
    for actual, expected in zip(gp.predict([point]), single.predict([point])):
        np.testing.assert_allclose(actual, expected, rtol=1e-9)


def test_expert_disagreement_inflates_variance_and_preserves_hot_upper_rate():
    class Expert:
        def __init__(self, rate):
            self.mu = math.log(rate)
        def predict_log(self, points):
            return np.full(len(points), self.mu), np.full(len(points), .1)
    gp = SpatialGP()
    gp.experts[(0, 0, 0)] = Expert(2.)
    gp.experts[(1, 0, 0)] = Expert(100.)
    points = [[4.-1e-6, 2., .75], [4., 2., .75], [4.+1e-6, 2., .75]]
    mean, sigma, upper = gp.predict(points)
    assert sigma[1]**2 > .1+3.
    assert upper[1] >= 100.*math.exp(2*math.sqrt(.1))
    assert np.ptp(mean) < .001 and np.ptp(sigma) < .001


def test_unseen_heights_and_exhausted_regions_retain_prior_uncertainty():
    gp = SpatialGP(capacity=8, max_experts=1)
    for _ in range(50):
        gp.update_counts([2., 2., .75], 2, .5)
    _, variance = gp.predict_log([[2., 2., .75], [2., 2., 5.]])
    assert variance[1] == pytest.approx(gp.variance) and variance[0] < variance[1]
    gp.update_counts([20., 20., 5.], 400, .5)
    assert gp.saturated and len(gp.experts) == 1
    assert len(next(iter(gp.experts.values())).Z) <= 8
    gp.predict([[100., 100., 100.]])
    assert len(gp.experts) == 1  # Read-only queries never allocate models.


@pytest.mark.parametrize('likelihood', ['poisson', 'gaussian'])
def test_spatial_archive_round_trip_and_gaussian_interface(tmp_path, likelihood):
    gp = SpatialGP(16, 32, likelihood=likelihood)
    rng = np.random.default_rng(3)
    for point in rng.uniform([-3, -3, .4], [5, 5, 2.8], (80, 3)):
        if likelihood == 'poisson':
            gp.update_counts(point, 20, .5)
        else:
            gp.update(point, 3., .2)
    query = rng.uniform([-3, -3, .4], [5, 5, 2.8], (50, 3))
    gp.save(tmp_path/'posterior.npz')
    loaded = load_gp(tmp_path/'posterior.npz')
    for a, b in zip(gp.predict(query), loaded.predict(query)):
        np.testing.assert_allclose(a, b, rtol=1e-12)
    assert loaded.updates == gp.updates


def test_global_archive_round_trip_remains_supported(tmp_path):
    gp = OnlineGP(8)
    gp.update([0, 0, 0], 2., .1)
    gp.save(tmp_path/'global.npz')
    loaded = load_gp(tmp_path/'global.npz')
    np.testing.assert_allclose(loaded.predict([[1, 1, 1]]), gp.predict([[1, 1, 1]]))


def test_invalid_measurements_do_not_partially_update_experts():
    gp = SpatialGP()
    for point, counts in (([0, 0, 0], -1), ([float('nan'), 0, 0], 1)):
        with pytest.raises(ValueError):
            gp.update_counts(point, counts, .5)
    assert not gp.experts and gp.updates == 0


def test_initial_spatial_archive_without_explicit_hyperparameters_is_readable(tmp_path):
    gp = SpatialGP()
    gp.update_counts([2., 2., .75], 20, .5)
    gp.save(tmp_path/'new.npz')
    with np.load(tmp_path/'new.npz') as data:
        legacy = {k: data[k] for k in data.files if k not in ('lengthscales', 'variance', 'prior_log_rate')}
    np.savez_compressed(tmp_path/'initial.npz', **legacy)
    np.testing.assert_allclose(load_gp(tmp_path/'initial.npz').predict([[2., 2., .75]]), gp.predict([[2., 2., .75]]))
