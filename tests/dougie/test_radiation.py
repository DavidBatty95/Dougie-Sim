import math

import numpy as np
import pytest

from dougie_sim.radiation import Detector, RadiationField, segment_box_length
from dougie_sim.gp import OnlineGP, PoissonOnlineGP
from dougie_sim.simulation import Simulation
from dougie_sim.terrain import SurfaceMap


def test_inverse_square_3d_and_shield_attenuation():
    source = dict(position=[0, 0, 0], strength_usv_h_m2=100., core_radius_m=.01)
    field = RadiationField(dict(background_usv_h=0., sources=[source]))
    assert field.rate((1., 0., 0.))/field.rate((2., 0., 0.)) == pytest.approx(4., rel=1e-3)
    assert field.rate((0., 0., 2.)) == pytest.approx(field.rate((2., 0., 0.)))
    shield = dict(lower=[.5, -1, -1], upper=[1.5, 1, 1], attenuation_per_m=2.)
    attenuated = RadiationField(dict(background_usv_h=0., sources=[source], shields=[shield]))
    assert attenuated.rate((2., 0., 0.))/field.rate((2., 0., 0.)) == pytest.approx(math.exp(-2.))
    assert segment_box_length([0, 0, 0], [2, 0, 0], **{k:v for k,v in shield.items() if k != 'attenuation_per_m'}) == 1.


def test_counting_detector_units_seed_and_exposure_integration():
    first, second = Detector(seed=4), Detector(seed=4)
    for step in range(200):
        a = first.integrate([0, 0, 1], 36., .05, (step+1)*.05)
        b = second.integrate([0, 0, 1], 36., .05, (step+1)*.05)
        assert a == b
    assert first.samples == 20
    assert first.dose == pytest.approx(.1, rel=.05)
    assert a.dose_upper_usv > a.dose_usv
    assert a.variance_usv_h2 > 0.


def test_sources_are_encoded_in_mujoco_scene():
    sim = Simulation('reactor')
    field = RadiationField.from_mujoco(sim.model, sim.data)
    assert len(field.sources) == 3 and len(field.shields) == 4
    assert field.rate([5.8, 3.8, 1.2]) > field.rate([0, 0, .44])


def test_streaming_gp_matches_exact_posterior_when_every_sample_is_an_inducing_point():
    gp = OnlineGP(capacity=32, lengthscales=(.2, .2, .2))
    X = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 1.]])
    rates = np.array([2., 4., 8.])
    variances = .04*(1.+rates)**2
    for point, rate, variance in zip(X, rates, variances):
        gp.update(point, rate, variance)
    query = np.array([[.1, 0., 0.], [0., 1., .9]])
    K = gp.kernel(X, X)+np.eye(3)*.04
    k = gp.kernel(X, query)
    expected = gp.prior+k.T@np.linalg.solve(K, np.log1p(rates)-gp.prior)
    expected_var = gp.variance-np.sum(k*np.linalg.solve(K, k), axis=0)
    mean, variance = gp.predict_log(query)
    np.testing.assert_allclose(mean, expected, atol=1e-6)
    np.testing.assert_allclose(variance, expected_var, atol=1e-6)


def test_gp_is_bounded_and_unknown_height_retains_uncertainty():
    gp = OnlineGP(capacity=8)
    for x in np.linspace(-5, 5, 100):
        gp.update([x, 0., 0.], 2.+math.exp(-x*x), .1)
    assert len(gp.Z) <= 8
    _, variance = gp.predict_log([[0, 0, 0], [0, 0, 5]])
    assert variance[1] > variance[0]
    mean, _, upper = gp.predict([[0, 0, 0], [0, 0, 5]])
    assert np.all(upper >= mean) and np.all(mean >= 0)
    with pytest.raises(ValueError):
        gp.update([0, 0, 0], float('nan'), .1)


def test_poisson_gp_handles_zero_counts_and_recovers_constant_rate():
    gp = PoissonOnlineGP()
    for _ in range(50):
        gp.update_counts([0, 0, 0], 0, .5)
    low, std, upper = gp.predict([[0, 0, 0]])
    assert 0 < low[0] < .05 and upper[0] > low[0]
    gp = PoissonOnlineGP()
    # 50 counts in 0.5 s with 50,000 counts/uSv corresponds to 7.2 uSv/h.
    for _ in range(50):
        gp.update_counts([0, 0, 0], 50, .5)
    mean, _, _ = gp.predict([[0, 0, 0]])
    assert mean[0] == pytest.approx(7.2, rel=.03)
    assert np.linalg.eigvalsh(gp.cov).min() >= -1e-9
    with pytest.raises(ValueError):
        gp.update_counts([0, 0, 0], -1, .5)


@pytest.mark.parametrize('rise,connected', [(.06, True), (.25, False)])
def test_surface_graph_admits_small_steps_and_rejects_large_steps(rise, connected):
    terrain = SurfaceMap()
    supports = np.array([[x, y, 0. if x < 1. else rise]
                         for x in np.arange(0., 2.5, .1) for y in np.arange(-1., 1., .1)])
    terrain.update(supports, supports)
    terrain.build_graph()
    start = terrain.nearest([.5, 0., 0.])
    end = terrain.nearest([2., 0., rise])
    assert start is not None and end is not None
    _, _, _, parents = terrain.routes(start)
    assert (end in parents) is connected


def test_stacked_surfaces_do_not_create_a_vertical_teleport_edge():
    terrain = SurfaceMap()
    supports = np.array([[x, y, z] for x in np.arange(0., 2., .1)
                         for y in np.arange(0., 2., .1) for z in (0., 2.)])
    terrain.update(supports, supports)
    terrain.build_graph()
    start, end = terrain.nearest([1., 1., 0.]), terrain.nearest([1., 1., 2.])
    assert start is not None and end is not None
    _, _, _, parents = terrain.routes(start)
    assert end not in parents


def test_lidar3d_produces_real_xyz_support_observations():
    sim = Simulation('reactor')
    points, supports = sim.lidar3d()
    assert points.shape[1] == 3 and len(supports) > 100
    assert np.ptp(points[:, 2]) > 1.
    terrain = SurfaceMap()
    terrain.update(points, supports, sim.data.qpos[:3]+[0, 0, .12])
    terrain.build_graph()
    assert terrain.nearest([0, 0, 0]) is not None
    assert terrain.free_voxels
