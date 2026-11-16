import numpy as np

from dougie_sim.geometry_gp import GeometryPoissonGP
from dougie_sim.gp import PoissonOnlineGP
from dougie_sim.spatial_gp import load_gp
from dougie_sim.windowed_gp import WindowedGP


def test_paging_bounds_residency_without_forgetting_distant_beliefs(tmp_path):
    gp = WindowedGP(capacity=8, max_experts=2, max_planes=0, store_dir=tmp_path/'cache')
    first = np.array([[2., 2., .75]])
    gp.update_counts(first[0], 40, .5)
    for _ in range(7):
        gp.update_counts(first[0], 40, .5)
    expected = gp.predict(first)
    for x in (10., 18., 26., 34.):
        gp.update_counts([x, 2., .75], 3, .5)
        assert len(gp.experts) <= 2
    assert len(gp.catalog) == 5 and gp.evictions >= 3 and not gp.saturated
    for actual, before in zip(gp.predict(first), expected):
        np.testing.assert_allclose(actual, before, rtol=1e-10, atol=1e-10)
    assert gp.page_reads > 0
    known = len(gp.catalog)
    mean, sigma, upper = gp.predict([[1000., 1000., .75]])
    assert len(gp.catalog) == known and mean[0] > 0 and sigma[0] > 0 and upper[0] >= mean[0]


def test_only_observation_neighbours_update_and_count_windows_are_not_split(tmp_path):
    gp = WindowedGP(capacity=8, max_experts=2, max_planes=0, store_dir=tmp_path/'cache')
    gp.update_counts([22., 2., .75], 3, .5)
    distant = dict(gp.catalog[(5, 0, 0)])
    positions = np.array([[3.98, 2., .75], [4.02, 2., .75]])
    gp.update_counts_window(positions, [.25, .25], 17)
    assert gp.catalog[(5, 0, 0)] == distant
    assert gp.last_window_experts == 2
    for key in ((0, 0, 0), (1, 0, 0)):
        expert = gp._get(key)
        assert expert.updates == 1 and expert.records[-1][2] == 17
        np.testing.assert_array_equal(expert.records[-1][0], positions)


def test_local_history_limit_keeps_streaming_without_replaying_partial_history():
    gp = GeometryPoissonGP(8, max_planes=0, replay_capacity=2, streaming_after_capacity=True)
    reference = PoissonOnlineGP(8)
    for count in (2, 4, 30, 10, 1, 20):
        gp.update_counts_window([[1., 1., .5]], [.5], count)
        reference.update_counts_window([[1., 1., .5]], [.5], count)
    assert gp.updates == 6 and len(gp.records) == 2
    assert gp.geometry_frozen and not gp.saturated
    assert gp.streaming_updates_after_history == 4
    for actual, expected in zip(gp.predict([[1., 1., .5]]), reference.predict([[1., 1., .5]])):
        np.testing.assert_allclose(actual, expected, rtol=1e-10)


def test_windowed_checkpoint_is_self_contained_lazy_and_continues_learning(tmp_path):
    gp = WindowedGP(capacity=8, max_experts=1, replay_capacity=2, max_planes=0, store_dir=tmp_path/'cache')
    points = [[2., 2., .75], [10., 2., .75], [18., 2., .75]]
    for point in points:
        for count in (10, 20, 30):
            gp.update_counts(point, count, .5)
    expected = gp.predict(points)
    gp.save(tmp_path/'posterior.npz')
    restored = load_gp(tmp_path/'posterior.npz')
    assert isinstance(restored, WindowedGP) and len(restored.experts) == 0
    for actual, before in zip(restored.predict(points), expected):
        np.testing.assert_allclose(actual, before, rtol=1e-10)
    assert len(restored.experts) == 0 and restored.updates == 9
    restored.update_counts(points[0], 100, .5)
    assert restored.updates == 10 and not restored.saturated
    restored.save(tmp_path/'updated.npz')
    final = load_gp(tmp_path/'updated.npz')
    for actual, before in zip(final.predict(points), restored.predict(points)):
        np.testing.assert_allclose(actual, before, rtol=1e-10)


def test_initial_prior_is_calibrated_from_counts_without_removing_uncertainty(tmp_path):
    gp = WindowedGP(capacity=8, max_experts=2, max_planes=0, store_dir=tmp_path/'cache')
    for _ in range(8):
        gp.update_counts([2.,2.,.75], 2, .5)
    # 16 counts / (4 seconds * 50,000 counts/uSv / 3600 seconds/hour).
    expected_rate = 16./(4.*50000./3600.)
    mean, sigma, upper = gp.predict([[100.,100.,.75]])
    np.testing.assert_allclose(mean, [expected_rate], rtol=1e-10)
    assert sigma[0] > 1. and upper[0] > mean[0]
    assert gp.prior_calibrated and gp.updates == 8
    gp.save(tmp_path/'calibrated.npz')
    restored = load_gp(tmp_path/'calibrated.npz')
    original_prior = restored.prior
    restored.update_counts([100.,100.,.75], 200, .5)
    assert restored.prior == original_prior  # Startup calibration is not a global re-fit on every update.


def test_prediction_queries_do_not_evict_training_windows_or_load_history(tmp_path):
    import pytest
    gp = WindowedGP(capacity=8, max_experts=1, max_planes=0, store_dir=tmp_path/'cache')
    gp.update_counts([2.,2.,.75], 5, .5)
    gp.update_counts([10.,2.,.75], 8, .5)
    resident = tuple(gp.experts)
    evictions = gp.evictions
    view = gp._prediction_expert((0,0,0))
    assert not view.history_loaded and not view.records
    assert tuple(gp.experts) == resident and gp.evictions == evictions
    with pytest.raises(RuntimeError):
        view.update_counts([2.,2.,.75], 4, .5)
    gp.predict([[2.,2.,.75]])
    assert tuple(gp.experts) == resident


def test_empty_zero_weight_window_does_not_override_a_measured_return_location(tmp_path):
    gp=WindowedGP(capacity=8,max_experts=2,max_planes=0,store_dir=tmp_path/'cache')
    point=[2.,2.,.44]
    for _ in range(40):
        gp.update_counts(point,1,.5)
    # z=-1 is inside the expanded upper support but has no observations and
    # exactly zero ordinary mixture weight at this measured detector height.
    assert (0,0,-1) not in gp.catalog
    own=gp._get((0,0,0)).predict([point])
    actual=gp.predict([point])
    for a,b in zip(actual,own):
        np.testing.assert_allclose(a,b,rtol=1e-8,atol=1e-10)
    mean,std,upper=gp.predict([[2.,2.,10.]])
    assert std[0]>1. and upper[0]>mean[0]>0.
