import numpy as np
import pytest

from dougie_sim.gp import OnlineGP, PoissonOnlineGP


@pytest.mark.parametrize('kind', [OnlineGP, PoissonOnlineGP])
def test_prepared_prediction_matches_reference_after_updates_and_chunking(kind):
    rng = np.random.default_rng(43)
    gp = kind(capacity=24)
    queries = rng.normal(size=(137, 3))
    gp.prediction_chunk_size = 17
    for i, point in enumerate(rng.normal(size=(45, 3))):
        if kind is PoissonOnlineGP:
            gp.update_counts(point, i % 13, .5)
        else:
            gp.update(point, i % 13, .5)
        assert gp.prediction_cache_bytes == 0
        gp.use_prepared_predictions = False
        expected = gp.predict(queries)
        gp.use_prepared_predictions = True
        actual = gp.predict(queries)
        np.testing.assert_allclose(actual, expected, rtol=1e-9, atol=1e-9)
        builds = gp.prediction_cache_builds
        np.testing.assert_allclose(gp.predict(queries), actual)
        assert gp.prediction_cache_builds == builds
        assert gp.prediction_cache_bytes > 0
    assert gp.predict_log(np.empty((0, 3)))[0].shape == (0,)
