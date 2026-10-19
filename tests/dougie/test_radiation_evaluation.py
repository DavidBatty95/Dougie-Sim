import numpy as np
import pytest

from dougie_sim.radiation_evaluation import prediction_report


def test_stratified_scoring_exposes_far_underprediction_and_empty_groups():
    report = prediction_report(np.array([[0., 0., .4], [3., 0., .4], [3., 0., 2.8]]),
        [1., 10., 20.], [1., 5., 5.], [2., 6., 6.], [[0., 0., .4]], [0, 10, 20], [1., 1., 1.],
        [dict(lower=[10., 10., 0.], upper=[11., 11., 3.])])
    assert report['within_0.5m_of_training']['upper_rate_coverage'] == 1.
    assert report['beyond_0.5m_of_training']['upper_rate_coverage'] == 0.
    assert report['beyond_0.5m_of_training']['mean_upper_shortfall_usv_h'] == 9.
    assert report['within_0.5m_of_training']['poisson_deviance_at_predicted_mean'] == pytest.approx(2.)
    assert report['within_0.5m_of_shield_boundary']['samples'] == 0
    assert report['within_0.5m_of_shield_boundary']['upper_rate_coverage'] is None
    assert report['sensor_height_1.5_to_3m']['samples'] == 1
