import copy
import json

import numpy as np
import pytest

from dougie_sim.pose_uncertainty import PoseUncertaintyScale, core_fingerprint, domain, fit_episode_bound
from scripts.calibrate_dougie_uncertainty import calibrate


def record(index, score=.5):
    return dict(identity=str(index), sensor_seed=index, domain={'world': 'mezzanine'},
        source_sha256={'test': 'fixed'}, already_scaled=False, episode_score=score)


def test_episode_quantile_requires_enough_independent_runs_and_never_shrinks():
    with pytest.raises(ValueError, match='insufficient'):
        fit_episode_bound([.1]*8, .1)
    scale, rank = fit_episode_bound([.1]*8+[2.], .1)
    assert rank == 9 and scale == 2.
    assert fit_episode_bound([.1]*19, .05) == (1., 19)
    with pytest.raises(ValueError, match='finite'):
        fit_episode_bound([.1]*8+[float('inf')], .1)


def test_validation_is_held_out_and_duplicate_sensor_seeds_are_rejected():
    training = [record(i) for i in range(9)]
    heldout = [record(20, 2.), record(21, .2)]
    profile, report = calibrate(training, heldout)
    assert profile['sigma_multiplier'] == 1.  # Held-out error cannot tune the scale.
    assert report['heldout_episode_coverage'] == .5
    with pytest.raises(ValueError, match='disjoint'):
        calibrate(training, [record(0)])
    shifted = record(30)
    shifted['domain'] = {'world': 'facility'}
    with pytest.raises(ValueError, match='domain'):
        calibrate(training, [shifted])


def test_runtime_profile_contains_only_aggregates_and_pins_domain_and_source(tmp_path):
    config = dict(world='mezzanine', duration=300., policy={'sha256': 'test'}, software_versions={'test': '1'})
    records = [dict(record(i), domain=domain(config), source_sha256=core_fingerprint()) for i in range(10)]
    profile, _ = calibrate(records[:9], records[9:])
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(profile))
    scale = PoseUncertaintyScale.load(path, config)
    np.testing.assert_array_equal(scale.apply([.1, .2, .3]), [.1, .2, .3])
    with pytest.raises(ValueError, match='domain'):
        PoseUncertaintyScale.load(path, {**config, 'world': 'facility'})
    corrupt = copy.deepcopy(profile)
    corrupt['sigma_multiplier'] = .5
    path.write_text(json.dumps(corrupt))
    with pytest.raises(ValueError, match='inflation'):
        PoseUncertaintyScale.load(path, config)
    corrupt = copy.deepcopy(profile)
    corrupt['evaluator_positions'] = [[0., 0., 0.]]
    path.write_text(json.dumps(corrupt))
    with pytest.raises(ValueError, match='unexpected'):
        PoseUncertaintyScale.load(path, config)


def test_calibrated_runtime_bound_only_inflates_the_onboard_allowance():
    from dougie_sim.odometry import FusedOdometry
    odometry = FusedOdometry()
    raw = odometry.uncalibrated_position_std.copy()
    odometry.uncertainty_scale = PoseUncertaintyScale(2.)
    np.testing.assert_allclose(odometry.position_std, 2.*raw)
    np.testing.assert_array_equal(odometry.uncalibrated_position_std, raw)
