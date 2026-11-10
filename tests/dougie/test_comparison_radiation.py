import json

import numpy as np

from dougie_sim.comparison_radiation import RadiationMapComparison
from dougie_sim.radiation import RadiationField


def test_truth_and_gp_share_detector_height_and_do_not_fill_unqueried_floors(tmp_path):
    field = RadiationField(dict(background_usv_h=.2, sources=[dict(
        position=[0., 0., .44], strength_usv_h_m2=2., core_radius_m=.5)]))
    points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 0., 2.4]])
    comparison = RadiationMapComparison(points, field, [0., 2.4])
    # Rotate/translate an exported GP query into the matching render frame.
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    translation = np.array([1., 0., 0.])
    volume = (np.array([[0., 1., .44]]), np.array([3.]), np.array([.4]), np.array([8.]))
    original = [a.copy() for a in volume]
    comparison.update(volume, rotation, translation, 7.)
    comparison.save(tmp_path, [True, False, False])
    payload = json.loads((tmp_path/'radiation_map.json').read_text())
    assert payload['truth_usv_h'][0] == 8.2
    assert payload['gp_mean_usv_h'] == [3., None, None]
    assert payload['gp_std_log_rate'] == [.4, None, None]
    assert payload['layers'] == [0, 0, 1]
    assert payload['predicted_cells'] == 1 and payload['time_s'] == 7.
    for before, after in zip(original, volume):
        np.testing.assert_array_equal(before, after)
    # A new snapshot must clear old predictions, not retain a misleading stale map.
    comparison.update(None, np.eye(3), np.zeros(3), 9.)
    assert comparison.payload([False]*3)['gp_mean_usv_h'] == [None]*3
