"""Offline prediction scoring. Evaluator truth must never enter autonomy."""
import numpy as np
from scipy.spatial import cKDTree


def prediction_report(query, truth, mean, upper, training_positions, counts, exposure, shields=()):
    query, training_positions = np.asarray(query), np.asarray(training_positions)
    truth, mean, upper, counts, exposure = map(np.asarray, (truth, mean, upper, counts, exposure))
    distance = cKDTree(training_positions).query(query)[0]
    masks = {'all': np.ones(len(query), dtype=bool),
             'within_0.5m_of_training': distance <= .5, 'beyond_0.5m_of_training': distance > .5,
             'high_rate_top_decile': truth >= np.quantile(truth, .9)}
    if shields:
        boundary_distance = np.full(len(query), np.inf)
        for shield in shields:
            lo, hi = np.asarray(shield['lower']), np.asarray(shield['upper'])
            outside = np.linalg.norm(np.maximum(np.maximum(lo-query, query-hi), 0.), axis=1)
            inside = np.minimum(query-lo, hi-query).min(axis=1)
            boundary_distance = np.minimum(boundary_distance, np.where(inside >= 0, inside, outside))
        masks['within_0.5m_of_shield_boundary'] = boundary_distance <= .5
    for band in sorted(set(np.floor(query[:, 2]/1.5).astype(int))):
        masks[f'sensor_height_{band*1.5:g}_to_{(band+1)*1.5:g}m'] = np.floor(query[:, 2]/1.5) == band
    expected = np.maximum(mean*exposure, 1e-12)
    deviance = 2.*(counts*np.log(np.maximum(counts, 1.)/expected)-counts+expected)
    result = {}
    for name, mask in masks.items():
        n = int(mask.sum())
        result[name] = dict(samples=n,
            log1p_rate_rmse=float(np.sqrt(np.mean((np.log1p(mean[mask])-np.log1p(truth[mask]))**2))) if n else None,
            upper_rate_coverage=float(np.mean(upper[mask] >= truth[mask])) if n else None,
            mean_upper_shortfall_usv_h=float(np.mean(np.maximum(truth[mask]-upper[mask], 0.))) if n else None,
            poisson_deviance_at_predicted_mean=float(np.mean(deviance[mask])) if n else None)
    return result
