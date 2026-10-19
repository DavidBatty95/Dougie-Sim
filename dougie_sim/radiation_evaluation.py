"""Offline prediction scoring. Evaluator truth must never enter autonomy."""
import numpy as np
from scipy.spatial import cKDTree


def perturb_positions(positions, stamps, *, sigma_m=0., drift_m_sqrt_s=0., seed=0):
    """Offline sensitivity model, not an odometry estimator or sensor simulator.

    Independent XYZ measurement jitter plus a time-scaled random walk. Perturb
    training and query positions in one chronological stream to preserve drift.
    """
    positions, stamps = np.asarray(positions, dtype=float), np.asarray(stamps, dtype=float)
    if (positions.shape != (len(stamps), 3) or not np.isfinite(positions).all()
            or not np.isfinite(stamps).all() or np.any(np.diff(stamps) < 0)
            or not np.isfinite([sigma_m, drift_m_sqrt_s]).all() or min(sigma_m, drift_m_sqrt_s) < 0):
        raise ValueError('invalid position sensitivity inputs')
    rng = np.random.default_rng(seed)
    dt = np.diff(stamps, prepend=stamps[0]) if len(stamps) else stamps
    drift = np.cumsum(rng.normal(size=positions.shape)*np.sqrt(dt[:, None])*drift_m_sqrt_s, axis=0)
    return positions+drift+rng.normal(size=positions.shape)*sigma_m


def upper_bound_report(truth, upper):
    truth, upper = np.asarray(truth), np.asarray(upper)
    shortfall = np.maximum(truth-upper, 0.)
    return dict(coverage=float(np.mean(upper >= truth)),
                mean_shortfall_usv_h=float(shortfall.mean()),
                worst_shortfall_usv_h=float(shortfall.max()),
                mean_excess_usv_h=float(np.maximum(upper-truth, 0.).mean()))


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
