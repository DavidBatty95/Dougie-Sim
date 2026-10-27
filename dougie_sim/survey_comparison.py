"""Observed-route survey admission and independent comparison outcome labels."""
import math

import numpy as np

# Compact values shared with the evaluator minimap. Unknown is never admissible.
UNKNOWN, AVAILABLE, RADIATION, BUDGET, DISCONNECTED, TIME, TEMPORARY = range(7)
STATUS_NAMES = ('unknown', 'available', 'radiation', 'budget', 'disconnected', 'time', 'temporary')


def survey_admission(explorer, keys, root, home, rates, parent, home_parent,
                     dose, home_dose, outbound_time, home_time, stamp):
    """Classify known support using physical connectivity separately from risk."""
    _, _, _, physical = explorer.terrain.routes(root)
    _, _, _, physical_home = explorer.terrain.routes(home, reverse=True)
    statuses = {}
    for key in keys:
        if key not in physical or key not in physical_home:
            status = DISCONNECTED
        elif key not in parent or key not in home_parent or rates[key] > explorer.rate_limit:
            status = RADIATION
        elif any(math.dist(explorer.terrain.nodes[key], point) < .65 for point, _ in explorer.failed):
            status = TEMPORARY
        else:
            dwell = (rates[root]*explorer.wait_allowance+
                     rates[key]*(5.+explorer.wait_allowance+explorer.recovery_reserve_s))/3600.
            total = explorer.sample.dose_upper_usv+dose[key]+home_dose[key]+explorer.budget*.15+dwell
            deadline = stamp+outbound_time[key]+home_time[key]+2*explorer.wait_allowance+15.+explorer.recovery_reserve_s
            status = BUDGET if total > explorer.budget else TIME if deadline > explorer.mission_duration else AVAILABLE
        statuses[key] = status
    return statuses


def outcome(complete, state, reason, terminal=False, error=None):
    if error:
        return 'failed'
    if complete:
        return 'survey_complete'
    if state == 'interrupted':
        return 'interrupted'
    if reason in ('time_return_reserve', 'time_limit'):
        return 'time_limit'
    if reason in ('radiation_frontiers_blocked', 'no_rate_compliant_return_route',
                  'dose_return_reserve', 'measured_rate_limit', 'measured_dose_reserve',
                  'measured_radiation_requires_retreat'):
        return 'radiation_limited'
    if terminal:
        return 'navigation_stopped'
    if state == 'returned':
        return 'observed_frontiers_exhausted' if reason == 'survey_frontiers_exhausted' else 'incomplete_return'
    return 'surveying'


def project_admission(reference, positions, statuses, rotation=None, translation=None):
    """Evaluator only: align observed planner support to nearby reference cells.

    No extrapolation across unknown rooms or between floors. This is evidence
    about known support/return routes, not a whole-world feasibility oracle.
    """
    from scipy.spatial import cKDTree
    result = np.zeros(len(reference), dtype=np.uint8)
    positions = np.asarray(positions, dtype=float).reshape(-1, 3)
    if not len(positions):
        return result
    if rotation is not None:
        positions = positions@rotation.T+translation
    distance, index = cKDTree(positions).query(reference)
    valid = distance <= .4
    result[valid] = np.asarray(statuses, dtype=np.uint8)[index[valid]]
    return result
