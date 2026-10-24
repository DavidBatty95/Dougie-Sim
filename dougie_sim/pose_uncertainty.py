"""Conservative runtime use of an offline episode-level calibration artifact.

Runtime artifacts contain only an inflation factor and a pinned operating domain,
never evaluator poses. Coverage is marginal over exchangeable *episodes*, not an
unconditional guarantee for new layouts, sensor models or changed control policy.
"""
import hashlib
import json
import math
from decimal import Decimal, ROUND_CEILING
from pathlib import Path

import numpy as np

CORE = ('odometry.py', 'registration.py', 'pose_graph.py', 'scan_geometry.py', 'map_corrections.py',
        'pose_uncertainty.py', 'onboard_sensors.py', 'sensing.py', 'simulation.py', 'policy.py',
        'radiation.py', 'radiation_run.py', 'terrain.py', 'layered_terrain.py', 'dose_planner.py',
        'clustered_planner.py', 'planning_worker.py', 'path_motion.py', 'route_forecast.py',
        'motion_cost.py', 'mission_utility.py', 'region_clustering.py', 'region_agenda.py',
        'geometry_gp.py', 'gp.py', 'spatial_gp.py', 'recovery.py', 'reactor.py', 'multilevel.py', 'facility.py',
        'source_config.py', 'observed_circuit.py', 'mission_evaluation.py', 'circuit_evaluation.py', 'stance_hold.py', 'localization_checkpoint.py')
DOMAIN_KEYS = ('world', 'layout_seed', 'mezzanine_angle', 'source_profile', 'duration', 'dose_budget', 'rate_limit',
               'localization', 'navigation_profile', 'slam_backend', 'gp_mode', 'gp_likelihood',
               'gp_capacity', 'gp_expert_capacity', 'gp_max_experts', 'gp_cell_size', 'detector_likelihood',
               'path_mode', 'terrain_profile', 'mission_objective', 'utility_mode', 'region_scheduling',
               'fixed_plan_delay', 'fast', 'ros', 'lidar_dropout_after', 'circuit_radius',
               'require_clean', 'require_round_trip', 'require_circuit', 'headless', 'visualizer',
               'record_demo', 'record_view', 'show_sources', 'terminal_hold', 'support_fusion', 'record_support', 'loop_search_radius', 'localization_checkpoints')


def core_fingerprint():
    root = Path(__file__).resolve().parent
    return {'dougie_sim/'+name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in CORE}


def domain(config):
    result = {key: config.get(key) for key in DOMAIN_KEYS}
    result['policy'] = config.get('policy')
    result['software_versions'] = config.get('software_versions')
    result['field_config_sha256'] = config.get('source_sha256', {}).get('radiation_source_config')
    return json.loads(json.dumps(result))  # Normalize tuple-valued CLI settings.


def quantile_rank(n, alpha):
    if n < 1 or not math.isfinite(alpha) or not 0 < alpha < 1:
        raise ValueError('invalid calibration size or miscoverage')
    return int((Decimal(n+1)*(Decimal(1)-Decimal(str(alpha)))).to_integral_value(rounding=ROUND_CEILING))


def fit_episode_bound(scores, alpha=.1):
    scores = np.asarray(scores, dtype=float)
    if scores.ndim != 1 or not np.isfinite(scores).all() or np.any(scores < 0):
        raise ValueError('every declared calibration episode needs finite telemetry')
    rank = quantile_rank(len(scores), alpha)
    if rank > len(scores):
        raise ValueError('insufficient independent episodes for a finite requested-coverage bound')
    return max(1., float(np.sort(scores)[rank-1])), rank


class PoseUncertaintyScale:
    def __init__(self, multiplier=1., identity=None, alpha=None):
        if not math.isfinite(multiplier) or not 1. <= multiplier <= 100.:
            raise ValueError('uncertainty inflation must be finite and between 1 and 100')
        self.multiplier, self.identity, self.alpha = multiplier, identity, alpha

    @classmethod
    def load(cls, path, config):
        raw = Path(path).read_bytes()
        profile = json.loads(raw)
        allowed = {'schema_version', 'method', 'alpha', 'base_sigma_multiple', 'sigma_multiplier',
                   'calibration_episodes', 'quantile_rank', 'calibration_ids', 'calibration_scores', 'domain', 'source_sha256', 'scope'}
        if set(profile) != allowed or profile['schema_version'] != 1 or profile['method'] != 'episode_max_split_conformal':
            raise ValueError('unsupported or unexpected uncertainty-profile content')
        n = profile['calibration_episodes']
        if (not isinstance(n, int) or n < 1 or profile['base_sigma_multiple'] != 3.
                or profile['quantile_rank'] != quantile_rank(n, profile['alpha']) or profile['quantile_rank'] > n
                or len(profile['calibration_ids']) != n or len(set(profile['calibration_ids'])) != n):
            raise ValueError('invalid episode-level calibration evidence')
        scale = cls(float(profile['sigma_multiplier']), hashlib.sha256(raw).hexdigest(), profile['alpha'])
        expected, _ = fit_episode_bound(profile['calibration_scores'], profile['alpha'])
        if len(profile['calibration_scores']) != n or not math.isclose(scale.multiplier, expected, rel_tol=0., abs_tol=1e-12):
            raise ValueError('calibration factor does not match the episode quantile')
        if profile['source_sha256'] != core_fingerprint():
            raise ValueError('uncertainty profile does not match current estimator/control source')
        if profile['domain'] != domain(config):
            raise ValueError('uncertainty profile operating domain does not match this mission')
        return scale

    def apply(self, std):
        return np.asarray(std)*self.multiplier

    def metrics(self):
        return dict(profile_sha256=self.identity, sigma_multiplier=self.multiplier,
            reference_episode_miscoverage=self.alpha,
            scope='reference-policy episode bound under exchangeability; control feedback/domain shift require fresh validation')
