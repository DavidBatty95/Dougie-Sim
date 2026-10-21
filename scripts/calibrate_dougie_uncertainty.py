#!/usr/bin/env python3
"""Fit conservative episode-max pose envelopes; evaluate disjoint held-out runs.

Evaluator truth is used offline only. Temporal samples are NOT counted as
independent calibration examples, and failed missions are not filtered out.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from dougie_sim.pose_uncertainty import CORE, PoseUncertaintyScale, domain, fit_episode_bound


def episode(run):
    config_file, telemetry_file = run/'run_config.json', run/'localization_evaluation.jsonl'
    config = json.loads(config_file.read_text())
    records = [json.loads(line) for line in telemetry_file.read_text().splitlines() if line.strip()]
    if not records:
        raise ValueError(f'{run}: missing localization telemetry')
    estimate = np.array([r['position'] for r in records])
    truth = np.array([r['evaluator_true_position'] for r in records])
    std = np.array([r.get('uncalibrated_position_std_m', r['position_std_m']) for r in records])
    if not np.isfinite(np.r_[estimate.ravel(), truth.ravel(), std.ravel()]).all() or np.any(std <= 0):
        raise ValueError(f'{run}: invalid telemetry; episode cannot be dropped silently')
    difference = estimate-truth
    aligned = difference-difference[0]  # Initial translation gauge only; no fitted trajectory alignment.
    scores = np.max(np.abs(aligned)/(3.*std), axis=1)
    source = config.get('source_sha256', {})
    names = ['dougie_sim/'+name for name in CORE]
    if any(name not in source for name in names) or not config.get('software_versions'):
        raise ValueError(f'{run}: source/software provenance predates calibration protocol')
    summary_file = run/'radiation_summary.json'
    summary = json.loads(summary_file.read_text())
    evaluation = summary.get('localization_evaluation', {})
    episode_score = evaluation.get('episode_max_three_sigma_ratio')
    if episode_score is None or not np.isfinite(episode_score) or evaluation.get('episode_control_tick_samples', 0) < len(records):
        raise ValueError(f'{run}: missing all-control-tick episode maximum')
    identity = hashlib.sha256(telemetry_file.read_bytes()+summary_file.read_bytes()+config_file.read_bytes()).hexdigest()
    return dict(run=str(run), identity=identity,
        telemetry_sha256=hashlib.sha256(telemetry_file.read_bytes()).hexdigest(),
        summary_sha256=hashlib.sha256(summary_file.read_bytes()).hexdigest(),
        config_sha256=hashlib.sha256(config_file.read_bytes()).hexdigest(),
        sensor_seed=config['sensor_seed'], detector_seed=config['seed'], domain=domain(config),
        source_sha256={name: source[name] for name in names},
        already_scaled=bool(config.get('uncertainty_profile')), samples=len(records),
        episode_score=float(episode_score) if summary.get('error') is None else None, logged_sample_score=float(scores.max()),
        control_tick_samples=evaluation['episode_control_tick_samples'], clean_return=summary.get('clean_return'),
        end_state=summary.get('end_state'), error=summary.get('error'),
        aligned_position_rmse_m=float(np.sqrt(np.mean(np.sum(aligned**2, axis=1)))))


def calibrate(training, heldout, alpha=.1):
    if not training or not heldout:
        raise ValueError('separate calibration and held-out episodes are required')
    all_runs = training+heldout
    if len({r['identity'] for r in all_runs}) != len(all_runs) or len({r['sensor_seed'] for r in all_runs}) != len(all_runs):
        raise ValueError('calibration/test episodes must have disjoint telemetry and sensor seeds')
    reference = training[0]
    if any(r['domain'] != reference['domain'] or r['source_sha256'] != reference['source_sha256'] for r in all_runs):
        raise ValueError('calibration and validation must use the same declared source and operating domain')
    if any(r['already_scaled'] for r in training):
        raise ValueError('fit on unscaled reference-policy episodes, not previously calibrated feedback runs')
    multiplier, rank = fit_episode_bound([r['episode_score'] for r in training], alpha)
    PoseUncertaintyScale(multiplier)  # Refuse unusable runtime artifacts instead of clipping a bound.
    profile = dict(schema_version=1, method='episode_max_split_conformal', alpha=alpha,
        base_sigma_multiple=3., sigma_multiplier=multiplier, calibration_episodes=len(training),
        quantile_rank=rank, calibration_ids=[r['identity'] for r in training],
        calibration_scores=[r['episode_score'] for r in training],
        domain=reference['domain'], source_sha256=reference['source_sha256'],
        scope='marginal coverage of active control-tick poses requires exchangeable reference-policy episodes; no guarantee under domain/control-policy shift')
    report = dict(profile=profile, calibration=training, heldout=heldout,
        heldout_episode_coverage=float(np.mean([r['episode_score'] is not None and r['episode_score'] <= multiplier for r in heldout])),
        heldout_episodes=len(heldout),
        scope='held-out empirical validation; small test cohorts do not prove the nominal coverage rate')
    return profile, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-runs', nargs='+', type=Path, required=True)
    parser.add_argument('--test-runs', nargs='+', type=Path, required=True)
    parser.add_argument('--alpha', type=float, default=.1)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    try:
        profile, report = calibrate([episode(p) for p in args.calibration_runs], [episode(p) for p in args.test_runs], args.alpha)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.profile.write_text(json.dumps(profile, indent=2, allow_nan=False)+'\n')
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(profile=str(args.profile), sigma_multiplier=profile['sigma_multiplier'],
        heldout_episode_coverage=report['heldout_episode_coverage'], heldout_episodes=len(report['heldout'])), indent=2))


if __name__ == '__main__':
    main()
