#!/usr/bin/env python3
"""Matched onboard reactor missions at 2 and 10 uSv; retain every outcome.

Physical visitation is scored offline from the evaluator log. It is a trajectory
proxy, not complete surveyed area, and never crosses into mission planning.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def score(run, returncode):
    summary_file = run/'radiation_summary.json'
    if not summary_file.exists():
        return dict(run=run.name, returncode=returncode, clean_return=False, error='missing summary')
    summary = json.loads(summary_file.read_text())
    config = json.loads((run/'run_config.json').read_text())
    poses = [json.loads(line) for line in (run/'localization_evaluation.jsonl').read_text().splitlines() if line.strip()]
    points = np.asarray([p['evaluator_true_position'] for p in poses]).reshape(-1, 3)
    cells = np.floor((points[:, :2]+.25)/.5).astype(int)
    unique = len(set(map(tuple, cells)))
    localized = [p for p in poses if p.get('position_std_m') is not None]
    consistency = dict(scope='offline heuristic envelope check, not Gaussian calibration qualification',
                       samples=len(localized), three_sigma_axis_envelope_fraction=None, normalized_max_axis_error_p95=None)
    if localized:
        difference = np.asarray([p['position'] for p in localized])-np.asarray([p['evaluator_true_position'] for p in localized])
        error = np.abs(difference)
        std = np.asarray([p['position_std_m'] for p in localized])
        normalized = np.max(error/np.maximum(std, 1e-9), axis=1)
        aligned = np.linalg.norm(difference-difference[0], axis=1)
        consistency.update(three_sigma_axis_envelope_fraction=float(np.mean(normalized <= 3.)),
                           normalized_max_axis_error_p95=float(np.percentile(normalized, 95)),
                           initial_translation_offset_m=difference[0].tolist(),
                           translation_aligned_position_rmse_m=float(np.sqrt(np.mean(aligned**2))),
                           translation_aligned_position_p95_m=float(np.percentile(aligned, 95)))
    seconds, dose = summary['simulation']['simulation_time_s'], summary['true_dose_usv']
    progress = []
    for cutoff in (90., 180., 300., 600.):
        indices = [i for i, p in enumerate(poses) if p['stamp_s'] <= cutoff]
        progress.append(dict(through_s=min(cutoff, seconds),
                             visited_xy_cells=len(set(map(tuple, cells[indices])))))
        if cutoff >= seconds:
            break
    keys = ('clean_return', 'end_state', 'error', 'true_dose_usv', 'path_length_m', 'mapped_surface_area_m2',
            'decision_reason', 'nonfoot_contacts', 'nonfoot_impulse_ns', 'nonfoot_peak_force_n',
            'localization_evaluation', 'worker_p95_ms', 'worker_phase_p95_ms', 'wall_time_s',
            'source_sha256', 'gp', 'route_forecast', 'recovery', 'terminal_stop', 'post_stop_true_dose_usv',
            'map_consistency', 'count_calibration', 'localization', 'tracking_clearance_margin_m', 'localization_margin_m')
    return dict(run=run.name, returncode=returncode, summary_sha256=digest(summary_file),
        config_sha256=digest(run/'run_config.json'), trajectory_sha256=digest(run/'localization_evaluation.jsonl'),
        configuration={k: config.get(k) for k in ('seed', 'sensor_seed', 'dose_budget', 'duration', 'rate_limit',
            'world', 'localization', 'gp_mode', 'detector_likelihood', 'fixed_plan_delay', 'navigation_profile', 'utility_mode', 'mission_objective')},
        simulated_time_s=seconds, physical_visited_xy_cells=unique, physical_visitation_cell_size_m=.5,
        physical_visitation_proxy_m2=unique*.25,
        visitation_cells_per_usv=unique/dose if dose > 0 else None,
        visitation_cells_per_minute=unique/(seconds/60.) if seconds > 0 else None,
        budget_fraction_used=dose/config['dose_budget'], visitation_progress=progress, uncertainty_consistency=consistency,
        **{k: summary.get(k) for k in keys})


def paired_report(trials):
    pairs = []
    seeds = sorted({(t['configuration']['seed'], t['configuration']['sensor_seed']) for t in trials if 'configuration' in t})
    for seed, sensor in seeds:
        matched = {t['configuration']['dose_budget']: t for t in trials if 'configuration' in t and
                   (t['configuration']['seed'], t['configuration']['sensor_seed']) == (seed, sensor)}
        if 2. not in matched or 10. not in matched:
            continue
        a, b = matched[2.], matched[10.]
        fields = ('simulated_time_s', 'true_dose_usv', 'path_length_m', 'physical_visited_xy_cells',
                  'visitation_cells_per_usv', 'visitation_cells_per_minute')
        pairs.append(dict(seed=seed, sensor_seed=sensor, both_clean=bool(a['clean_return'] and b['clean_return']),
            delta_10_minus_2={k: b[k]-a[k] if a[k] is not None and b[k] is not None else None for k in fields},
            scope='matched time cap and seed pair; unequal visited trajectories are not matched-work dose reduction'))
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--seed-pairs', nargs='+', default=['3:0', '4:1'])
    parser.add_argument('--navigation-profile', choices=('robust', 'legacy'), default='robust')
    parser.add_argument('--utility-mode', choices=('information', 'legacy'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    args.utility_mode = args.utility_mode or 'legacy'
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive and finite')
    try:
        seeds = [tuple(map(int, value.split(':'))) for value in args.seed_pairs]
        if any(len(pair) != 2 or min(pair) < 0 for pair in seeds) or len(set(seeds)) != len(seeds):
            raise ValueError()
    except ValueError:
        parser.error('seed pairs must be unique nonnegative DETECTOR:SENSOR pairs')
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('budget_5x_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    report = dict(scope='paired reactor development experiment; no reliability or complete-coverage qualification',
        duration_limit_s=args.duration, budgets_usv=[2., 10.], allowance_multiplier=5,
        rate_limit_usv_h=100., navigation_profile=args.navigation_profile, utility_mode=args.utility_mode,
        harness_sha256=digest(Path(__file__)), trials=[], pairs=[])
    for seed, sensor in seeds:
        for budget in (2., 10.):
            run = output/f'seed_{seed}_sensor_{sensor}_budget_{budget:g}'
            command = [sys.executable, '-m', 'dougie_sim.radiation_run', '--world', 'reactor', '--headless', '--fast',
                '--duration', str(args.duration), '--dose-budget', str(budget), '--rate-limit', '100',
                '--seed', str(seed), '--sensor-seed', str(sensor), '--localization', 'fused', '--gp-mode', 'geometry',
                '--navigation-profile', args.navigation_profile, '--utility-mode', args.utility_mode,
                '--mission-objective', 'survey', '--path-mode', 'smooth',
                '--detector-likelihood', 'window', '--fixed-plan-delay', '.5', '--require-clean', '--output', str(run)]
            with (output/f'{run.name}.log').open('w') as log:
                try:
                    result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=3600)
                    code = result.returncode
                except subprocess.TimeoutExpired:
                    code = 124
            record = score(run, code)
            record['command'] = command
            report['trials'].append(record)
            report['pairs'] = paired_report(report['trials'])
            payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
            (output/'summary.json').write_text(payload)
            if args.report:
                args.report.write_text(payload)
            print(json.dumps({k: record.get(k) for k in ('run', 'returncode', 'clean_return', 'simulated_time_s',
                'true_dose_usv', 'physical_visited_xy_cells', 'decision_reason', 'error')}), flush=True)
    print(f'Budget experiment evidence: {output}', flush=True)
    if not all(t.get('clean_return') and t['returncode'] == 0 for t in report['trials']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
