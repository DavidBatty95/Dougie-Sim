#!/usr/bin/env python3
"""Paired utility comparison with identical initially observed goals and physical gates."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=240.)
    parser.add_argument('--dose-budget', type=float, default=2.)
    parser.add_argument('--radius', type=float, default=1.5)
    parser.add_argument('--seed-pairs', nargs='+', default=['3:0', '4:1'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.dose_budget)) or not 1. <= args.radius <= 3.:
        parser.error('invalid time, budget or circuit radius')
    try:
        seeds = [tuple(map(int, value.split(':'))) for value in args.seed_pairs]
        if any(len(pair) != 2 or min(pair) < 0 for pair in seeds) or len(set(seeds)) != len(seeds):
            raise ValueError()
    except ValueError:
        parser.error('seed pairs must be unique nonnegative DETECTOR:SENSOR pairs')
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('circuit_matrix_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    report = dict(scope='paired observed-goal development comparison; same sensing/fusion/recovery, different candidate utility',
        harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), trials=[], pairs=[])
    for seed, sensor in seeds:
        paired = []
        for mode in ('legacy', 'information'):
            run = output/f'seed_{seed}_sensor_{sensor}_{mode}'
            command = [sys.executable, '-m', 'dougie_sim.radiation_run', '--headless', '--fast', '--world', 'reactor',
                '--duration', str(args.duration), '--dose-budget', str(args.dose_budget), '--seed', str(seed), '--sensor-seed', str(sensor),
                '--navigation-profile', 'robust', '--utility-mode', mode, '--mission-objective', 'circuit',
                '--localization', 'fused', '--gp-mode', 'geometry', '--detector-likelihood', 'window', '--path-mode', 'smooth',
                '--circuit-radius', str(args.radius), '--require-circuit', '--fixed-plan-delay', '.5', '--output', str(run)]
            with (output/f'{run.name}.log').open('w') as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            path = run/'radiation_summary.json'
            summary = json.loads(path.read_text()) if path.exists() else {}
            fields = ('clean_return', 'circuit_mission_completed', 'circuit', 'circuit_evaluation', 'true_dose_usv',
                'path_length_m', 'localization_evaluation', 'recovery', 'terminal_stop', 'count_calibration', 'source_sha256')
            record = dict(run=run.name, seed=seed, sensor_seed=sensor, utility_mode=mode, command=command,
                returncode=result.returncode, simulated_time_s=summary.get('simulation', {}).get('simulation_time_s'),
                summary_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None,
                **{key: summary.get(key) for key in fields})
            paired.append(record)
            report['trials'].append(record)
            print(json.dumps({key: record[key] for key in ('run', 'returncode', 'circuit_mission_completed', 'true_dose_usv', 'simulated_time_s')}), flush=True)
            payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
            (output/'summary.json').write_text(payload)
            if args.report:
                args.report.write_text(payload)
        a, b = paired
        ga, gb = [(r.get('circuit') or {}).get('initial_goals', []) for r in paired]
        pa, pb = [(r.get('circuit_evaluation') or {}).get('physical_goals') for r in paired]
        matched = bool(len(ga) == len(gb) == 4 and np.allclose(ga, gb, rtol=0., atol=1e-8)
            and pa is not None and pb is not None and np.allclose(pa, pb, rtol=0., atol=1e-6))
        qualified = matched and all(r.get('circuit_mission_completed') and r['returncode'] == 0 for r in paired)
        report['pairs'].append(dict(seed=seed, sensor_seed=sensor, matched_goals=matched, both_completed=bool(qualified),
            dose_difference_usv=b['true_dose_usv']-a['true_dose_usv'] if qualified else None,
            duration_difference_s=b['simulated_time_s']-a['simulated_time_s'] if qualified else None))
        payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
        (output/'summary.json').write_text(payload)
        if args.report:
            args.report.write_text(payload)
    print(f'Circuit evidence: {output}', flush=True)
    if not all(pair['both_completed'] for pair in report['pairs']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
