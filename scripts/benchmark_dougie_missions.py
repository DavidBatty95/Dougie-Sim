#!/usr/bin/env python3
"""Run an explicit detector/layout seed matrix, retaining logs and failed trials."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from dougie_sim.assets import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int, required=True)
    parser.add_argument('--layout-seeds', nargs='+', type=int, required=True)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--landmark-count', type=int, default=8)
    parser.add_argument('--landmark-seed', type=int, default=7)
    parser.add_argument('--mission-objective', choices=('survey', 'inspect'), default='inspect')
    parser.add_argument('--require-round-trip', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive and finite')
    if any(seed < 0 for seed in args.seeds+args.layout_seeds) or len(set(args.seeds)) != len(args.seeds) or len(set(args.layout_seeds)) != len(args.layout_seeds):
        parser.error('seeds must be unique nonnegative integers')
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('mission_matrix_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    report = dict(scope='development seed matrix; clean return is separate from landmark and vertical completion',
        gate='round_trip' if args.require_round_trip else 'clean_return',
        arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), trials=[])
    for layout in args.layout_seeds:
        for seed in args.seeds:
            run = output/f'layout_{layout}_seed_{seed}'
            command = [sys.executable, '-m', 'dougie_sim.radiation_run', '--world', 'facility', '--headless', '--fast',
                '--duration', str(args.duration), '--dose-budget', '5', '--gp-mode', 'spatial', '--fixed-plan-delay', '.5',
                '--seed', str(seed), '--layout-seed', str(layout), '--landmark-count', str(args.landmark_count),
                '--landmark-seed', str(args.landmark_seed), '--mission-objective', args.mission_objective,
                '--require-round-trip' if args.require_round_trip else '--require-clean', '--output', str(run)]
            with (output/f'{run.name}.log').open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            summary_file = run/'radiation_summary.json'
            summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
            keys = ('clean_return', 'round_trip_completed', 'landmark_mission_completed', 'landmarks',
                    'true_dose_usv', 'path_length_m', 'error', 'physical_witness', 'worker_phase_p95_ms')
            record = dict(run=str(run), seed=seed, layout_seed=layout, returncode=result.returncode,
                passed_required_gate=result.returncode == 0 and summary.get('round_trip_completed' if args.require_round_trip else 'clean_return') is True,
                summary_sha256=hashlib.sha256(summary_file.read_bytes()).hexdigest() if summary_file.exists() else None,
                simulated_time_s=summary.get('simulation', {}).get('simulation_time_s'),
                **{key: summary.get(key) for key in keys})
            report['trials'].append(record)
            (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(record), flush=True)
    print(f'Matrix evidence: {output}', flush=True)
    if not all(trial['passed_required_gate'] for trial in report['trials']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
