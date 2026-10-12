#!/usr/bin/env python3
"""Matched native-physics planner comparisons, retaining failures and source IDs."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def run_job(job):
    planner, profile, seed, duration, budget, terrain_profile, root = job
    output = root/f'{profile}_{planner}_seed{seed}'
    output.mkdir(parents=True, exist_ok=False)
    with tarfile.open(output/'source_snapshot.tar.gz', 'w:gz') as archive:
        for path in [*sorted((ROOT/'dougie_sim').glob('*.py')), ROOT/'config/dougie_assets.json', ROOT/'config/dougie_radiation_sources.json']:
            archive.add(path, arcname=str(path.relative_to(ROOT)))
    command = [sys.executable, '-m', 'dougie_sim.radiation_run', '--headless', '--fast',
        '--fixed-plan-delay', '.5', '--planner', planner, '--source-profile', profile,
        '--seed', str(seed), '--duration', str(duration), '--dose-budget', str(budget), '--terrain-profile', terrain_profile, '--output', str(output)]
    with (output/'run.log').open('w') as log:
        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=600)
    path = output/'radiation_summary.json'
    summary = json.loads(path.read_text()) if path.exists() else dict(error='missing summary', mission_completed=False)
    return dict(planner=planner, source_profile=profile, seed=seed, exit_code=result.returncode,
        run=str(output.relative_to(ROOT)), summary_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None,
        **{key: summary.get(key) for key in ('error', 'mission_completed', 'clean_return', 'true_dose_usv',
            'actual_budget_exceeded', 'mapped_surface_area_m2', 'observed_free_volume_m3', 'planning_p95_ms',
            'path_length_m', 'target_switches', 'nonfoot_impulse_ns', 'nonfoot_peak_force_n', 'source_sha256')},
        simulated_seconds=summary.get('simulation', {}).get('simulation_time_s'),
        gp_updates=summary.get('gp', {}).get('updates'),
        gp_inducing_points=summary.get('gp', {}).get('inducing_points'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planners', nargs='+', choices=('baseline', 'clustered', 'clustered-no-commit'), default=['baseline', 'clustered'])
    parser.add_argument('--profiles', nargs='+', default=['baseline', 'hotspots-and-leaks'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[11, 12, 13])
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--budget', type=float, default=2.)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--terrain-profile', choices=('conservative', 'experimental'), default='conservative')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    jobs = [(planner, profile, seed, args.duration, args.budget, args.terrain_profile, output)
            for profile in args.profiles for seed in args.seeds for planner in args.planners]
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_job, job) for job in jobs]):
            row = future.result()
            rows.append(row)
            print(f"{len(rows)}/{len(jobs)} {row['source_profile']} {row['planner']} seed={row['seed']}: return={row['mission_completed']} clean={row['clean_return']} area={row['mapped_surface_area_m2']} dose={row['true_dose_usv']}", flush=True)
            (output/'partial.json').write_text(json.dumps(rows, indent=2)+'\n')
    groups = []
    for profile in args.profiles:
        for planner in args.planners:
            selected = [row for row in rows if row['planner'] == planner and row['source_profile'] == profile]
            group = dict(profile=profile, planner=planner, runs=len(selected),
                returned=sum(bool(row['mission_completed']) for row in selected),
                clean_returns=sum(bool(row['clean_return']) for row in selected),
                process_failures=sum(row['exit_code'] != 0 for row in selected),
                incomplete_missions=sum(not row['mission_completed'] for row in selected))
            for metric in ('mapped_surface_area_m2', 'observed_free_volume_m3', 'true_dose_usv', 'planning_p95_ms', 'path_length_m', 'target_switches'):
                values = [row[metric] for row in selected if row[metric] is not None]
                group['median_'+metric] = float(np.median(values)) if values else None
            groups.append(group)
    report = dict(protocol=dict(duration_s=args.duration, budget_usv=args.budget, seeds=args.seeds,
        fixed_simulated_planning_delay_s=.5, scene='reactor', workers=args.workers, terrain_profile=args.terrain_profile,
        note='Same policy, scene, sensing, dose constraints; compare complete outcomes and failed runs. CPU timings include concurrent processes.'),
        groups=groups, runs=rows)
    (output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(groups, indent=2))


if __name__ == '__main__':
    main()
