#!/usr/bin/env python3
"""Predeclare independent calibration/validation episodes, then fit and audit.

All trials are retained. Mission failures are reported independently of pose
coverage; invalid or externally interrupted calibration episodes prevent fitting.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import numpy as np

from dougie_sim.assets import ROOT
from dougie_sim.pose_uncertainty import quantile_rank
from scripts.calibrate_dougie_uncertainty import episode, calibrate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='mezzanine')
    parser.add_argument('--mezzanine-angle', type=float, default=10.)
    parser.add_argument('--duration', type=float, default=300.)
    parser.add_argument('--dose-budget', type=float, default=10.)
    parser.add_argument('--calibration-episodes', type=int, default=9)
    parser.add_argument('--validation-episodes', type=int, default=3)
    parser.add_argument('--alpha', type=float, default=.1)
    parser.add_argument('--protocol-seed', type=int, default=20261021)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--require-round-trip', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if (args.validation_episodes < 1 or not 1 <= args.workers <= 4
            or not np.isfinite([args.duration, args.dose_budget]).all() or min(args.duration, args.dose_budget) <= 0):
        parser.error('invalid episode, worker or mission settings')
    try:
        if quantile_rank(args.calibration_episodes, args.alpha) > args.calibration_episodes:
            parser.error('too few independent calibration episodes for requested coverage')
    except ValueError as exc:
        parser.error(str(exc))
    if args.protocol_seed < 0:
        parser.error('protocol seed must be nonnegative')
    if args.require_round_trip and args.world == 'reactor':
        parser.error('vertical qualification requires a multi-floor world')
    if args.profile.exists() or args.report.exists():
        parser.error('profile/report paths must be new; existing evidence is never overwritten')
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('uncertainty_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    n = args.calibration_episodes+args.validation_episodes
    rng = np.random.default_rng(args.protocol_seed)
    sensor_seeds = rng.choice(1000000, n+1, replace=False)
    detector_seeds = rng.choice(1000000, n+1, replace=False)
    cases = []
    for i, (sensor, detector) in enumerate(zip(sensor_seeds, detector_seeds)):
        role = 'calibration' if i < args.calibration_episodes else 'validation' if i < n else 'application'
        run = output/f'{role}_{i:02d}'
        command = [sys.executable, '-m', 'dougie_sim.radiation_run', '--headless', '--fast', '--world', args.world,
            '--duration', str(args.duration), '--dose-budget', str(args.dose_budget), '--seed', str(detector),
            '--sensor-seed', str(sensor), '--navigation-profile', 'robust', '--slam-backend', 'pose-graph',
            '--gp-mode', 'geometry', '--detector-likelihood', 'window', '--utility-mode', 'legacy',
            '--fixed-plan-delay', '.5', '--require-clean', '--output', str(run)]
        if args.require_round_trip:
            command.append('--require-round-trip')
        if args.world == 'mezzanine':
            command += ['--mezzanine-angle', str(args.mezzanine_angle)]
        if role == 'application':
            command += ['--uncertainty-profile', str(args.profile)]
        cases.append(dict(index=i, role=role, run=str(run), detector_seed=int(detector), sensor_seed=int(sensor), command=command))
    protocol = dict(arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}, cases=cases,
                    harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    report = dict(protocol=protocol, trials=[], calibration_status='collecting')

    def trial(case):
        with (output/f"{case['role']}_{case['index']:02d}.log").open('w') as log:
            process = subprocess.Popen(case['command'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=max(600., args.duration*8.))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                code = 124
        summary_path = Path(case['run'])/'radiation_summary.json'
        def invalid_constant(value):
            raise ValueError('nonfinite summary value: '+value)
        invalid_summary = False
        try:
            summary = json.loads(summary_path.read_text(), parse_constant=invalid_constant)
        except (OSError, ValueError) as exc:
            summary, invalid_summary = dict(error=str(exc)), True
        return dict(**case, returncode=code, clean_return=summary.get('clean_return'),
            invalid_summary=invalid_summary,
            round_trip_completed=summary.get('round_trip_completed'), error=summary.get('error'),
            true_dose_usv=summary.get('true_dose_usv'), physical_witness=summary.get('physical_witness'),
            localization_evaluation=summary.get('localization_evaluation'),
            summary_sha256=hashlib.sha256(summary_path.read_bytes()).hexdigest() if summary_path.exists() else None)

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(trial, case) for case in cases if case['role'] != 'application']
        for future in as_completed(futures):
            result = future.result()
            report['trials'].append(result)
            report['trials'].sort(key=lambda item: item['index'])
            args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps({k: result[k] for k in ('index', 'role', 'returncode', 'clean_return', 'round_trip_completed', 'true_dose_usv')}), flush=True)
    try:
        if any(t['returncode'] not in (0, 1) or t['summary_sha256'] is None or t['invalid_summary'] for t in report['trials']):
            raise ValueError('externally interrupted or missing episodes invalidate the predeclared split')
        training = [episode(Path(c['run'])) for c in cases if c['role'] == 'calibration']
        validation = [episode(Path(c['run'])) for c in cases if c['role'] == 'validation']
        profile, validation_report = calibrate(training, validation, args.alpha)
        args.profile.write_text(json.dumps(profile, indent=2, allow_nan=False)+'\n')
        report.update(calibration_status='fitted', uncertainty=validation_report)
        application_case = next(c for c in cases if c['role'] == 'application')
        applied = trial(application_case)
        report['application_check'] = applied
        if applied['summary_sha256'] is not None and not applied['invalid_summary']:
            applied_episode = episode(Path(applied['run']))
            report['application_pose_check'] = applied_episode
            report['application_envelope_covered'] = bool(applied_episode['episode_score'] is not None
                and applied_episode['episode_score'] <= profile['sigma_multiplier'])
    except (ValueError, OSError, KeyError) as exc:
        report.update(calibration_status='unavailable', calibration_error=str(exc))
    report['passed_mission_gates'] = (all(t['returncode'] == 0 for t in report['trials'])
                                    and report.get('application_check', {}).get('returncode') == 0)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(args.report), calibration_status=report['calibration_status'],
                          passed_mission_gates=report['passed_mission_gates'])), flush=True)
    if report['calibration_status'] != 'fitted' or not report['passed_mission_gates']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
