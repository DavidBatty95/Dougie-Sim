#!/usr/bin/env python3
"""Audit a matched failed-mission terminal-hold comparison without calling it recovery."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from scripts.report_dougie_navigation import terminal_motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--gravity', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        parser.error('audit output must be new')
    runs = [args.baseline, args.gravity]
    summaries = [json.loads((run/'radiation_summary.json').read_text()) for run in runs]
    configs = [json.loads((run/'run_config.json').read_text()) for run in runs]
    excluded = {'output', 'record_demo', 'record_view', 'terminal_hold'}
    assert {k: v for k, v in configs[0].items() if k not in excluded} == {k: v for k, v in configs[1].items() if k not in excluded}
    assert [c['terminal_hold'] for c in configs] == ['zero-policy', 'gravity']
    assert all(s['end_state'] == 'stopped_unrecoverable' and s['round_trip_completed'] is False for s in summaries)
    assert summaries[0]['terminal_stop'] == summaries[1]['terminal_stop']
    stop = summaries[0]['terminal_stop']['time_s']
    prefixes = []
    for run in runs:
        with np.load(run/'motion_trace.npz', allow_pickle=False) as archive:
            rows = archive['rows']
            prefixes.append(rows[rows[:, 0] <= stop])
    assert len(prefixes[0]) > 0
    np.testing.assert_array_equal(prefixes[0], prefixes[1])
    gravity = summaries[1]
    assert gravity['error'] is None and not gravity['actual_budget_exceeded']
    assert gravity['nonfoot_impulse_ns'] <= .1 and gravity['nonfoot_peak_force_n'] <= 25.
    assert gravity['simulation']['simulation_time_s'] >= configs[1]['duration']
    assert gravity['simulation']['terminal_stance']['phase'] == 'holding'
    report = dict(scope='identical pre-stop physical trajectory; terminal posture comparison, both vertical missions failed',
        matched_pre_stop_samples=len(prefixes[0]), terminal_stop_s=stop,
        audit_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), trials=[])
    for run, summary in zip(runs, summaries):
        report['trials'].append(dict(run=run.name, post_stop_motion=terminal_motion(run, summary),
            input_sha256={name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                          for name in ('run_config.json', 'radiation_summary.json', 'motion_trace.npz')},
            end_s=summary['simulation']['simulation_time_s'], true_dose_usv=summary['true_dose_usv'],
            nonfoot_impulse_ns=summary['nonfoot_impulse_ns'], round_trip_failures=summary['round_trip_failures']))
    if args.output:
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
