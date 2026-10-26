#!/usr/bin/env python3
"""Compare complete matched planned-stop cohorts, preserving gains and regressions."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, candidate = [json.loads(path.read_text()) for path in (args.baseline, args.candidate)]
    for key in ('stop_s', 'approach_s', 'push_start_s', 'push_duration_s', 'post_stop_duration_s',
                'drift_limit_m', 'nonfoot_impulse_limit_ns', 'nonfoot_peak_force_limit_n', 'force_axis', 'controller'):
        assert baseline['protocol'][key] == candidate['protocol'][key], f'unmatched protocol: {key}'
    assert baseline['source_sha256'] == candidate['source_sha256'], 'runtime differs across the braking ablation'
    key = lambda row: tuple(row[name] for name in ('angle_deg', 'direction', 'friction', 'force_n', 'sensor_seed'))
    controls = {key(row): row for row in baseline['results']}
    trials = {key(row): row for row in candidate['results']}
    assert len(controls) == len(baseline['results']) == baseline['expected_cases']
    assert len(trials) == len(candidate['results']) == candidate['expected_cases']
    assert controls.keys() == trials.keys()
    prefix_end = baseline['protocol']['stop_s']-max(baseline['protocol'].get('brake_seconds', 0.),
                                                 candidate['protocol'].get('brake_seconds', 0.))-.03
    pairs = []
    for identity, trial in trials.items():
        control = controls[identity]
        assert control['scene_sha256'] == trial['scene_sha256'] and control['policy'] == trial['policy']
        prefix = lambda row: [(sample['time_s'], sample['evaluator_position']) for sample in row['trace']
                              if sample['time_s'] < prefix_end]
        assert prefix(control) == prefix(trial) and prefix(trial), 'unmatched pre-braking physical samples'
        pairs.append(dict(case=identity, baseline_passed=control['passed'], candidate_passed=trial['passed'],
            baseline_drift_m=control['max_horizontal_drift_m'], candidate_drift_m=trial['max_horizontal_drift_m'],
            candidate_failures=trial['failures'], matched_prefix_samples=len(prefix(trial))))
    report = dict(scope='matched planned-stop simulation comparison; not emergency braking or reliability qualification',
        input_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest()
                      for name, path in (('baseline', args.baseline), ('candidate', args.candidate))},
        cases=len(pairs), baseline_passes=sum(pair['baseline_passed'] for pair in pairs),
        candidate_passes=sum(pair['candidate_passed'] for pair in pairs),
        recovered_cases=sum(not pair['baseline_passed'] and pair['candidate_passed'] for pair in pairs),
        regressed_cases=sum(pair['baseline_passed'] and not pair['candidate_passed'] for pair in pairs), pairs=pairs)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'pairs'}, indent=2))


if __name__ == '__main__':
    main()
