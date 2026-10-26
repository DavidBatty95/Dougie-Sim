#!/usr/bin/env python3
"""Audit the complete declared stance disturbance cohort and retain all failures."""
import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.report.read_bytes()
    data = json.loads(raw)
    rows, protocol = data['results'], data['protocol']
    seeds = protocol.get('sensor_seeds', [0, 1])
    assert seeds and len(seeds) == len(set(seeds)) and all(isinstance(seed, int) and seed >= 0 for seed in seeds)
    expected = {(angle, direction, friction, force, seed)
        for (angle, direction), friction, force, seed in itertools.product(
            ((0., 'up'), (8.53, 'up'), (8.53, 'down')), (.4, .25), (0., 15.), seeds)}
    key = lambda row: tuple(row[name] for name in ('angle_deg', 'direction', 'friction', 'force_n', 'sensor_seed'))
    assert len(rows) == data['expected_cases'] == len(expected)
    assert {key(row) for row in rows} == expected, 'missing/duplicate disturbance cases'
    assert protocol['drift_limit_m'] == .05
    assert protocol['nonfoot_impulse_limit_ns'] == .1 and protocol['nonfoot_peak_force_limit_n'] == 20.
    for row in rows:
        assert row['error'] is None, 'incomplete simulation must not count as a completed screen'
        assert row['simulated_time_s'] >= protocol['stop_s']+protocol['post_stop_duration_s']
        assert all(math.isfinite(row[name]) and row[name] >= 0 for name in
            ('max_horizontal_drift_m', 'max_post_push_drift_m', 'nonfoot_impulse_ns',
             'nonfoot_peak_force_n', 'peak_motor_torque_nm', 'applied_impulse_ns'))
        assert row['contact_friction_range'] and all(abs(v-row['friction']) < 1e-9 for v in row['contact_friction_range'])
        assert abs(row['applied_impulse_ns']-row['force_n']*protocol['push_duration_s']) < .08
        assert row['peak_motor_torque_nm'] <= 23.5+1e-6
        if 'brake_seconds' in protocol:
            assert row['brake_seconds'] == protocol['brake_seconds']
            assert 0 <= row['brake_seconds'] <= 3.
            assert all(sample['command'] == [0., 0., 0.] for sample in row['trace']
                       if sample['time_s'] >= protocol['stop_s']+.03)
        if 'settling_yaw_gain' in protocol:
            assert row['settling_yaw_gain'] == protocol['settling_yaw_gain']
            assert 0 <= row['settling_yaw_gain'] <= 2.
            assert all(abs(sample['command'][2]) <= .2+1e-12 for sample in row['trace'])
        failures = []
        if row['stance'] is None or row['stance']['phase'] != 'holding':
            failures.append('stance_not_retained')
        if row['nonfoot_impulse_ns'] > .1 or row['nonfoot_peak_force_n'] > 20.:
            failures.append('nonfoot_contact')
        if row['max_horizontal_drift_m'] > .05:
            failures.append('drift_above_5cm')
        assert row['failures'] == failures and row['passed'] == (not failures)
    # Every disturbance has an otherwise identical quiet control. The fixture
    # and all sampled physical positions before the push must agree exactly.
    lookup = {key(row): row for row in rows}
    for row in rows:
        if not row['force_n']:
            continue
        angle, direction, friction, _, seed = key(row)
        control = lookup[angle, direction, friction, 0., seed]
        assert row['scene_sha256'] == control['scene_sha256']
        before = lambda r: [(sample['time_s'], sample['evaluator_position']) for sample in r['trace']
                            if sample['time_s'] < protocol['push_start_s']]
        assert before(row) == before(control), 'unmatched pre-push physics'
    groups = []
    for friction, force in itertools.product((.4, .25), (0., 15.)):
        group = [row for row in rows if row['friction'] == friction and row['force_n'] == force]
        groups.append(dict(friction=friction, force_n=force, cases=len(group), passed=sum(row['passed'] for row in group),
            failure_counts=dict(Counter(failure for row in group for failure in row['failures'])),
            maximum_horizontal_drift_m=max(row['max_horizontal_drift_m'] for row in group)))
    result = dict(scope='complete matched simulation screen; no reliability or mission qualification',
        report_sha256=hashlib.sha256(raw).hexdigest(), cases=len(rows),
        brake_seconds=protocol.get('brake_seconds', 0.), settling_yaw_gain=protocol.get('settling_yaw_gain', 0.),
        sensor_seeds=seeds, groups=groups,
        failed_cases=[{name: row[name] for name in ('angle_deg', 'direction', 'friction', 'force_n',
            'sensor_seed', 'max_horizontal_drift_m', 'stance', 'failures')} for row in rows if not row['passed']],
        verified=['cohort completeness', 'actual contact friction', 'applied impulse', 'motor torque limit',
                  'full-duration scoring', 'matched pre-push physical samples', 'failure scoring'])
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
