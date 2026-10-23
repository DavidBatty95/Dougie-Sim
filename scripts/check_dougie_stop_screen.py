#!/usr/bin/env python3
"""Audit complete stop cohorts and optionally require contact or dropout gates."""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path


def audit(report, require_contact_clean=False, require_range_withheld=False):
    expected = [tuple(case) for case in report['cases']]
    actual = [(r['angle_deg'], r['mode'], r['sensor_seed'], r['direction']) for r in report['results']]
    if not expected or len(set(expected)) != len(expected) or actual != expected:
        raise ValueError('missing, duplicated or unexpected predeclared cases')
    if require_range_withheld and report.get('lidar_dropout_after_s') is None:
        raise ValueError('dropout audit requires a declared dropout')
    groups, prefixes = defaultdict(list), {}
    faults_checked = 0
    for result in report['results']:
        for key in ('max_horizontal_drift_m', 'nonfoot_impulse_ns', 'nonfoot_peak_force_n', 'peak_motor_torque_nm'):
            if not math.isfinite(result[key]) or result[key] < 0:
                raise ValueError('invalid evaluator metric: '+key)
        paired = result['angle_deg'], result['sensor_seed'], result['direction']
        prefix = result['scene_sha256'], result['evaluator_approach_distance_m']
        if paired in prefixes and prefix != prefixes[paired]:
            raise ValueError('paired cases do not share fixture and physical approach')
        prefixes[paired] = prefix
        failed = (result['error'] is not None or result.get('stance_failure') is not None or result['nonfoot_impulse_ns'] > .1
                  or result['nonfoot_peak_force_n'] > 25.)
        if require_contact_clean and failed:
            raise ValueError('physical contact/error gate failed: '+str(paired))
        if require_range_withheld and result['mode'] == 'range_position_hold':
            dropout = report['lidar_dropout_after_s']
            if (result['hold_started_s'] is None or result['hold_started_s'] >= dropout
                    or result['range_inhibited_ticks'] <= 0
                    or result['max_inhibited_requested_command_m_s'] != 0.):
                raise ValueError('range hold did not engage before dropout and then withhold corrections')
            after = [row for row in result['trace'] if row['time_s'] >= dropout+.05]
            if not after or any(row['range_feedback_ready'] or any(abs(v) > 1e-12 for v in row['command']) for row in after):
                raise ValueError('post-dropout trace does not show withheld commands')
            faults_checked += 1
        groups[result['angle_deg'], result['direction'], result['mode']].append((result, failed))
    if require_range_withheld and faults_checked == 0:
        raise ValueError('no range-hold fault cases were audited')
    return [dict(angle_deg=key[0], direction=key[1], mode=key[2], cases=len(values),
        drift_m=[min(r['max_horizontal_drift_m'] for r, _ in values), max(r['max_horizontal_drift_m'] for r, _ in values)],
        estimated_drift_m=None if any('max_estimated_horizontal_drift_m' not in r for r, _ in values) else
            [min(r['max_estimated_horizontal_drift_m'] for r, _ in values),
             max(r['max_estimated_horizontal_drift_m'] for r, _ in values)],
        range_inhibited_ticks=sum(r.get('range_inhibited_ticks', 0) for r, _ in values),
        contact_or_error_failures=sum(failed for _, failed in values),
        entry_failures=sum(r['hold_entry_failed'] for r, _ in values),
        stance_refusals=sum(r.get('stance_failure') is not None for r, _ in values),
        anchor_corrections=sum(r.get('anchor_corrections', 0) for r, _ in values))
        for key, values in groups.items()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--require-contact-clean', action='store_true')
    parser.add_argument('--require-range-withheld', action='store_true')
    args = parser.parse_args()
    result = audit(json.loads(args.report.read_text()), args.require_contact_clean, args.require_range_withheld)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
