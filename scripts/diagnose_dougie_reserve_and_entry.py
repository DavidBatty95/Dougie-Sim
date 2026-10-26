#!/usr/bin/env python3
"""Summarize recorded reserve triggers and noisy stance-entry measurements."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation', type=Path, required=True)
    parser.add_argument('--stance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.navigation.read_text().splitlines() if line.strip()]
    triggers, previous = [], None
    for row in rows:
        reason = row.get('decision_reason')
        if reason == 'localization_return_reserve' and previous != reason:
            local = row.get('localization', {})
            triggers.append(dict(time_s=row['time_s'], position=local.get('position'),
                position_std_m=local.get('position_std_m'), conditional_position_std_m=local.get('conditional_position_std_m'),
                map_revision=local.get('map_revision'), loop_search=local.get('loop_search'),
                checkpoints=row.get('localization_checkpoints')))
        previous = reason
    data = json.loads(args.stance.read_text())
    entry = []
    for result in data['results']:
        if result['direction'] != 'down' or result['friction'] != .25 or result['force_n']:
            continue
        entry.append(dict(sensor_seed=result['sensor_seed'], stance=result['stance'],
            samples=[{k: sample.get(k) for k in ('time_s', 'minimum_foot_force_n', 'gyro_norm_rad_s',
                'maximum_joint_speed_rad_s', 'estimated_velocity')} for sample in result['trace']
                if 13. <= sample['time_s'] <= 16.01]))
    report = dict(scope=__doc__, reserve_triggers=triggers, low_traction_downhill_entry=entry,
        final_record={key: rows[-1].get(key) for key in ('time_s', 'state', 'decision_reason', 'terminal_stop', 'localization')},
        input_sha256={key: hashlib.sha256(path.read_bytes()).hexdigest()
                      for key, path in (('navigation', args.navigation), ('stance', args.stance))})
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
