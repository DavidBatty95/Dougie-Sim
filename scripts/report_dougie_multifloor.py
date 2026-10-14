#!/usr/bin/env python3
"""Export compact, source-attributed multi-level development evidence."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--training', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if len({p.resolve() for p in args.runs}) != len(args.runs):
        parser.error('duplicate run directories')
    report = dict(schema_version=1, scope='development runs, not a held-out qualification study', missions=[])
    keys = ('seed', 'world', 'end_state', 'error', 'true_dose_usv', 'budget_usv', 'clean_return',
            'path_length_m', 'sampled_height_range_m', 'physical_witness', 'mapped_surface_area_m2',
            'nonfoot_impulse_ns', 'exploration_complete', 'source_sha256')
    for directory in args.runs:
        path = directory/'radiation_summary.json'
        raw = path.read_bytes()
        summary = json.loads(raw)
        config = json.loads((directory/'run_config.json').read_text())
        report['missions'].append(dict(run=directory.name, summary_sha256=hashlib.sha256(raw).hexdigest(),
            **{key: summary.get(key) for key in keys}, policy=summary['simulation']['policy'],
            runtime={key: config.get(key) for key in ('fast', 'ros', 'fixed_plan_delay')},
            performance=summary['performance'],
            simulated_time_s=summary['simulation']['simulation_time_s']))
    report['training'] = dict(run=args.training.name,
        result=json.loads((args.training/'result.json').read_text()),
        policy=json.loads((args.training/'policy.json').read_text()))
    qualification = json.loads((args.qualification/'summary.json').read_text())
    if qualification['policy']['sha256'] != report['training']['policy']['sha256']:
        parser.error('qualification policy does not match the supplied training export')
    report['qualification'] = dict(run=args.qualification.name, policy=qualification['policy'],
        completed_trials=qualification['completed_trials'], complete=qualification['complete'],
        cases=qualification['cases_summary'])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
