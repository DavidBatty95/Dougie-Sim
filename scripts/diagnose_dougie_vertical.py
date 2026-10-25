#!/usr/bin/env python3
"""Summarize recorded onboard vertical decisions; no simulator truth enters planning."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def diagnose(run):
    raw = (run/'trace.jsonl').read_bytes()
    rows = [json.loads(line) for line in raw.splitlines() if line.strip()]
    decisions = []
    previous = None
    for row in rows:
        target = row.get('selected_target')
        signature = (row.get('decision_reason'), None if target is None else tuple(round(v, 2) for v in target))
        if signature != previous:
            decisions.append({key: row.get(key) for key in ('time_s', 'state', 'decision_reason',
                'selected_target', 'sampled_support_height_bands_m', 'rejection_counts', 'route_forecast', 'vertical_frontier')})
            previous = signature
    return dict(run=run.name, trace_sha256=hashlib.sha256(raw).hexdigest(),
        reason_samples=dict(Counter(row.get('decision_reason') for row in rows)),
        maximum_selected_height_m=max((row['selected_target'][2] for row in rows if row.get('selected_target')), default=None),
        height_diagnostics=dict(
            preemption_samples=sum(bool(row.get('vertical_frontier', {}).get('commitment_preempted')) for row in rows),
            reachable_new_height_samples=sum(row.get('vertical_frontier', {}).get('reachable_new_height_nodes', 0) > 0 for row in rows),
            maximum_observed_height_m=max((row['vertical_frontier']['maximum_observed_height_m']
                for row in rows if row.get('vertical_frontier')), default=None),
            maximum_reachable_height_m=max((row['vertical_frontier']['maximum_reachable_height_m']
                for row in rows if row.get('vertical_frontier')), default=None)),
        final_sampled_height_bands_m=rows[-1].get('sampled_support_height_bands_m'), decisions=decisions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = dict(scope=__doc__, runs=[diagnose(run) for run in args.runs])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for run in report['runs']:
        print(json.dumps({key: value for key, value in run.items() if key != 'decisions'}, indent=2))


if __name__ == '__main__':
    main()
