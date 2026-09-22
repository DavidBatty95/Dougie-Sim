#!/usr/bin/env python3
"""Summarize each world/count cohort without dropping invalid attempts."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path


def summarize(dataset):
    groups = defaultdict(list)
    for record in dataset['records']:
        if record.get('role') != 'scored':
            continue
        groups[(record.get('world', 'unspecified'), record['result'].get('people_count'))].append(record)
    summaries = []
    for (world, people), records in sorted(groups.items()):
        valid = [r for r in records if r['result'].get('valid_trial') is True]
        passed = [r for r in valid if r['result'].get('success') is True and r.get('execution', {}).get('return_code', 0) == 0]
        summaries.append(dict(world=world, people=people, attempted=len(records), valid=len(valid),
                             passed=len(passed), goals_reached=sum(r['result'].get('goals_reached', 0) for r in valid),
                             goals_requested=sum(r['result'].get('goals_requested', 0) for r in valid),
                             contacts=sum(r['result'].get('collisions', 0) for r in valid),
                             failure_reasons=dict(Counter(r['result'].get('failure_reason') or r['result'].get('invalid_reason') or
                                 ('validator_rejected' if r.get('execution', {}).get('return_code') not in (None, 0) else 'none') for r in records))))
    return summaries


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(json.loads(args.dataset.read_text())), indent=2))
