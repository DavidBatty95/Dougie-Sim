#!/usr/bin/env python3
"""Summarize each world/count cohort without dropping invalid attempts."""
import argparse
from collections import Counter, defaultdict
import json
import re
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


def cohort_summary(dataset):
    records = [r for r in dataset['records'] if r.get('role') == 'scored']
    return dict(
        attempted=len(records),
        passed=sum(r['result'].get('valid_trial') is True and r['result'].get('success') is True and
                   r.get('execution',{}).get('return_code',0) == 0 for r in records),
        invalid=sum(r['result'].get('valid_trial') is not True for r in records),
        goals_reached=sum(r['result'].get('goals_reached',0) for r in records),
        goals_requested=sum(r['result'].get('goals_requested',0) for r in records),
        reported_contacts=sum(r['result'].get('collisions',0) for r in records))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--cohort-table',action='store_true',help='Summarize all versioned datasets in a directory as Markdown')
    parser.add_argument('--minimum-version',type=int,default=0)
    args = parser.parse_args()
    if args.cohort_table:
        paths = list(args.dataset.glob('avoidance_*.json')) if args.dataset.is_dir() else [args.dataset]
        def version(path):
            match = re.search(r'_v(\d+)(?:_|\.)',path.name)
            return int(match[1]) if match else 0
        print('| Dataset | Strict passes | Goals | Reported contacts | Invalid | Startup |')
        print('| --- | --- | --- | --- | --- | --- |')
        for path in sorted(paths,key=lambda p:(version(p),p.name)):
            if version(path) < args.minimum_version:
                continue
            dataset = json.loads(path.read_text())
            row = cohort_summary(dataset)
            startup = 'gated' if dataset.get('provenance',{}).get('synchronized_start') else 'standard'
            print(f"| {path.name} | {row['passed']}/{row['attempted']} | {row['goals_reached']}/{row['goals_requested']} | {row['reported_contacts']} | {row['invalid']} | {startup} |")
    else:
        print(json.dumps(summarize(json.loads(args.dataset.read_text())), indent=2))
