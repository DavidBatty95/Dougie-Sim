#!/usr/bin/env python3
"""Summarize retained planner decisions without printing full geometry arrays."""
import argparse
from collections import Counter
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--start', type=float, default=0.)
    parser.add_argument('--end', type=float, default=float('inf'))
    parser.add_argument('--candidates', action='store_true', help='include the eight highest-scored region representatives')
    args = parser.parse_args()
    rows = []
    with (args.run/'trace.jsonl').open() as stream:
        for line in stream:
            item = json.loads(line)
            if args.start <= item['time_s'] <= args.end:
                rows.append(item)
    changes, previous = [], None
    for item in rows:
        target = item.get('selected_target')
        signature = (item['state'], None if target is None else tuple(round(v, 2) for v in target))
        if signature != previous:
            changes.append({key: item.get(key) for key in
                ('time_s', 'state', 'selected_target', 'decision_reason', 'rejection_counts', 'blocked_edges')})
            if args.candidates:
                changes[-1]['candidates'] = item.get('clusters', [])[:8]
            previous = signature
    print(json.dumps(dict(plans=len(rows), reasons=dict(Counter(r.get('decision_reason') for r in rows)),
                         goal_changes=changes), indent=2))


if __name__ == '__main__':
    main()
