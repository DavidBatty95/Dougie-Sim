#!/usr/bin/env python3
"""Inspect recorded planning decisions around a measured radiation incident."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--start',type=float,required=True)
    parser.add_argument('--end',type=float,required=True)
    args = parser.parse_args()
    result = []
    with (args.run/'trace.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['time_s'] < args.start:
                continue
            if row['time_s'] > args.end:
                break
            target = row.get('selected_target')
            choices = row.get('frontier_decision',{}).get('shortlisted',[])
            matched = next((v for v in choices if target is not None and np.linalg.norm(np.array(v['position'])-target)<.05),None)
            result.append(dict(time_s=row['time_s'],state=row['state'],target=target,
                reason=row.get('decision_reason'),follower=row.get('follower_status'),
                selected_utility=row.get('selected_utility'),selected_candidate=matched))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
