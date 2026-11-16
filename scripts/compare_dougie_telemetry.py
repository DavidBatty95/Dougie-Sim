#!/usr/bin/env python3
"""Locate the first numerical/decision divergence between retained paired runs."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('left', type=Path)
    parser.add_argument('right', type=Path)
    args = parser.parse_args()
    report = {}
    for mode in ('blind', 'aware'):
        left = [json.loads(line) for line in (args.left/mode/'comparison.jsonl').read_text().splitlines()]
        right = [json.loads(line) for line in (args.right/mode/'comparison.jsonl').read_text().splitlines()]
        divergence = next((i for i,(a,b) in enumerate(zip(left,right)) if any(
            abs(a[k]-b[k]) > 1e-9 for k in ('time_s','true_dose_usv','explored_percent','distance_m'))),None)
        report[mode] = dict(compared_rows=min(len(left),len(right)), first_divergent_row=divergence,
            left=left[divergence] if divergence is not None else None,
            right=right[divergence] if divergence is not None else None)
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
