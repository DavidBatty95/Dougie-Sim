#!/usr/bin/env python3
"""Attach downsampled offline evaluation traces for browser replay."""
import argparse
import json
import math
from pathlib import Path

from demo_suite import REPO_ROOT, file_sha256
from run_avoidance_matrix import publish_dataset


def trace_from_log(path, interval=.5):
    samples, people = [], []
    start, previous = None, -math.inf
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            data = row.get('data')
            if not isinstance(data, dict):
                continue
            if row.get('topic') == 'mujoco_people':
                people = [[p['x'], p['y']] for p in data.get('people', [])]
            if row.get('topic') != 'evaluation_odom' or data.get('frame_id') != 'world':
                continue
            stamp = data.get('source_stamp', row['stamp'])
            if start is None:
                start = stamp
            if stamp-previous < interval:
                continue
            values = [stamp-start, data['x'], data['y']]
            if not all(math.isfinite(value) for value in values):
                continue
            previous = stamp
            samples.append(dict(t=round(values[0], 2), x=values[1], y=values[2],
                                people=[[round(x,3), round(y,3)] for x,y in people]))
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', type=Path, help='Publish an enriched copy instead of updating the input')
    args = parser.parse_args()
    dataset = json.loads(args.dataset.read_text())
    enriched = 0
    for record in dataset['records']:
        source = record.get('source_artifact')
        if not source:
            continue
        result = REPO_ROOT / source
        log = result.with_name(result.stem + '_human_aware.jsonl')
        if not log.exists():
            continue
        record['evaluation_trace'] = trace_from_log(log)
        record['trace_frame'] = 'world; offline scoring only'
        record['trace_time_origin'] = 'first evaluation odometry sample in the log, including startup'
        record['trace_source_sha256'] = file_sha256(log)
        enriched += 1
    output = args.output or args.dataset
    publish_dataset(output, dataset)
    print(f'Attached evaluation traces to {enriched} records in {output}')


if __name__ == '__main__':
    main()
