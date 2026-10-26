#!/usr/bin/env python3
"""Audit bounded correction histories and matched pre-capacity physical samples."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    evidence, traces, events, configs = [], [], [], []
    for run in (args.baseline, args.candidate):
        config = json.loads((run/'run_config.json').read_text())
        summary = json.loads((run/'radiation_summary.json').read_text())
        corrections = [json.loads(line) for line in (run/'map_corrections.jsonl').read_text().splitlines() if line.strip()]
        capacity = config.get('loop_correction_capacity', 16)
        local = summary['localization']
        assert capacity in (16, 32) and len(corrections) <= capacity
        assert [event['revision'] for event in corrections] == list(range(1, len(corrections)+1))
        assert local['map_revision'] == len(corrections) == local['pose_graph']['accepted_loops']
        assert local['pose_graph']['nodes'] <= local['pose_graph']['capacity'] == 512
        with np.load(run/'motion_trace.npz', allow_pickle=False) as archive:
            rows = archive['rows']
        assert np.isfinite(rows).all() and np.all(np.diff(rows[:, 0]) > 0)
        configs.append(config)
        traces.append(rows)
        events.append(corrections)
        evidence.append(dict(run=run.name, capacity=capacity, corrections=len(corrections),
            graph_nodes=local['pose_graph']['nodes'], clean_return=summary['clean_return'],
            strict_round_trip_completed=summary['round_trip_completed'],
            input_sha256={name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
                ('run_config.json', 'radiation_summary.json', 'map_corrections.jsonl', 'motion_trace.npz')}))
    exclude = {'output', 'source_sha256', 'loop_correction_capacity'}
    assert {k: v for k, v in configs[0].items() if k not in exclude} == {k: v for k, v in configs[1].items() if k not in exclude}
    assert evidence[0]['capacity'] == 16 and evidence[1]['capacity'] == 32
    assert len(events[0]) == 16, 'comparator did not reach its declared correction limit'
    boundary = events[0][-1]['end_s']
    before = [rows[rows[:, 0] <= boundary] for rows in traces]
    assert len(before[0]) and np.array_equal(before[0], before[1]), 'pre-capacity physical traces differ'
    count = min(map(len, traces))
    different = np.flatnonzero(np.any(traces[0][:count] != traces[1][:count], axis=1))
    report = dict(scope=__doc__, runs=evidence, matched_pre_capacity_samples=len(before[0]),
        baseline_capacity_reached_s=boundary,
        first_physical_difference_s=None if not len(different) else float(traces[0][different[0], 0]),
        extra_corrections=max(0, len(events[1])-16),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
