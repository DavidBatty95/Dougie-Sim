#!/usr/bin/env python3
"""Evaluator-only sampled intermediate-landing witness; never a full-height pass."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from dougie_sim.simulation import Simulation


def check(run):
    config = json.loads((run/'run_config.json').read_text())
    if config['world'] != 'facility':
        raise ValueError('this evaluator requires the facility fixture')
    summary = json.loads((run/'radiation_summary.json').read_text())
    with np.load(run/'motion_trace.npz', allow_pickle=False) as archive:
        rows = archive['rows']
    if rows.ndim != 2 or rows.shape[1] < 20 or not np.isfinite(rows[:, :20]).all() or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('invalid physical pose trace')
    sim = Simulation(controller='stand', model_xml=(run/'scene.xml').read_text())
    landing = sim.model.geom('intermediate_landing').id
    feet = [sim.model.body(leg+'_foot').id for leg in ('FR', 'FL', 'RR', 'RL')]
    start = witness = previous = None
    longest = 0.
    for row in rows:
        stamp = float(row[0])
        sim.data.qpos[:] = row[1:20]
        mujoco.mj_forward(sim.model, sim.data)
        relative = (sim.data.xpos[feet]-sim.data.geom_xpos[landing])@sim.data.geom_xmat[landing].reshape(3, 3)
        half = sim.model.geom_size[landing]
        on = bool(np.all(np.abs(relative[:, :2]) < half[:2]-.05)
                  and np.all(np.abs(relative[:, 2]-.022-half[2]) < .12))
        if on:
            if start is None or previous is None or stamp-previous > .051:
                start = stamp
            longest = max(longest, stamp-start)
            if witness is None and stamp-start >= .2-1e-9:
                witness = stamp
        else:
            start = None
        previous = stamp
    return dict(run=run.name, sampled_intermediate_witness_s=witness,
        longest_sampled_dwell_s=longest, maximum_sampled_base_height_m=float(rows[:, 3].max()),
        clean_return=summary['clean_return'], strict_round_trip_completed=summary['round_trip_completed'],
        input_sha256={name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                      for name in ('scene.xml', 'motion_trace.npz', 'run_config.json', 'radiation_summary.json')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = dict(scope=__doc__, sampling='20 Hz foot-pose replay, >=.2 s sampled dwell; no contact-force witness',
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), runs=[check(run) for run in args.runs])
    if len(args.runs) == 2:
        with np.load(args.runs[0]/'motion_trace.npz', allow_pickle=False) as a, np.load(args.runs[1]/'motion_trace.npz', allow_pickle=False) as b:
            count = min(len(a['rows']), len(b['rows']))
            same = np.all(a['rows'][:count] == b['rows'][:count], axis=1)
            different = np.flatnonzero(~same)
            report['shared_physical_prefix'] = dict(compared_samples=count, all_equal=bool(same.all()),
                first_difference_s=None if not len(different) else float(a['rows'][different[0], 0]))
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
