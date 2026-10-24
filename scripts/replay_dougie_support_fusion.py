#!/usr/bin/env python3
"""Compare support estimators on identical archived observations.

First-failure captures retain that decision's frame; final captures include later
SLAM corrections. This is a geometry ablation, not an executable radiation-safe route.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.layered_terrain import LayeredSurfaceMap
from dougie_sim.pose_uncertainty import core_fingerprint
from scripts.diagnose_dougie_support import connectivity


def replay(path):
    with np.load(path, allow_pickle=False) as archive:
        data = dict(archive)
    if int(data['schema_version']) != 1 or bool(data['journal_saturated']) or not len(data['stamps_s']):
        raise ValueError('support replay needs a complete, nonempty observation journal')
    if str(data['capture_scope']) not in ('first_support_failure', 'final'):
        raise ValueError('unknown support capture scope')
    if not np.isfinite(data['stamps_s']).all() or np.any(np.diff(data['stamps_s']) <= 0):
        raise ValueError('support captures must be strictly ordered')
    count = len(data['stamps_s'])
    if data['origins'].shape != (count, 3) or not np.isfinite(data['origins']).all():
        raise ValueError('invalid estimated scan origins')
    for offsets, points in (('point_offsets', 'points'), ('support_offsets', 'support_points')):
        values = data[offsets]
        if (len(values) != count+1 or values[0] != 0 or values[-1] != len(data[points])
                or np.any(np.diff(values) < 0) or data[points].ndim != 2 or data[points].shape[1] != 3
                or not np.isfinite(data[points]).all()):
            raise ValueError('invalid support capture boundaries')
    if (data['home'].shape != (3,) or data['last_position'].shape != (3,)
            or not np.isfinite(np.r_[data['home'], data['last_position']]).all()):
        raise ValueError('replay needs estimated home and last support position')
    results = []
    for mode in ('maximum', 'bounded-mean'):
        terrain = LayeredSurfaceMap(resolution=float(data['resolution']), radius=float(data['base_radius']),
            max_slope_deg=float(data['max_slope_deg']), residual_limit=float(data['residual_limit']),
            support_capacity=int(data['support_capacity']), support_fusion=mode)
        if terrain.fusion_span_m != float(data['fusion_span_m']):
            raise ValueError('reconstruction fusion span differs from captured runtime')
        terrain.set_localization_margin(float(data['localization_margin']))
        for i in range(count):
            p, q = data['point_offsets'][i:i+2]
            a, b = data['support_offsets'][i:i+2]
            terrain.update(data['points'][p:q], data['support_points'][a:b], data['origins'][i])
        terrain.build_graph()
        home, current = terrain.nearest(data['home']), terrain.nearest(data['last_position'])
        connected = home is not None and current is not None and current in terrain.routes(home)[0]
        results.append(dict(mode=mode, metrics=terrain.metrics(), components=connectivity(terrain),
            estimated_home_supported=home is not None, estimated_current_supported=current is not None,
            estimated_home_connected=connected,
            matches_original_node_count=len(terrain.nodes) == int(data['archived_nodes']) if mode == str(data['support_fusion']) else None))
    return dict(scope=__doc__, captures=count, map_revision=int(data['map_revision']),
                capture_scope=str(data['capture_scope']), planning_state=str(data['planning_state']),
                planning_reason=str(data['planning_reason']),
                original_mode=str(data['support_fusion']), results=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture', choices=('support_failure.npz', 'support_observations.npz'), default='support_failure.npz')
    args = parser.parse_args()
    with threadpool_limits(limits=1):
        report = replay(args.run/args.capture)
    report.update(run=args.run.name, source_sha256=core_fingerprint(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256={name: hashlib.sha256((args.run/name).read_bytes()).hexdigest()
                      for name in (args.capture, 'run_config.json')})
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
