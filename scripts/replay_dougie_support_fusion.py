#!/usr/bin/env python3
"""Compare support estimators on identical archived observations.

First-failure captures retain that decision's frame; final captures include later
SLAM corrections. This is a geometry ablation, not an executable radiation-safe route.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

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
        home_component = {} if home is None else terrain.routes(home)[0]
        highest = max(home_component, key=lambda key: terrain.nodes[key][2], default=None)
        results.append(dict(mode=mode, metrics=terrain.metrics(), components=connectivity(terrain),
            estimated_home_supported=home is not None, estimated_current_supported=current is not None,
            estimated_home_connected=connected,
            estimated_current_position=data['last_position'].tolist(),
            nearest_current_node=None if current is None else terrain.nodes[current].tolist(),
            nearest_current_distance_m=None if current is None else float(np.linalg.norm(terrain.nodes[current]-data['last_position'])),
            maximum_home_connected_height_m=max((float(terrain.nodes[key][2]) for key in home_component), default=None),
            current_support_diagnosis=terrain.support_diagnosis(data['last_position']),
            highest_connected_support_diagnosis=None if highest is None else terrain.support_diagnosis(terrain.nodes[highest]),
            matches_original_node_count=len(terrain.nodes) == int(data['archived_nodes']) if mode == str(data['support_fusion']) else None))
    return dict(scope=__doc__, captures=count, map_revision=int(data['map_revision']),
                capture_scope=str(data['capture_scope']), planning_state=str(data['planning_state']),
                planning_reason=str(data['planning_reason']),
                original_mode=str(data['support_fusion']), results=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--quiet', action='store_true', help='write the report without printing its contents')
    parser.add_argument('--capture', choices=('support_failure.npz', 'support_observations.npz'), default='support_failure.npz')
    parser.add_argument('--export-inputs', type=Path, help='new directory for exact portable capture/config copies')
    args = parser.parse_args()
    with threadpool_limits(limits=1):
        report = replay(args.run/args.capture)
    report.update(run=args.run.name, source_sha256=core_fingerprint(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256={name: hashlib.sha256((args.run/name).read_bytes()).hexdigest()
                      for name in (args.capture, 'run_config.json')})
    if args.export_inputs is not None:
        args.export_inputs.mkdir(exist_ok=False)
        for name in (args.capture, 'run_config.json'):
            shutil.copyfile(args.run/name, args.export_inputs/name)
            if hashlib.sha256((args.export_inputs/name).read_bytes()).hexdigest() != report['input_sha256'][name]:
                raise ValueError('portable support replay copy failed checksum validation')
        report['portable_inputs'] = str(args.export_inputs)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    if not args.quiet:
        print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
