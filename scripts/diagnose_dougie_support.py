#!/usr/bin/env python3
"""Rebuild archived observed support and report connectivity without simulator truth.

The archive is the final map, not a capture-time failure snapshot. In particular,
post-stop graph corrections can make it differ from the map that lost its route.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from dougie_sim.layered_terrain import LayeredSurfaceMap
from dougie_sim.pose_uncertainty import core_fingerprint


def connectivity(terrain):
    remaining = set(terrain.nodes)
    components = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        pending, members = [start], []
        while pending:
            node = pending.pop()
            members.append(node)
            for neighbour in terrain.graph.get(node, []):
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    pending.append(neighbour)
        xyz = np.asarray([terrain.nodes[key] for key in members])
        components.append(dict(nodes=len(members), minimum_xyz=xyz.min(axis=0).tolist(),
                               maximum_xyz=xyz.max(axis=0).tolist()))
    return sorted(components, key=lambda item: (-item['nodes'], item['minimum_xyz']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--column', type=float, nargs=2, action='append', default=[], metavar=('X', 'Y'),
                        help='inspect candidate admission at an observed-map XY column (repeatable)')
    args = parser.parse_args()
    summary_path = args.run/'radiation_summary.json'
    summary = json.loads(summary_path.read_text())
    terrain = LayeredSurfaceMap(radius=.45+summary.get('tracking_clearance_margin_m', 0.))
    terrain.set_localization_margin(summary.get('localization_margin_m', 0.))
    with np.load(args.run/'observed_geometry.npz', allow_pickle=False) as observed:
        points = observed['points']
    with np.load(args.run/'layered_geometry.npz', allow_pickle=False) as layered:
        supports = layered['support_returns']
    terrain.update(points, supports)
    terrain.build_graph()
    entries = [entry for _, cached in terrain._candidate_cache.values() for entry in cached]
    files = ('radiation_summary.json', 'observed_geometry.npz', 'layered_geometry.npz')
    columns = []
    for xy in args.column:
        key = tuple(np.floor(np.asarray(xy)/terrain.resolution).astype(int))
        count, cached = terrain._candidate_cache.get(key, (0, []))
        columns.append(dict(xy=xy, observed_layers=count, rejections=terrain.fit_rejections.get(key, []), candidates=[dict(
            position=point.tolist(), slope_deg=float(np.degrees(np.arctan(np.linalg.norm(plane[:2])))),
            admission='body_blocked' if blocked else 'admitted' if node in terrain.nodes else 'footprint_rejected')
            for node, point, plane, blocked in cached]))
    report = dict(scope='offline final observed-map reconstruction, not capture-time failure diagnosis',
        run=args.run.name, input_sha256={name: hashlib.sha256((args.run/name).read_bytes()).hexdigest() for name in files},
        reconstruction_source_sha256=core_fingerprint(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        radius_m=terrain.radius, observed_support_returns=len(supports),
        fitted_candidates=len(entries), body_blocked_candidates=sum(bool(entry[3]) for entry in entries),
        admitted_nodes=len(terrain.nodes), archived_nodes=summary.get('terrain', {}).get('support_nodes'),
        components=connectivity(terrain), inspected_columns=columns)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: report[key] for key in ('run', 'admitted_nodes', 'archived_nodes', 'components', 'inspected_columns')}))


if __name__ == '__main__':
    main()
