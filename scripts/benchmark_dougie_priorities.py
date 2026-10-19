#!/usr/bin/env python3
"""Controlled routing checks and optional retained occupancy-map compaction replay.

Uses prescribed graph rates to isolate routing, not a learned-GP or physical
avoidance comparison. Report completion alongside exposure; a stop is not success.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from dougie_sim.assets import ROOT
from dougie_sim.layered_terrain import VoxelEvidence
from dougie_sim.route_forecast import forecast_tree
from dougie_sim.terrain import SurfaceMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--occupancy-archive', type=Path)
    args = parser.parse_args()
    terrain = SurfaceMap()
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([1., 0., 0.]),
        2: np.array([2., 0., 0.]), 3: np.array([0., 2., 0.]), 4: np.array([2., 2., 0.])}
    terrain.graph = {0: [1, 3], 1: [0, 2], 2: [1, 4], 3: [0, 4], 4: [3, 2]}
    cases = []
    for name, hotspot in (('uniform_control', 1.), ('hot_shortcut', 200.)):
        rates = {key: hotspot if key == 1 else 1. for key in terrain.nodes}
        for mode in ('geometry_only', 'reactive_stop', 'rate_constrained'):
            _, _, _, parent = terrain.routes(0, rates if mode == 'rate_constrained' else None,
                max_rate=100. if mode == 'rate_constrained' else None)
            route = terrain.path(parent, 2)
            if mode == 'reactive_stop':
                stop = next((i for i, key in enumerate(route) if rates[key] > 100.), len(route)-1)
                route = route[:stop+1]
            selected_tree = {key: route[i-1] if i else None for i, key in enumerate(route)}
            duration, dose, peak = forecast_tree(terrain, selected_tree, rates)
            cases.append(dict(scenario=name, mode=mode, route=route, completed=route[-1] == 2,
                forecast_duration_s=duration[route[-1]], integrated_upper_edge_dose_usv=dose[route[-1]],
                peak_rate_usv_h=peak[route[-1]]))
    report = dict(scope=__doc__, routing=cases)
    if args.occupancy_archive:
        with np.load(args.occupancy_archive) as archive:
            occupancy = VoxelEvidence()
            occupancy.evidence = {tuple(map(int, k)): int(v) for k, v in
                zip(archive['occupancy_keys'], archive['occupancy_evidence'])}
            if 'coarse_free_keys' in archive:
                occupancy.coarse_free = {tuple(map(int, k)): int(v) for k, v in
                    zip(archive['coarse_free_keys'], archive['coarse_free_evidence'])}
        occupied = {k: v for k, v in occupancy.evidence.items() if v >= 0}
        represented_before = len(occupancy.evidence)+8*len(occupancy.coarse_free)
        before = occupancy.storage_cells
        merged = occupancy.compact_free()
        assert all(occupancy.evidence[k] == value for k, value in occupied.items())
        assert represented_before == len(occupancy.evidence)+8*len(occupancy.coarse_free)
        report['compaction'] = dict(archive=str(args.occupancy_archive),
            archive_sha256=hashlib.sha256(args.occupancy_archive.read_bytes()).hexdigest(),
            storage_cells_before=before, storage_cells_after=occupancy.storage_cells,
            merged_blocks=merged, represented_fine_cells=represented_before, nonfree_cells_preserved=len(occupied))
    files = [Path(__file__), ROOT/'dougie_sim/terrain.py', ROOT/'dougie_sim/route_forecast.py', ROOT/'dougie_sim/layered_terrain.py']
    report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    output = ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('priorities_%Y%m%dT%H%M%S_%fZ.json')
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), **report), indent=2))


if __name__ == '__main__':
    main()
