#!/usr/bin/env python3
"""Audit retained frontier geometry without supplying evaluator maps to planning."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from dougie_sim.layered_terrain import LayeredSurfaceMap
from dougie_sim.region_clustering import ObservedGain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--point',type=float,nargs=3)
    args = parser.parse_args()
    terrain = LayeredSurfaceMap()
    with np.load(args.run/'layered_geometry.npz') as layers, np.load(args.run/'observed_geometry.npz') as hits:
        terrain.update(hits['points'],layers['support_returns'])
        terrain.occupancy.evidence = dict(zip(map(tuple,layers['occupancy_keys']),layers['occupancy_evidence']))
        terrain.occupancy.coarse_free = dict(zip(map(tuple,layers['coarse_free_keys']),layers['coarse_free_evidence']))
    terrain.build_graph()
    gain = ObservedGain(terrain)
    keys = list(terrain.nodes)
    values = gain.values(keys)
    report = dict(frontiers_by_height=dict(Counter(str(round(float(terrain.nodes[k][2]),1))
        for k,v in zip(keys,values) if v)),support_nodes=len(keys))
    if args.point:
        key = terrain.nearest(args.point)
        if key is None:
            raise ValueError('point has no nearby supported node')
        # Use a sparse view to retain the individual first-unknown ray witnesses.
        columns, hits = {}, {}
        for destination, points in ((columns,terrain.support_returns.values()),(hits,terrain.hits.values())):
            for point in points:
                cell = tuple(np.floor(point[:2]/terrain.resolution+1e-8).astype(int))
                destination.setdefault(cell,[]).append(float(point[2]))
        witnesses = []
        for i,ray in enumerate(gain.offsets):
            height = float(terrain.nodes[key][2])
            for step,(dx,dy) in enumerate(ray):
                cell = key[0]+dx,key[1]+dy
                support = columns.get(cell,[])
                eligible = [z for z in support if abs(z-height) <= gain.allowed_steps[i,step]]
                next_height = min(eligible,key=lambda z:abs(z-height)) if eligible else height
                if any(.12 < z-next_height < .7 for z in hits.get(cell,[])):
                    break
                if not eligible:
                    cleared = bool(gain.cleared_support(np.asarray(cell),np.asarray(height),gain.allowed_steps[i,step]))
                    drop = cleared
                    if not drop:
                        witnesses.append(dict(xy=((np.asarray(cell)+.5)*terrain.resolution).tolist(),
                            expected_height_m=height,support_heights_m=sorted(support),
                            hit_heights_m=sorted(hits.get(cell,[])),support_interval_cleared=cleared))
                    break
                height = next_height
        report.update(point=terrain.nodes[key].tolist(),gain=int(gain.value(key)),unknown_ray_witnesses=witnesses)
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
