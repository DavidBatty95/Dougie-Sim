#!/usr/bin/env python3
"""Profile support-graph construction from a retained multi-level mission map."""
import argparse
import cProfile
import json
from pathlib import Path
import pstats
import time

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.layered_terrain import LayeredSurfaceMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--focus', type=float, nargs=3, metavar=('X', 'Y', 'RADIUS'))
    parser.add_argument('--compare-footprints', action='store_true', help='compare warm incremental and forced-full footprint checks')
    args = parser.parse_args()
    with threadpool_limits(limits=1):
        terrain = LayeredSurfaceMap()
        with np.load(args.run/'observed_geometry.npz') as hits, np.load(args.run/'layered_geometry.npz') as layers:
            terrain.update(hits['points'], layers['support_returns'])
        profiler = cProfile.Profile()
        start = time.perf_counter()
        profiler.runcall(terrain.build_graph)
        print(f'Graph: {len(terrain.nodes)} nodes / {len(terrain.support_returns)} returns / {time.perf_counter()-start:.3f} s')
        print(terrain.metrics())
        if args.compare_footprints:
            reference = {k: list(v) for k, v in terrain.graph.items()}
            timings = {'incremental': [], 'full': []}
            counts = {}
            for _ in range(3):
                for mode in timings:
                    if mode == 'full':
                        terrain._footprint_candidates = {}
                    started = time.perf_counter()
                    terrain.build_graph()
                    timings[mode].append((time.perf_counter()-started)*1000.)
                    counts[mode] = terrain.footprints_checked
                    assert terrain.graph == reference, 'incremental graph differs from full footprint reconstruction'
            print(json.dumps(dict(scope='same retained map; warm candidate cache; full versus incremental footprints only',
                graph_nodes=len(reference), footprint_checks=counts,
                median_ms={k: float(np.median(v)) for k, v in timings.items()}), indent=2))
        if args.focus:
            x, y, radius = args.focus
            for name, values in (('supports', terrain.support_returns), ('nodes', terrain.nodes)):
                selected = np.asarray([p for p in values.values() if np.linalg.norm(p[:2]-[x, y]) < radius]).reshape(-1, 3)
                print(name, len(selected), 'height range', selected[:, 2].min() if len(selected) else None,
                      selected[:, 2].max() if len(selected) else None)
                if name == 'nodes':
                    print(selected[:30])
        pstats.Stats(profiler).sort_stats('cumulative').print_stats(15)


if __name__ == '__main__':
    main()
