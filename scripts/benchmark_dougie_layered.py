#!/usr/bin/env python3
"""Profile support-graph construction from a retained multi-level mission map."""
import argparse
import cProfile
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
