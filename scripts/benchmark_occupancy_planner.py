#!/usr/bin/env python3
"""Compare old/new occupancy access inside real frontier evaluation."""
import argparse
from contextlib import nullcontext
import gc
import json
from pathlib import Path
import time
import tracemalloc
from unittest.mock import patch

import numpy as np

from dougie_sim.layered_terrain import LayeredSurfaceMap
from dougie_sim.region_clustering import ObservedGain
from scripts.benchmark_occupancy_access import old_index


def run(terrain,legacy):
    context=patch('dougie_sim.occupancy_access.dense_confirmed_free',old_index) if legacy else nullcontext()
    gc.collect()
    with context:
        tracemalloc.start()
        started=time.perf_counter()
        gain=ObservedGain(terrain)
        setup=time.perf_counter()-started
        peak=tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
        if gain.free_grid is None:
            raise ValueError('fixture does not exercise dense occupancy path')
        started=time.perf_counter()
        values=gain.values(list(terrain.nodes))
        evaluation=time.perf_counter()-started
    return gain,values,dict(setup_seconds=setup,setup_peak_traced_bytes=peak,evaluation_seconds=evaluation,
        frontiers=int(np.count_nonzero(values)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    terrain=LayeredSurfaceMap()
    with np.load(args.run/'observed_geometry.npz') as hits,np.load(args.run/'layered_geometry.npz') as data:
        terrain.update(hits['points'],data['support_returns'])
        terrain.occupancy.evidence=dict(zip(map(tuple,data['occupancy_keys']),map(int,data['occupancy_evidence'])))
        terrain.occupancy.coarse_free=dict(zip(map(tuple,data['coarse_free_keys']),map(int,data['coarse_free_evidence'])))
    started=time.perf_counter()
    terrain.build_graph()
    graph_s=time.perf_counter()-started
    baseline,old,old_metrics=run(terrain,True)
    current,new,new_metrics=run(terrain,False)
    np.testing.assert_array_equal(old,new)
    if baseline.witnesses!=current.witnesses:
        raise AssertionError('frontier unknown-cell witnesses changed')
    report=dict(scope='One retained multilevel map; exact old/new frontier calculation from the same reconstructed support graph. No physical mission or dose inference.',
        source=str(args.run),graph_nodes=len(terrain.nodes),graph_reconstruction_seconds=graph_s,
        identical_gains=True,identical_unknown_witnesses=True,baseline=old_metrics,optimized=new_metrics,
        setup_time_ratio=new_metrics['setup_seconds']/old_metrics['setup_seconds'],
        setup_peak_memory_ratio=new_metrics['setup_peak_traced_bytes']/old_metrics['setup_peak_traced_bytes'])
    payload=json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload)


if __name__=='__main__':
    main()
