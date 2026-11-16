#!/usr/bin/env python3
"""Benchmark exact compact-occupancy lookup without changing a mission map."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import tracemalloc

import numpy as np
from dougie_sim.occupancy_access import dense_confirmed_free,SparseConfirmedFree


def old_index(evidence,coarse,origin,shape):
    cells={k for k,v in evidence.items() if v<=-3}
    for block in coarse:
        for dx in (0,1):
            for dy in (0,1):
                for dz in (0,1):
                    cells.add((2*block[0]+dx,2*block[1]+dy,2*block[2]+dz))
    grid=np.zeros(tuple(shape),dtype=bool)
    indices=np.asarray(list(cells),dtype=int).reshape(-1,3)-origin
    indices=indices[np.all((indices>=0)&(indices<shape),axis=1)]
    grid[tuple(indices.T)]=True
    return grid


def old_sparse(evidence,coarse):
    cells={k for k,v in evidence.items() if v<=-3}
    for block in coarse:
        for dx in (0,1):
            for dy in (0,1):
                for dz in (0,1):
                    cells.add((2*block[0]+dx,2*block[1]+dy,2*block[2]+dz))
    return cells


def measure(function,*args,repeats=4):
    times=[]
    peaks=[]
    result=None
    plain=[]
    for _ in range(repeats):
        started=time.perf_counter()
        result=function(*args)
        plain.append(time.perf_counter()-started)
    for _ in range(repeats):
        tracemalloc.start()
        started=time.perf_counter()
        result=function(*args)
        times.append(time.perf_counter()-started)
        peaks.append(tracemalloc.get_traced_memory()[1])
        tracemalloc.stop()
    return result,dict(seconds_with_tracemalloc=times,median_seconds=float(np.median(plain)),
        untraced_seconds=plain,
        peak_traced_bytes=max(peaks))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with np.load(args.map,allow_pickle=False) as data:
        evidence=dict(zip(map(tuple,data['occupancy_keys']),map(int,data['occupancy_evidence'])))
        coarse=dict(zip(map(tuple,data['coarse_free_keys']),map(int,data['coarse_free_evidence'])))
    combined=np.concatenate((np.asarray(list(evidence),dtype=int).reshape(-1,3),
        np.asarray(list(coarse),dtype=int).reshape(-1,3)*2))
    origin=combined.min(axis=0)-2
    shape=combined.max(axis=0)-origin+4
    if int(np.prod(shape))>5000000:
        raise ValueError('fixture too large for dense reference benchmark')
    old,baseline=measure(old_index,evidence,coarse,origin,shape)
    new,proposed=measure(dense_confirmed_free,evidence,coarse,origin,shape)
    np.testing.assert_array_equal(old,new)
    expanded,old_sparse_stats=measure(old_sparse,evidence,coarse)
    sparse,sparse_stats=measure(SparseConfirmedFree,evidence,coarse)
    rng=np.random.default_rng(21)
    queries=rng.integers(origin,origin+shape,size=(50000,3))
    old_values,old_query_stats=measure(lambda:np.asarray([tuple(key) in expanded for key in queries]))
    new_values,new_query_stats=measure(lambda:sparse.contains(queries))
    np.testing.assert_array_equal(new_values,old_values)
    report=dict(scope='Read-only index construction on one retained occupancy snapshot; not end-to-end navigation or mapping throughput.',
        source=str(args.map),source_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),
        fine_cells=len(evidence),coarse_free_blocks=len(coarse),
        dense_shape=shape.tolist(),confirmed_free_cells=int(new.sum()),
        exact_equal=True,baseline=baseline,proposed=proposed,
        sparse_exact_equal=True,sparse_baseline=old_sparse_stats,sparse_proposed=sparse_stats,
        sparse_query_count=len(queries),sparse_query_baseline=old_query_stats,sparse_query_proposed=new_query_stats,
        time_ratio=proposed['median_seconds']/baseline['median_seconds'],
        peak_memory_ratio=proposed['peak_traced_bytes']/baseline['peak_traced_bytes'])
    payload=json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload)


if __name__=='__main__':
    main()
