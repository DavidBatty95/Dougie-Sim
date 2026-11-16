#!/usr/bin/env python3
"""Read-only continuity and dense-display benchmark of a retained GP posterior."""
import argparse
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from dougie_sim.gp_field import observed_field_queries,interpolate_field
from dougie_sim.spatial_gp import load_gp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    with np.load(args.run/'layered_geometry.npz') as archive:
        support = archive['support_returns']
    terrain = SimpleNamespace(support_returns=dict(enumerate(support)),nodes={})
    started = time.perf_counter()
    query = observed_field_queries(terrain)
    query_seconds = time.perf_counter()-started
    posterior = args.run/'gp_posterior.npz'
    gp = load_gp(posterior)
    original = gp.blend
    # Probe both cell faces and the entry/exit edges of expert overlap supports.
    probes = []
    lower,upper = support.min(axis=0),support.max(axis=0)
    for x in np.arange(np.floor(lower[0]/gp.cell_size[0]),np.ceil(upper[0]/gp.cell_size[0])+1)*gp.cell_size[0]:
        for offset in (-gp.halo*gp.cell_size[0],0.,gp.halo*gp.cell_size[0]):
            for y in np.arange(lower[1],upper[1]+1e-9,2.):
                for z in (.44,2.84):
                    probes.append([x+offset,y,z])
    probes = np.asarray(probes)
    epsilon = 1e-6
    pair = np.stack((probes-[epsilon,0.,0.],probes+[epsilon,0.,0.]),axis=1).reshape(-1,3)
    continuity = {}
    for mode in ('linear','quintic'):
        gp.blend = mode
        values = gp.predict(pair)
        continuity[mode] = {name:float(np.max(np.abs(value.reshape(-1,2)[:,1]-value.reshape(-1,2)[:,0])))
            for name,value in zip(('mean_usv_h','sigma_log_rate','upper_usv_h'),values)}
    gp.blend = 'quintic'
    timings = []
    for _ in range(2):
        started = time.perf_counter()
        predictions = gp.predict(query)
        timings.append(time.perf_counter()-started)
    retained = json.loads((args.run/'radiation_map.json').read_text())
    reference = np.asarray(retained['points'])+[0.,0.,.44]
    started = time.perf_counter()
    mean,std,bound,_,counts = interpolate_field(reference,query,*predictions)
    interpolation_seconds = time.perf_counter()-started
    report = dict(scope='Read-only reconstruction of one retained posterior; not a navigation/dose-performance trial or accuracy calibration.',
        source=str(args.run),posterior_sha256=hashlib.sha256(posterior.read_bytes()).hexdigest(),
        recorded_blending=original,query_count=len(query),query_build_seconds=query_seconds,
        prediction_seconds_cold_then_warm=timings,interpolation_seconds=interpolation_seconds,
        reference_cells=len(reference),predicted_cells=int(np.isfinite(mean).sum()),
        retained_display_predicted_cells=retained['predicted_cells'],
        maximum_neighbours=int(counts.max()),resident_training_experts_after_prediction=len(gp.experts),
        continuity=dict(probes=len(probes),separation_m=2*epsilon,max_absolute_change=continuity),
        all_available_predictions_finite=bool(np.isfinite(std[np.isfinite(mean)]).all() and np.isfinite(bound[np.isfinite(mean)]).all()))
    payload = json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload)


if __name__=='__main__':
    main()
