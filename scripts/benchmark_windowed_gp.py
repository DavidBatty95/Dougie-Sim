#!/usr/bin/env python3
"""Synthetic online-update/paging check, not a robot mission benchmark."""
import argparse
import json
from pathlib import Path
import tempfile
import time

import numpy as np

from dougie_sim.windowed_gp import WindowedGP


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--regions', type=int, default=24)
    parser.add_argument('--updates-per-region', type=int, default=400)
    parser.add_argument('--resident', type=int, default=4)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if min(args.regions, args.updates_per_region, args.resident) < 1:
        parser.error('counts must be positive')
    rng = np.random.default_rng(42)
    with tempfile.TemporaryDirectory(prefix='windowed-gp-benchmark-') as cache:
        gp = WindowedGP(32, args.resident, max_planes=0, store_dir=cache)
        timings, residents = [], []
        retained = None
        start = time.perf_counter()
        for region in range(args.regions):
            for _ in range(args.updates_per_region):
                point = np.array([region*4.+2.,2.,.75])+rng.uniform([-1.,-1.,-.25],[1.,1.,.25])
                window = point+rng.uniform(-.02,.02,(4,3))
                tick = time.perf_counter()
                gp.update_counts_window(window, [.125]*4, int(rng.poisson(12.)))
                timings.append((time.perf_counter()-tick)*1000.)
                residents.append(len(gp.experts))
            if region == 0:
                retained = gp.predict([[2.,2.,.75]])
        restored = gp.predict([[2.,2.,.75]])
        for actual, before in zip(restored, retained):
            np.testing.assert_allclose(actual,before,rtol=1e-10,atol=1e-10)
        report = dict(scope='Synthetic four-point count windows, static local Matérn kernel; no navigation or geometry-fitting workload.',
            updates=len(timings), regions=args.regions, max_resident_seen=max(residents),
            update_p50_ms=float(np.median(timings)), update_p95_ms=float(np.quantile(timings,.95)),
            wall_time_s=time.perf_counter()-start, retained_prediction_equal=True, gp=gp.metrics())
    payload = json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload)


if __name__ == '__main__':
    main()
