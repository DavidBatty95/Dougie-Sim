#!/usr/bin/env python3
"""Matched XYZ GP scalability/accuracy benchmark with explicit model budgets."""
from datetime import datetime, timezone
import json
import time

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.assets import ROOT
from dougie_sim.gp import PoissonOnlineGP
from dougie_sim.radiation import RadiationField
from dougie_sim.spatial_gp import SpatialGP


def main():
    rng = np.random.default_rng(43)
    train = np.column_stack((rng.uniform(-12, 12, (3000, 2)), rng.choice([.44, 2.84, 5.24], 3000)))
    query = np.column_stack((rng.uniform(-12, 12, (10000, 2)), rng.choice([.44, 2.84, 5.24], 10000)))
    field = RadiationField(dict(background_usv_h=.4, sources=[
        dict(position=p, strength_usv_h_m2=s, core_radius_m=.15)
        for p, s in (([-5, -4, .6], 35.), ([4, 6, 2.8], 70.), ([0, -5, 5.4], 40.))]))
    truth_train = np.asarray([field.rate(p) for p in train])
    truth = np.asarray([field.rate(p) for p in query])
    counts = rng.poisson(truth_train*25000./3600.)
    models = [('global_256', PoissonOnlineGP(256)), ('global_512', PoissonOnlineGP(512)),
              ('spatial_32x256', SpatialGP(32, 256))]
    results = []
    with threadpool_limits(limits=1):
        for name, gp in models:
            start = time.perf_counter()
            for p, count in zip(train, counts):
                gp.update_counts(p, int(count), .5)
            update_wall = time.perf_counter()-start
            gp.predict(query)  # Warm up the same batch.
            times = []
            for _ in range(5):
                start = time.perf_counter()
                mean, std, upper = gp.predict(query)
                times.append((time.perf_counter()-start)*1000.)
            results.append(dict(name=name, training_wall_s=update_wall,
                prediction_p50_ms=float(np.median(times)), prediction_p95_ms=float(np.percentile(times, 95)),
                log1p_rate_rmse=float(np.sqrt(np.mean((np.log1p(mean)-np.log1p(truth))**2))),
                rate_rmse=float(np.sqrt(np.mean((mean-truth)**2))),
                upper_rate_coverage=float(np.mean(upper >= truth)), metrics=gp.metrics()))
    report = dict(seed=43, training_samples=len(train), query_points=len(query),
                  domain_m=[24, 24], sensor_heights_m=[.44, 2.84, 5.24],
                  scope='matched synthetic counting observations; model budgets differ and are reported; not mission qualification',
                  results=results)
    output = ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('spatial_gp_%Y%m%dT%H%M%S_%fZ.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), **report), indent=2))


if __name__ == '__main__':
    main()
