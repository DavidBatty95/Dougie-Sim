#!/usr/bin/env python3
"""Reproducible XYZ mapping benchmark, separate from closed-loop qualification."""
import json
from pathlib import Path
import time

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.gp import OnlineGP, PoissonOnlineGP
from dougie_sim.radiation import RadiationField
from dougie_sim.reactor import radiation_description


def main():
    rng = np.random.default_rng(42)
    train = rng.uniform([-8, -5, .2], [8, 5, 2.5], (2000, 3))
    heldout = rng.uniform([-8, -5, .2], [8, 5, 2.5], (1000, 3))
    field = RadiationField(radiation_description())
    truth_train = np.array([field.rate(point) for point in train])
    truth = np.array([field.rate(point) for point in heldout])
    expected_counts = truth_train/3600.*50000.*.5
    counts = rng.poisson(expected_counts)
    observed = counts*3600./25000.
    noise = (counts+1)*(3600./25000.)**2
    results = []
    with threadpool_limits(limits=1):
        for kind, capacity in [(kind, n) for kind in ('gaussian', 'poisson') for n in (32, 64, 128)]:
            gp = PoissonOnlineGP(capacity=capacity) if kind == 'poisson' else OnlineGP(capacity=capacity)
            for x, y, var, count in zip(train, observed, noise, counts):
                if kind == 'poisson':
                    gp.update_counts(x, int(count), .5)
                else:
                    gp.update(x, y, var)
            start = time.perf_counter()
            mean, std, upper = gp.predict(heldout)
            elapsed = time.perf_counter()-start
            logmean, logvar = gp.predict_log(heldout)
            target = np.log(truth) if kind == 'poisson' else np.log1p(truth)
            metrics = gp.metrics()
            metrics.update(likelihood=kind, query_points=len(heldout), batch_prediction_ms=elapsed*1000.,
                rmse_usv_h=float(np.sqrt(np.mean((mean-truth)**2))),
                log1p_rate_rmse=float(np.sqrt(np.mean((np.log1p(mean)-np.log1p(truth))**2))),
                latent_rmse=float(np.sqrt(np.mean((logmean-target)**2))),
                prior_latent_rmse=float(np.sqrt(np.mean((gp.prior-target)**2))),
                latent_95_coverage=float(np.mean(np.abs(target-logmean) <= 1.96*np.sqrt(logvar))),
                upper_rate_coverage=float(np.mean(truth <= upper)))
            results.append(metrics)
    output = Path('results/dougie/gp_benchmark.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(seed=42, training_points=len(train), heldout_points=len(heldout),
                  scope='uniform 3-D synthetic sensor samples, not a robot mission or a SOTA comparison', results=results)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
