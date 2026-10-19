#!/usr/bin/env python3
"""Chronological facility-count replay; evaluator truth is used only for scoring."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.assets import ROOT
from dougie_sim.gp import PoissonOnlineGP
from dougie_sim.radiation import RadiationField
from dougie_sim.spatial_gp import SpatialGP
from dougie_sim.radiation_evaluation import prediction_report


def prepared(gp, enabled):
    for expert in gp.experts.values() if isinstance(gp, SpatialGP) else [gp]:
        expert.use_prepared_predictions = enabled
        expert.invalidate_prediction_cache()


def timed_query(gp, query):
    start = time.perf_counter()
    prediction = gp.predict(query)
    return prediction, (time.perf_counter()-start)*1000.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    measurement_file = args.archive/'measurements.jsonl'
    field_file = args.archive/'evaluator_field.json'
    records = [json.loads(line) for line in measurement_file.read_text().splitlines() if line.strip()]
    split = int(.8*len(records))
    if split < 1 or split == len(records):
        parser.error('replay requires training and held-out measurements')
    query = np.asarray([r['position'] for r in records[split:]])
    field = RadiationField(json.loads(field_file.read_text()))
    truth = np.asarray([field.rate(p) for p in query])
    training_positions = np.asarray([r['position'] for r in records[:split]])
    heldout_counts = np.asarray([r['counts'] for r in records[split:]])
    exposure = np.asarray([r['integration_s']*r['counts_per_usv']/3600. for r in records[split:]])
    models = [('global_256', PoissonOnlineGP(256)), ('global_512', PoissonOnlineGP(512)),
              ('spatial_32x128', SpatialGP(32, 128))]
    results = []
    with threadpool_limits(limits=1):
        for name, gp in models:
            start = time.perf_counter()
            for r in records[:split]:
                gp.update_counts(r['position'], r['counts'], r['integration_s'], r['counts_per_usv'])
            training_s = time.perf_counter()-start
            modes = {}
            reference = None
            for enabled in (False, True):
                prepared(gp, enabled)
                prediction, cold_ms = timed_query(gp, query)
                if reference is None:
                    reference = prediction
                else:
                    np.testing.assert_allclose(prediction, reference, rtol=1e-8, atol=1e-8)
                times = [timed_query(gp, query)[1] for _ in range(5)]
                modes['prepared' if enabled else 'reference'] = dict(cold_ms=cold_ms,
                    warm_p50_ms=float(np.median(times)), warm_p95_ms=float(np.percentile(times, 95)),
                    max_absolute_difference=float(np.max(np.abs(np.asarray(prediction)-reference))))
            mean, _, upper = prediction
            results.append(dict(name=name, training_wall_s=training_s, prediction=modes,
                log1p_rate_rmse=float(np.sqrt(np.mean((np.log1p(mean)-np.log1p(truth))**2))),
                upper_rate_coverage=float(np.mean(upper >= truth)), metrics=gp.metrics(),
                strata=prediction_report(query, truth, mean, upper, training_positions, heldout_counts, exposure, field.shields)))
    files = [measurement_file, field_file, Path(__file__), ROOT/'dougie_sim/gp.py', ROOT/'dougie_sim/spatial_gp.py',
             ROOT/'dougie_sim/radiation_evaluation.py']
    report = dict(archive=str(args.archive), split='first 80% train; final 20% queries, never assimilated',
        training_samples=split, query_points=len(query), training_end_s=records[split-1]['stamp_s'],
        scope='recorded facility counts, chronologically held-out trajectory; spatially correlated revisits, not unseen-building validation; different model budgets',
        source_sha256={str(p.relative_to(ROOT) if p.is_absolute() and p.is_relative_to(ROOT) else p):
                       hashlib.sha256(p.read_bytes()).hexdigest() for p in files}, results=results)
    output = ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('gp_replay_%Y%m%dT%H%M%S_%fZ.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), **report), indent=2))


if __name__ == '__main__':
    main()
