#!/usr/bin/env python3
"""Read-only matched-progress diagnosis from retained paired measurements."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def rows(path, horizon=None, stamp='time_s'):
    with path.open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                break
            if horizon is not None and row[stamp] > horizon:
                break
            yield row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--horizon', type=float, default=600.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = dict(run=args.run.name, diagnosis_horizon_s=args.horizon, modes={}, milestones=[])
    series = {}
    for mode in ('blind', 'aware'):
        directory = args.run/mode
        series[mode] = list(rows(directory/'comparison.jsonl'))
        samples = list(rows(directory/'measurements.jsonl', args.horizon, 'stamp_s'))
        plans = list(rows(directory/'trace.jsonl', args.horizon))
        targets = Counter(tuple(np.round(np.asarray(p['selected_target'])/[.5, .5, .2]).astype(int))
                          for p in plans if p.get('selected_target') is not None)
        dose_terms = [5.*p['route_forecast']['outbound_upper_dose_usv'] for p in plans
                      if 'outbound_upper_dose_usv' in p.get('route_forecast', {})]
        rates = [s['rate_usv_h'] for s in samples]
        last = series[mode][-1]
        first_at_final_floor = next(row for row in series[mode] if row['explored_percent'] >= last['explored_percent']-.05)
        summary_path = directory/'radiation_summary.json'
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        report['modes'][mode] = dict(endpoint=last,
            last_meaningful_floor_progress_s=first_at_final_floor['time_s'],
            time_after_floor_plateau_s=last['time_s']-first_at_final_floor['time_s'],
            dose_after_floor_plateau_usv=last['true_dose_usv']-first_at_final_floor['true_dose_usv'],
            rate_median_usv_h=float(np.median(rates)) if rates else None,
            rate_p95_usv_h=float(np.quantile(rates,.95)) if rates else None,
            legacy_dose_penalty_median=float(np.median(dose_terms)) if dose_terms else None,
            legacy_dose_penalty_p95=float(np.quantile(dose_terms,.95)) if dose_terms else None,
            plans_with_revisit_utility=sum(bool(p.get('selected_utility')) for p in plans),
            planning_updates=len(plans), distinct_half_metre_targets=len(targets),
            gp=summary.get('gp'), termination=summary.get('decision_reason'),
            physical_witness=summary.get('physical_witness'))
    for threshold in (25.,50.,75.,90.):
        hits = {mode:next((r for r in series[mode] if r['explored_percent'] >= threshold),None) for mode in series}
        if all(hits.values()):
            report['milestones'].append(dict(reference_percent=threshold,
                **{m:{k:r[k] for k in ('time_s','true_dose_usv','distance_m','explored_percent')} for m,r in hits.items()},
                mean_true_rate_usv_h={m:r['true_dose_usv']*3600./r['time_s'] for m,r in hits.items()},
                aware_time_change_percent=100.*(hits['aware']['time_s']/hits['blind']['time_s']-1.),
                aware_distance_change_percent=100.*(hits['aware']['distance_m']/hits['blind']['distance_m']-1.),
                aware_relative_dose_change_percent=100.*(hits['aware']['true_dose_usv']/hits['blind']['true_dose_usv']-1.)))
    payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload)


if __name__ == '__main__':
    main()
