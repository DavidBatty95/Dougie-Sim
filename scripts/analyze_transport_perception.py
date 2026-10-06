#!/usr/bin/env python3
"""Summarize acquisition timing and outcomes from retained transport runs."""
import argparse
import json
from pathlib import Path

import numpy as np


def distribution(values):
    if not values:
        return dict(count=0)
    return dict(count=len(values),median=float(np.median(values)),
                p95=float(np.quantile(values,.95)),maximum=float(max(values)))


def summarize(run):
    history = json.loads((run/'perception_history.json').read_text()) if (run/'perception_history.json').exists() else []
    attempts = json.loads((run/'perception_attempts.json').read_text()) if (run/'perception_attempts.json').exists() else []
    result = json.loads((run/'result.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text())
    arrivals = {}
    for item in attempts:
        arrivals.setdefault(item['stamp'],item['processed_at'])
    stamps = sorted(set(item['stamp'] for item in history))
    return dict(run=run.name,policy=manifest['settings'].get('policy_path'),
                seed=manifest['seed'],
                source_sha256=manifest['source_sha256'],
                localization_recoveries=sum(bool(item.get('localization_recovered')) for item in result.get('events',[])),
                object_timestamp_scope=result.get('evaluation_health',{}).get('object_timestamp_scope'),
                history_available=(run/'perception_history.json').exists(),
                camera=[manifest['settings'][key] for key in ('camera_width','camera_height')],
                duration_s=manifest['duration_s'],
                acquisition_interval_s=distribution(np.diff(stamps).tolist()),
                first_attempt_age_s=distribution([arrival-stamp for stamp,arrival in arrivals.items()]),
                processed_age_s=distribution([item['processed_at']-item['stamp'] for item in history]),
                confirmed_ids=sorted({identifier for item in history for identifier in item['confirmed']}),
                success=result['success'],data_valid=result['data_valid'],fallen=result['fallen'],
                deliveries=result['deliveries'],dropped_objects=result['dropped_objects'],error=result.get('error'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    text = json.dumps([summarize(run) for run in args.runs],indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text,end='')


if __name__=='__main__':
    main()
