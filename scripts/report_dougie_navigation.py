#!/usr/bin/env python3
"""Compact, hashed navigation evidence retaining failed gates and forecast errors."""
import argparse
import hashlib
import json
from pathlib import Path


def load(path):
    raw = path.read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--missions', nargs='+', type=Path, required=True)
    parser.add_argument('--gp-replays', nargs='*', type=Path, default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = dict(scope='development comparisons; varying objectives and observed area; not reliability qualification',
                  missions=[], gp_replays=[])
    for run in args.missions:
        summary, summary_hash = load(run/'radiation_summary.json')
        config, config_hash = load(run/'run_config.json')
        trace = [json.loads(line) for line in (run/'trace.jsonl').read_text().splitlines() if line.strip()]
        returning = next((row for row in trace if row['state'].startswith('return') and row['state'] != 'returned'), None)
        forecast = None
        if returning and summary.get('clean_return'):
            forecast = dict(first_return_plan_s=returning['time_s'],
                forecast_s=returning.get('route_forecast', {}).get('return_duration_s'),
                actual_remaining_s=summary['simulation']['simulation_time_s']-returning['time_s'])
        keys = ('clean_return', 'round_trip_completed', 'round_trip_failures', 'landmark_mission_completed',
                'landmarks', 'error', 'true_dose_usv', 'path_length_m', 'mapped_surface_area_m2',
                'turning_in_place_s', 'target_switches', 'nonfoot_contacts', 'worker_p95_ms',
                'worker_phase_p95_ms', 'wall_time_s', 'physical_witness', 'region_agenda', 'source_sha256',
                'localization', 'localization_evaluation', 'gp', 'information_contract', 'map_consistency',
                'recovery', 'terminal_stop', 'post_stop_true_dose_usv', 'count_calibration', 'circuit_mission_completed',
                'tracking_clearance_margin_m', 'localization_margin_m')
        report['missions'].append(dict(run=run.name, summary_sha256=summary_hash,
            config_sha256=config_hash, configuration={k: config.get(k) for k in
                ('world', 'seed', 'layout_seed', 'duration', 'dose_budget', 'gp_mode', 'path_mode',
                 'mission_objective', 'region_scheduling', 'fixed_plan_delay', 'ros', 'fast',
                 'localization', 'sensor_seed', 'detector_likelihood', 'navigation_profile', 'utility_mode',
                 'lidar_dropout_after', 'circuit_radius', 'require_circuit')},
            simulated_time_s=summary['simulation']['simulation_time_s'], return_forecast=forecast,
            **{k: summary.get(k) for k in keys}))
    for path in args.gp_replays:
        replay, digest = load(path)
        report['gp_replays'].append(dict(file=path.name, sha256=digest,
            position_sensitivity=replay['position_sensitivity'], source_sha256=replay['source_sha256'],
            results=[{k: result[k] for k in ('name', 'log1p_rate_rmse', 'upper_bound_sweep', 'strata')}
                     for result in replay['results']]))
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
