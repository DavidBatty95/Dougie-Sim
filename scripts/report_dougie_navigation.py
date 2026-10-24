#!/usr/bin/env python3
"""Compact, hashed navigation evidence retaining failed gates and forecast errors."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def load(path):
    raw = path.read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def terminal_motion(run, summary):
    """Independent 20 Hz physical trace; never substitute estimated pose drift."""
    stop = summary.get('terminal_stop')
    path = run/'motion_trace.npz'
    if not stop or not path.exists():
        return None
    with np.load(path, allow_pickle=False) as archive:
        rows = archive['rows']
    if rows.ndim != 2 or rows.shape[1] < 4 or not np.isfinite(rows[:, :4]).all():
        raise ValueError('invalid physical motion trace')
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('physical motion trace times must increase')
    selected = rows[rows[:, 0] >= stop['time_s']]
    if not len(selected):
        raise ValueError('missing post-stop physical samples')
    return dict(scope='sampled evaluator XY drift from first post-stop frame; 20 Hz, not a continuous-time bound',
        trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), samples=len(selected),
        first_sample_s=float(selected[0, 0]), last_sample_s=float(selected[-1, 0]),
        first_position=selected[0, 1:4].tolist(), last_position=selected[-1, 1:4].tolist(),
        max_horizontal_drift_m=float(np.linalg.norm(selected[:, 1:3]-selected[0, 1:3], axis=1).max()))


def navigation_transitions(trace):
    """Retain failure onset and recovery changes, not just the final summary.

    Trace rows are planning snapshots, so timestamps are observation times rather
    than an assertion of the exact instant a physical failure occurred.
    """
    events = []
    previous = None
    for row in trace:
        recovery = row.get('recovery') or {}
        checkpoint = row.get('localization_checkpoints') or {}
        revision = (row.get('map_consistency') or {}).get('map_revision')
        signature = (row.get('state'), row.get('decision_reason'), revision,
                     recovery.get('phase'), recovery.get('reason'), recovery.get('retreat_attempts'),
                     checkpoint.get('used'), checkpoint.get('outcome'))
        if signature == previous:
            continue
        previous = signature
        localization = row.get('localization') or {}
        events.append(dict(time_s=row['time_s'], state=row.get('state'),
            decision_reason=row.get('decision_reason'), map_revision=revision,
            recovery=recovery, estimated_position=localization.get('position'),
            localization_checkpoints=checkpoint or None,
            support_diagnosis=row.get('support_diagnosis'),
            position_std_m=localization.get('position_std_m'),
            scan_rmse_m=localization.get('scan_rmse_m'),
            localization_margin_m=row.get('localization_margin_m'),
            supported_graph_nodes=row.get('supported_graph_nodes'),
            planned_path_points=len(row.get('planned_path') or []),
            return_path_points=len(row.get('return_path') or [])))
    return events


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
        trace_bytes = (run/'trace.jsonl').read_bytes()
        trace = [json.loads(line) for line in trace_bytes.splitlines() if line.strip()]
        returning = next((row for row in trace if row['state'].startswith('return') and row['state'] != 'returned'), None)
        forecast = None
        if returning and summary.get('clean_return'):
            forecast = dict(first_return_plan_s=returning['time_s'],
                forecast_s=returning.get('route_forecast', {}).get('return_duration_s'),
                actual_remaining_s=summary['simulation']['simulation_time_s']-returning['time_s'])
        keys = ('clean_return', 'round_trip_completed', 'round_trip_failures', 'landmark_mission_completed',
                'landmarks', 'error', 'true_dose_usv', 'path_length_m', 'mapped_surface_area_m2',
                'turning_in_place_s', 'target_switches', 'nonfoot_contacts', 'nonfoot_impulse_ns',
                'nonfoot_peak_force_n', 'worker_p95_ms',
                'worker_phase_p95_ms', 'wall_time_s', 'physical_witness', 'region_agenda', 'source_sha256',
                'localization', 'localization_evaluation', 'gp', 'information_contract', 'map_consistency',
                'recovery', 'terminal_stop', 'post_stop_true_dose_usv', 'count_calibration', 'circuit_mission_completed',
                'tracking_clearance_margin_m', 'localization_margin_m', 'terrain', 'localization_checkpoints')
        report['missions'].append(dict(run=run.name, summary_sha256=summary_hash,
            trace_sha256=hashlib.sha256(trace_bytes).hexdigest(),
            config_sha256=config_hash, configuration={k: config.get(k) for k in
                ('world', 'seed', 'layout_seed', 'duration', 'dose_budget', 'gp_mode', 'path_mode',
                 'mission_objective', 'region_scheduling', 'fixed_plan_delay', 'ros', 'fast',
                 'localization', 'sensor_seed', 'detector_likelihood', 'navigation_profile', 'utility_mode',
                 'lidar_dropout_after', 'circuit_radius', 'require_circuit', 'slam_backend',
                 'mezzanine_angle', 'uncertainty_profile', 'require_round_trip', 'terminal_hold', 'support_fusion', 'record_support', 'loop_search_radius', 'localization_checkpoints', 'keyframe_retention', 'registration_sampling')},
            terminal_stance=summary['simulation'].get('terminal_stance'),
            post_stop_motion=terminal_motion(run, summary),
            simulated_time_s=summary['simulation']['simulation_time_s'], return_forecast=forecast,
            navigation_transitions=navigation_transitions(trace),
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
