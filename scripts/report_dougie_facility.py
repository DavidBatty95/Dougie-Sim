#!/usr/bin/env python3
"""Portable evidence for facility development and matched policy screening."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dougie_sim.mission_evaluation import round_trip_failures


def load(path):
    raw = path.read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--missions', nargs='+', type=Path, required=True)
    parser.add_argument('--qualifications', nargs='+', type=Path, default=[])
    parser.add_argument('--training', nargs='+', type=Path, default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--motion-benchmark', type=Path)
    parser.add_argument('--gp-benchmark', type=Path)
    parser.add_argument('--planning-benchmark', type=Path)
    args = parser.parse_args()
    report = dict(schema_version=1, scope='development and screening; not independent repeatability qualification',
                  missions=[], qualifications=[], training=[])
    for directory in args.missions:
        summary, digest = load(directory/'radiation_summary.json')
        config, config_digest = load(directory/'run_config.json')
        keys = ('end_state', 'error', 'true_dose_usv', 'budget_usv', 'clean_return', 'physical_witness',
                'exploration_complete', 'path_length_m', 'sampled_height_range_m', 'mapped_surface_area_m2',
                'nonfoot_impulse_ns', 'nonfoot_peak_force_n', 'performance', 'source_sha256',
                'command_total_variation', 'turning_in_place_s', 'gp', 'route_replan_rejections',
                'planning_p95_ms', 'worker_p95_ms', 'landmarks', 'landmark_mission_completed',
                'terrain', 'decision_reason', 'information_contract', 'route_forecast', 'exploration_agenda',
                'observed_support_visit_fraction', 'known_landmarks_pending', 'worker_phase_p95_ms', 'region_agenda')
        failures = round_trip_failures(summary)
        report['missions'].append(dict(run=directory.name, summary_sha256=digest, config_sha256=config_digest,
            round_trip_completed=not failures, round_trip_failures=failures,
            configuration={k: config.get(k) for k in ('world', 'layout_seed', 'seed', 'duration', 'dose_budget',
                'gp_capacity', 'fixed_plan_delay', 'require_clean', 'require_round_trip', 'policy', 'path_mode', 'gp_mode',
                'gp_expert_capacity', 'gp_max_experts', 'gp_cell_size', 'field_display', 'display_background',
                'landmark_count', 'landmark_seed', 'mission_objective', 'region_scheduling')},
            simulated_time_s=summary['simulation']['simulation_time_s'], **{k: summary.get(k) for k in keys}))
    for directory in args.qualifications:
        summary, digest = load(directory/'summary.json')
        report['qualifications'].append(dict(run=directory.name, summary_sha256=digest,
            policy=summary['policy'], seeds=summary['seeds'], complete=summary['complete'],
            trials=summary['completed_trials'], cases=summary['cases_summary']))
    for directory in args.training:
        manifest, digest = load(directory/'manifest.json')
        result_path = directory/'result.json'
        result = load(result_path)[0] if result_path.exists() else None
        report['training'].append(dict(run=directory.name, manifest_sha256=digest,
            arguments=manifest['arguments'], packages=manifest['packages'], device=manifest['device'],
            reward_foot_order=manifest.get('reward_foot_order'), result=result,
            source_sha256=manifest['source_sha256'],
            status='completed' if result is not None else 'interrupted; only retained checkpoints are evidence'))
    report['benchmarks'] = {}
    for name, path in (('motion', args.motion_benchmark), ('gp', args.gp_benchmark), ('planning', args.planning_benchmark)):
        if path:
            data, digest = load(path)
            report['benchmarks'][name] = dict(file=path.name, sha256=digest, report=data)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
