#!/usr/bin/env python3
"""Publish every attempt in a dated transport development series, including failures."""
import argparse
from datetime import datetime,timezone
import json
import re
from pathlib import Path

from transport_evidence import digest

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'


def summarize(directory):
    row = dict(run_id=directory.name)
    for name in ('manifest','result','runtime_provenance','source_provenance','effective_parameters','response_diagnostics'):
        path = directory/(name+'.json')
        if path.is_file():
            data = json.loads(path.read_text())
            row[name+'_sha256'] = digest(path)
            if name == 'manifest':
                row['configuration'] = {key:data.get(key) for key in
                    ('world','people','seed','duration_s','build_reused','carry_posture')}
                row['source_sha256'] = data.get('source_sha256',{})
            elif name == 'result':
                row['outcome'] = {key:data.get(key) for key in
                    ('success','mission_complete','error','data_valid','deliveries','fallen',
                     'dropped_objects','contact_samples','attachment_state','object_may_be_held','evaluation_health')}
                events = data.get('events',[])
                row['last_phase'] = events[-1]['phase'] if events else None
                row['elapsed_to_last_event_s'] = events[-1]['elapsed_s'] if events else None
                row['recovery_events'] = [event for event in events if event['phase'] in
                    ('wait_for_clearance','redock_pickup') or 'dock_settled' in event]
            elif name == 'runtime_provenance':
                row['python_source_alignment'] = data.get('python_source_alignment')
            elif name == 'effective_parameters':
                row['effective_parameters'] = dict(required_complete=data.get('required_complete'),
                    incomplete_required_nodes=data.get('incomplete_required_nodes'),node_count=len(data.get('nodes',{})))
            elif name == 'response_diagnostics':
                row['response_diagnostics'] = dict(planner_period=data.get('planner_period'),
                    planner_active_period=data.get('planner_active_period'),
                    metrics=data.get('metrics'),malformed_lines=data.get('malformed_lines'))
    row['incomplete_artifacts'] = not all(key in row for key in ('manifest_sha256','result_sha256'))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',required=True,help='UTC date or timestamp prefix; includes every matching attempt')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--after',help='Optional exclusive UTC run-id lower bound (YYYYMMDDTHHMMSSZ)')
    args = parser.parse_args()
    if not args.prefix.isdigit() or len(args.prefix) != 8:
        parser.error('--prefix must be a YYYYMMDD UTC date')
    if args.after is not None and not re.fullmatch(r'\d{8}T\d{6}Z',args.after):
        parser.error('--after must be a YYYYMMDDTHHMMSSZ run ID')
    runs = [summarize(path) for path in sorted(RESULTS.glob(args.prefix+'*'))
            if path.is_dir() and (args.after is None or path.name>args.after)]
    report = dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),
        role='development_attempt_ledger_not_frozen_reliability_cohort',
        source='ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo',
        selection=dict(prefix=args.prefix,after=args.after),runs=runs)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(attempts=len(runs),successes=sum(bool(row.get('outcome',{}).get('success')) for row in runs),
                         incomplete=sum(row['incomplete_artifacts'] for row in runs),output=str(args.output))))


if __name__ == '__main__':
    main()
