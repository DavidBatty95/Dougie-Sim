#!/usr/bin/env python3
"""Retain complete policy pilot statistics without hiding failed revisions."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--studies',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    studies = []
    lines = ['# Exposure-policy pilot results','',
        'Each row compares awareness with the same-revision blind ablation. Revisions and seed sets differ; before/after totals are not paired causal estimates.', '',
        '| Pilot | Policy | Terminal / requested | Censored | Frontier complete blind / aware | ≥50% eligible pairs | Blind mean dose | Aware mean dose | Paired aware−blind 95% mean interval |',
        '|---|---|---:|---:|---|---:|---:|---:|---|']
    fmt = lambda value:'—' if value is None else f'{value:.6f}'
    for directory in args.studies:
        config = json.loads((directory/'batch_config.json').read_text())
        summary = json.loads((directory/'batch_summary.json').read_text())
        notes_path = directory/'validation_notes.json'
        notes = json.loads(notes_path.read_text()) if notes_path.exists() else dict(valid_for_performance_claims=True)
        if summary['terminal_pairs'] != summary['requested_pairs'] and notes['valid_for_performance_claims']:
            raise ValueError(f'{directory}: requested seeds have not all reached terminal status')
        study = dict(name=directory.name,configuration=config,statistics=summary,validation=notes,
            input_sha256={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in
                ('batch_config.json','batch_summary.json','runs.csv','source_snapshot.tar.gz')})
        studies.append(study)
        if not notes['valid_for_performance_claims']:
            lines.append(f'| {directory.name} | INVALID pilot: {notes["reason"]} | — | — | — | — | — | — | — |')
            continue
        milestone = next((m for m in summary['milestones'] if m['reference_percent']==50.),None)
        blind,aware = summary['modes']['blind'],summary['modes']['aware']
        interval = milestone['paired_aware_minus_blind_dose']['mean_ci95'] if milestone else None
        ci = f'[{fmt(interval[0])}, {fmt(interval[1])}]' if interval else 'unavailable'
        lines.append(f'| {directory.name} | {config["protocol"].get("exposure_policy","standard")} | {summary["terminal_pairs"]}/{summary["requested_pairs"]} | {summary["censored_pairs"]} | {blind["confirmed_frontier_completions"]} / {aware["confirmed_frontier_completions"]} | {milestone["eligible_pairs"] if milestone else 0} | {fmt(milestone["blind_dose"]["mean"]) if milestone else "—"} | {fmt(milestone["aware_dose"]["mean"]) if milestone else "—"} | {ci} |')
    lines += ['', 'Dose units are µSv. Negative paired differences favour awareness. Intervals are percentile bootstrap mean intervals over eligible paired seeds, not safety bounds. Actual milestone overshoots and all individual crossings are retained in the JSON.', '',
        'All trials here are development/pilot experiments on declared fixed layouts. External wall guards are censoring, not successful frontier completion. Lower dose after less progress or earlier termination is not evidence of equal-work improvement. Missing higher milestones and failed/invalid revisions remain visible.', '']
    args.output.write_text(json.dumps(dict(scope='Development policy pilots; no multi-layout or hardware performance claim.',studies=studies),indent=2,allow_nan=False)+'\n')
    args.output.with_suffix('.md').write_text('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
