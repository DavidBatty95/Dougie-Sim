#!/usr/bin/env python3
"""Read-only paired campaign holdout report; preserve incomplete outcomes."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path


def iter_rows(path):
    if not path.exists():
        return
    with path.open() as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                break


def rows(path):
    return list(iter_rows(path))


def inspect(directory):
    manifest_path=directory/'comparison_session.json'
    manifest=json.loads(manifest_path.read_text())
    campaign=manifest['campaign']
    config=json.loads((directory/'comparison_config.json').read_text())
    result=dict(directory=str(directory),status=campaign['status'],reason=campaign['reason'],
        requested_sorties=campaign['max_sorties'],completed_sorties=len(campaign['history']),
        censored=campaign['status'] in ('active','halted','sortie_limit'),modes={},
        settings={k:config.get(k) for k in ('world','layout_seed','sensor_seed','seed','duration',
            'campaign_sorties','lifetime_dose','survival_fraction','localization','campaign_strategy')})
    hashes=set()
    for record in manifest['runs']:
        if record.get('campaign_id')!=campaign['id']:
            continue
        hashes.add(record.get('learning_protocol_sha256'))
    result['protocol_sha256']=sorted(x for x in hashes if x)
    for mode in ('blind','aware'):
        state=campaign['modes'][mode]
        trajectory=[]
        active_last=None
        sorties=[]
        for record in manifest['runs']:
            if record.get('campaign_id')!=campaign['id']:
                continue
            run=directory/record['directory']/mode
            telemetry=rows(run/'comparison.jsonl')
            if record['status']=='running' and telemetry:
                active_last={k:telemetry[-1].get(k) for k in ('time_s','campaign_time_s','campaign_dose_usv',
                    'explored_percent','state','frontier_count')}
                active_last['telemetry_age_s']=max(0.,time.time()-(run/'comparison.jsonl').stat().st_mtime)
            summary_path=run/'radiation_summary.json'
            summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
            trajectory.extend(telemetry)
            candidates=0
            selections=[]
            previous_reason=None
            for r in iter_rows(run/'trace.jsonl'):
                candidates+=bool(r.get('uncertainty_probe_count'))
                target=r.get('selected_target')
                if selections and selections[-1]['cutoff'] is None:
                    goal=selections[-1]['goal']
                    if (r.get('state','').startswith('return') or target is None
                            or goal is not None and math.dist(target,goal)>.35):
                        selections[-1]['cutoff']=r['time_s']
                reason=r.get('decision_reason')
                if reason=='uncertainty_measurement_probe' and previous_reason!=reason:
                    selections.append(dict(selected_s=r['time_s'],goal=target,cutoff=None))
                previous_reason=reason
            measurements=rows(run/'measurements.jsonl')
            probes=[]
            for selected in selections:
                goal=selected['goal']
                cutoff=selected['cutoff'] if selected['cutoff'] is not None else math.inf
                relevant=[s for s in measurements if selected['selected_s']<=s.get('stamp_s',0)<=cutoff and s.get('position_valid',True)]
                distance=min((math.dist([s['position'][0],s['position'][1],s['position'][2]-.44],goal)
                    for s in relevant),default=None) if goal else None
                probes.append(dict(selected_s=selected['selected_s'],goal=goal,minimum_distance_m=distance,
                    reached_25cm=distance is not None and distance<=.25,
                    scope='valid detector-window position minus nominal 0.44 m support height, before goal change/return; not an independent foot-contact witness'))
            sorties.append(dict(id=record['id'],state=record['status'],exit_code=record['exit_codes'].get(mode),
                end_state=summary.get('end_state'),error=summary.get('error'),frontier_complete=summary.get('frontier_complete'),
                frontier_confirmation_count=summary.get('frontier_confirmation_count'),
                frontier_count=summary.get('frontier_count'),upper_landing_witness_s=(summary.get('physical_witness') or {}).get('upper_landing_witness_s'),
                reached_heights_m=(summary.get('comparison_survey') or {}).get('physically_reached_heights_m'),
                decision_reason=summary.get('decision_reason'),probe_candidate_plans=candidates,probes=probes))
        trajectory.sort(key=lambda r:r.get('campaign_time_s',0))
        milestones=[]
        for threshold in (25.,40.,50.,60.,70.,75.,80.,90.,95.):
            hit=next((r for r in trajectory if r.get('explored_percent',0)>=threshold),None)
            if hit:
                milestones.append(dict(threshold=threshold,observed_percent=hit['explored_percent'],
                    campaign_dose_usv=hit.get('campaign_dose_usv'),campaign_time_s=hit.get('campaign_time_s')))
        result['modes'][mode]=dict(**{k:state.get(k) for k in ('total_dose_usv','total_time_s','total_distance_m',
            'robots_lost','robots_recovered','sorties','explored_percent','last_outcome')},
            wrecks=len(state['wrecks']),milestones=milestones,sortie_outcomes=sorties,active_sortie=active_last,
            completion_censored=not any(s['frontier_complete'] is True and (s['frontier_confirmation_count'] or 0)>=3
                and s['end_state'] in ('returned','finished') and not s['error'] for s in sorties))
    result['manifest_sha256']=hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--studies',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    data=dict(scope='Fixed-layout/detector simulation campaigns; active or externally stopped attempts remain censored. No claims from unequal-work endpoints.',
        studies=[inspect(path) for path in args.studies])
    args.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    for s in data['studies']:
        aware=s['modes']['aware']
        print(f"{s['directory']}: {s['status']} {s['completed_sorties']}/{s['requested_sorties']} sorties; "
              f"finalized-sortie ledger {aware['explored_percent']:.2f}% / {aware['total_dose_usv']:.4f} µSv; "
              f"recovered {aware['robots_recovered']}, lost {aware['robots_lost']}; censored={s['censored']}")


if __name__=='__main__':
    main()
