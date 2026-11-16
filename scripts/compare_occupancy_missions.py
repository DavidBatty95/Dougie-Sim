#!/usr/bin/env python3
"""Compare fixed-delay physical missions across occupancy-index revisions."""
import argparse
import json
from pathlib import Path


def read_rows(path):
    rows=[]
    with path.open() as stream:
        for line in stream:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                break
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before',type=Path)
    parser.add_argument('after',type=Path)
    parser.add_argument('--milestone',type=float,default=60.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    configs=[json.loads((p/'run_config.json').read_text()) for p in (args.before,args.after)]
    fields=('world','layout_seed','seed','sensor_seed','localization','gp_mode','exposure_policy',
        'campaign_strategy','frontier_view_policy','frontier_rate_limit','lifetime_dose','survival_fraction',
        'fixed_plan_delay','duration','radiation_awareness')
    differences={k:[c.get(k) for c in configs] for k in fields if configs[0].get(k)!=configs[1].get(k)}
    if differences:
        raise ValueError(f'mission settings differ: {differences}')
    records=[read_rows(p/'comparison.jsonl') for p in (args.before,args.after)]
    hits=[next((r for r in rows if r['explored_percent']>=args.milestone),None) for rows in records]
    sources=[c.get('source_sha256',{}) for c in configs]
    changed=sorted(k for k in set(sources[0])|set(sources[1]) if sources[0].get(k)!=sources[1].get(k))
    report=dict(scope='Same-seed fixed-delay development replay. Both runs can be incomplete; this checks behavior, not independent efficacy.',
        before=str(args.before),after=str(args.after),settings={k:configs[0].get(k) for k in fields},
        milestone=args.milestone,crossings=[{k:r.get(k) for k in ('time_s','explored_percent','true_dose_usv','distance_m','frontier_count')} if r else None for r in hits],
        states=[r[-1].get('state') if r else 'missing telemetry' for r in records],
        changed_source_files=changed,source_sha256=sources)
    if all(hits):
        report['differences']={k:hits[1][k]-hits[0][k] for k in ('time_s','explored_percent','true_dose_usv','distance_m','frontier_count')}
        horizon=min(hits[0]['time_s'],hits[1]['time_s'])
        timed=[{round(r['time_s'],3):r for r in run if r['time_s']<=horizon+1e-6} for run in records]
        common=sorted(set(timed[0])&set(timed[1]))
        report['common_time_samples']=len(common)
        report['maximum_absolute_difference']={key:max((abs(timed[1][t][key]-timed[0][t][key])
            for t in common if isinstance(timed[0][t].get(key),(int,float)) and isinstance(timed[1][t].get(key),(int,float))),default=None)
            for key in ('explored_percent','true_dose_usv','distance_m','frontier_count')}
    payload=json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(payload)
        print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','settings')},indent=2))
    else:
        print(payload)


if __name__=='__main__':
    main()
