#!/usr/bin/env python3
"""Publish retained functional campaign checks without claiming equal-work efficacy."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def read(path):
    return json.loads(path.read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--death',type=Path,required=True)
    parser.add_argument('--recovery',type=Path,required=True)
    parser.add_argument('--reserve',type=Path,required=True)
    parser.add_argument('--wreck-sortie',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    death=read(args.death/'comparison_session.json')
    death=death['campaign_archives'][0] if death['campaign_archives'] else death['campaign']
    recovered=read(args.recovery/'comparison_session.json')['campaign']
    reserve=read(args.reserve/'comparison_session.json')['campaign']
    bodies=[b.get('name') for b in ET.parse(args.wreck_sortie/'scene.xml').findall('.//body')
        if b.get('name','').startswith('wreck_') and b.get('name','').endswith('_base_link')]
    assert all(death['modes'][m]['robots_lost']==1 for m in ('blind','aware'))
    assert all(recovered['modes'][m]['robots_recovered']==2 for m in ('blind','aware'))
    assert bodies
    report=dict(scope='Functional physics, hand-off and display checks plus one illustrative sortie. Not a multi-seed, equal-work or hardware-tolerance validation.',
        failure_check=dict(directory=str(args.death),limit_usv=death['limit'],status=death['status'],reason=death['reason'],
            outcomes={m:{k:v for k,v in death['modes'][m].items() if k not in ('wrecks','previous_directory')} for m in ('blind','aware')}),
        recovery_check=dict(directory=str(args.recovery),status=recovered['status'],sorties=len(recovered['history']),
            outcomes=recovered['history']),
        collision_import_check=dict(directory=str(args.wreck_sortie),retained_wreck_bodies=bodies,
            scope='Earlier tight-budget development run used only to verify that actual failure geometry appears in the following physical scene.'),
        indicative_default_budget_sortie=dict(directory=str(args.reserve),limit_usv=reserve['limit'],
            outcomes=reserve['history'],
            interpretation='Aware survival and blind loss occurred with unequal exploration. This is not proof of equal-work dose improvement.'),
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (args.death/'comparison_session.json',args.recovery/'comparison_session.json',args.reserve/'comparison_session.json',args.wreck_sortie/'scene.xml')})
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k in ('scope','collision_import_check')},indent=2))


if __name__=='__main__':
    main()
