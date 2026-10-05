#!/usr/bin/env python3
"""Compare legacy scoring samples with paired simulator truth, offline only."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def closest_stamp(states,stamp):
    if stamp is None or not states:
        return None
    result = min(states,key=lambda state:abs(state['stamp']-stamp))
    return result if abs(result['stamp']-stamp) < 1e-5 else None


def paired_clearance(states,start,end,robot_radius):
    """Swept minimum over a bracketed common-simulation-time interval."""
    minimum = math.inf
    cursor = start
    latest_wall = -math.inf
    if abs(end-start) < 1e-9:
        for state in states:
            if abs(state['sim_time']-start) < 1e-9:
                r = state['robot']
                minimum = min((math.hypot(r['x']-p['x'],r['y']-p['y'])-robot_radius-p['radius_m']
                               for p in state['people']),default=math.inf)
                return minimum,True,state['stamp']
    for a,b in zip(states,states[1:]):
        ta,tb = a['sim_time'],b['sim_time']
        if a.get('epoch') != b.get('epoch') or not 0. < tb-ta <= .2:
            continue
        low,high = max(start,ta),min(end,tb)
        if high < low or low > cursor+1e-6:
            continue
        left,right = {p['id']:p for p in a['people']},{p['id']:p for p in b['people']}
        if left.keys() != right.keys():
            continue
        for identity,p in left.items():
            q = right[identity]
            ax,ay = a['robot']['x']-p['x'],a['robot']['y']-p['y']
            bx,by = b['robot']['x']-q['x'],b['robot']['y']-q['y']
            u,v = (low-ta)/(tb-ta),(high-ta)/(tb-ta)
            x,y = ax+u*(bx-ax),ay+u*(by-ay)
            dx,dy = (v-u)*(bx-ax),(v-u)*(by-ay)
            fraction = max(0.,min(1.,-(x*dx+y*dy)/max(1e-12,dx*dx+dy*dy)))
            minimum = min(minimum,math.hypot(x+fraction*dx,y+fraction*dy)-robot_radius-max(p['radius_m'],q['radius_m']))
        cursor = max(cursor,high)
        latest_wall = max(latest_wall,b['stamp'])
        if cursor >= end-1e-6:
            break
    return minimum,cursor >= end-1e-6,latest_wall


def audit(path,robot_radius=.35,speed_limit=.9):
    states,metrics = [],[]
    with Path(path).open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['topic'] == 'evaluation_aligned':
                states.append(row['data'])
            elif row['topic'] == 'benchmark_metrics':
                metrics.append((row['stamp'],row['data']))
    collision_reports,speed_examples = [],[]
    previous_collisions = 0
    for logged,data in metrics:
        sample = data.get('scoring_sample')
        if not sample:
            continue
        interval = sample.get('velocity_interval')
        if interval:
            a,b = (closest_stamp(states,t) for t in interval)
            if a is not None and b is not None and b['sim_time'] > a['sim_time']:
                distance = math.hypot(b['robot']['x']-a['robot']['x'],b['robot']['y']-a['robot']['y'])
                physical = distance/(b['sim_time']-a['sim_time'])
                legacy = math.hypot(*sample['robot_velocity'])
                if legacy > speed_limit+.001 and physical <= speed_limit+.001:
                    speed_examples.append(dict(legacy_speed=legacy,simulation_time_speed=physical,
                        native_speed=math.hypot(b['robot']['vx'],b['robot']['vy']),interval=interval))
        collisions = data.get('collisions',0)
        if collisions > previous_collisions:
            root = closest_stamp(states,sample.get('robot_stamp'))
            people_time = sample.get('people_sim_time')
            report = dict(legacy_collisions=collisions,robot_stamp=sample.get('robot_stamp'),
                          people_stamp=sample.get('people_stamp'),comparison_covered=False)
            x,y = sample['robot_xy']
            report['legacy_sample_clearance'] = min((math.hypot(x-p[1],y-p[2])-robot_radius-p[3]
                                                     for p in sample['people']),default=None)
            if root is not None and people_time is not None:
                same_epoch = [state for state in states if state.get('epoch') == root.get('epoch')]
                low,high = sorted((root['sim_time'],people_time))
                clearance,covered,last_wall = paired_clearance(same_epoch,low,high,robot_radius)
                report.update(paired_min_clearance=clearance if math.isfinite(clearance) else None,
                              comparison_covered=covered,bracket_available_before_report=last_wall <= logged,
                              simulation_interval=[low,high])
            collision_reports.append(report)
        previous_collisions = max(previous_collisions,collisions)
    paired = [(b['sim_time']-a['sim_time'],b['stamp']-a['stamp']) for a,b in zip(states,states[1:])
              if a.get('epoch') == b.get('epoch') and b['stamp'] > a['stamp'] and b['sim_time'] > a['sim_time']]
    return dict(role='evaluation_diagnostic_only',source_log=str(path),
        simulation_to_wall_ratio=sum(p[0] for p in paired)/sum(p[1] for p in paired) if paired else None,
        source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        robot_radius_m=robot_radius,speed_limit_mps=speed_limit,aligned_samples=len(states),
        dropped_unaligned_samples=states[-1].get('dropped_unaligned_samples') if states else None,
        collision_reports=collision_reports,wall_time_only_speed_exceedances=speed_examples[:10],
        note='No scoring thresholds or benchmark outcomes are changed by this diagnostic.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    report = json.dumps(audit(args.log),indent=2)+'\n'
    if args.output:
        if args.output.resolve() == args.log.resolve():
            parser.error('output must not overwrite the source trace')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(report)
    print(report,end='')
