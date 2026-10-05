#!/usr/bin/env python3
"""Compact command-chain trace for diagnosing a recorded avoidance run."""
import argparse
import json
import math
from collections import Counter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--interval', type=float, default=5.)
    parser.add_argument('--start', type=float, default=0., help='First elapsed second to display')
    parser.add_argument('--end', type=float, default=math.inf, help='Last elapsed second to display')
    parser.add_argument('--candidates', action='store_true', help='Include candidate rejection counts and planning timing')
    parser.add_argument('--candidate-limit', type=int, default=0, help='Show only this many lowest-score candidates (0 shows all)')
    parser.add_argument('--tracks', action='store_true', help='Include the latest sensed track snapshot')
    parser.add_argument('--evaluation-people', action='store_true', help='Include offline scoring truth for diagnosis; frames may differ')
    parser.add_argument('--evaluation-robot', action='store_true', help='Include offline root pose for diagnosis; never a navigation input')
    args = parser.parse_args()
    pose, safety, people, start, previous = {}, {}, {}, None, -1e30
    previous_sim_time, previous_people_stamp = None, None
    tracker = {}
    evaluation_robot = {}
    print('seconds state                  reference(x,y,w)       selected(x,y,w)         policy(x,y,w)          sensed(x,y,yaw)       closest(d,tcpa,v) safety')
    with open(args.log) as stream:
        for line in stream:
            row = json.loads(line)
            data = row.get('data', {})
            if row.get('topic') == 'odom':
                pose = data
            if row.get('topic') == 'evaluation_odom':
                evaluation_robot = data
            if row.get('topic') == 'safety_debug':
                safety = data
            if row.get('topic') == 'mujoco_people':
                people = data
            if row.get('topic') == 'tracker_debug':
                tracker = data
            if row.get('topic') != 'planner_debug' or not isinstance(data, dict):
                continue
            stamp = row['stamp']
            if start is None:
                start = stamp
            if stamp-start < args.start:
                continue
            if stamp-start > args.end:
                break
            if stamp-previous < args.interval:
                continue
            previous = stamp
            def vector(value):
                if isinstance(value, dict):
                    value = [value.get(k, 0) for k in ('x', 'y', 'z')]
                return ','.join(f'{v:+.2f}' for v in (value or [0,0,0]))
            qx, qy, qz, qw = [pose.get(k, 0.) for k in ('qx', 'qy', 'qz', 'qw')]
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            person = data.get('closest_track') or {}
            sim_time, people_stamp = people.get('sim_time'), people.get('stamp')
            rtf = None
            if sim_time is not None and previous_sim_time is not None and people_stamp > previous_people_stamp:
                rtf = (sim_time-previous_sim_time)/(people_stamp-previous_people_stamp)
            previous_sim_time, previous_people_stamp = sim_time, people_stamp
            print(f'{stamp-start:6.1f} {data.get("state", ""):<22} {vector(data.get("reference_cmd")):<22} '
                  f'{vector(data.get("selected_cmd")):<22} {vector(data.get("policy_command")):<22} '
                  f'{pose.get("x",0):+.2f},{pose.get("y",0):+.2f},{yaw:+.2f} '
                  f'{person.get("distance")},{person.get("tcpa")},{math.hypot(person.get("vx",0),person.get("vy",0)):.2f} '
                   f'{safety.get("mode", "")}/{safety.get("closest_risk", "")} RTF={rtf}')
            if args.tracks:
                print(json.dumps(tracker, sort_keys=True))
            if args.evaluation_people:
                print(json.dumps({'evaluation_people':people},sort_keys=True))
            if args.evaluation_robot:
                print(json.dumps({'evaluation_robot':evaluation_robot},sort_keys=True))
            if args.candidates:
                results = data.get('candidate_results', [])
                reasons = Counter(result.get('reason') or 'feasible' for result in results)
                if args.candidate_limit > 0:
                    results = sorted(results, key=lambda result: result.get('score') if result.get('score') is not None else math.inf)[:args.candidate_limit]
                print(json.dumps({'rejections': dict(reasons), 'candidates': results,
                                  'planning': {key: value for key, value in data.items()
                                               if 'budget' in key or 'planning' in key or 'static' in key or key == 'low_obstacle_points'},
                                  'selected': data.get('selected')}, sort_keys=True))


if __name__ == '__main__':
    main()
