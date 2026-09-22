#!/usr/bin/env python3
"""Compact command-chain trace for diagnosing a recorded avoidance run."""
import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--interval', type=float, default=5.)
    args = parser.parse_args()
    pose, start, previous = {}, None, -1e30
    print('seconds state                  reference(x,y,w)       selected(x,y,w)         policy(x,y,w)          sensed(x,y,yaw)')
    with open(args.log) as stream:
        for line in stream:
            row = json.loads(line)
            data = row.get('data', {})
            if row.get('topic') == 'odom':
                pose = data
            if row.get('topic') != 'planner_debug' or not isinstance(data, dict):
                continue
            stamp = row['stamp']
            if start is None:
                start = stamp
            if stamp-previous < args.interval:
                continue
            previous = stamp
            def vector(value):
                if isinstance(value, dict):
                    value = [value.get(k, 0) for k in ('x', 'y', 'z')]
                return ','.join(f'{v:+.2f}' for v in (value or [0,0,0]))
            qx, qy, qz, qw = [pose.get(k, 0.) for k in ('qx', 'qy', 'qz', 'qw')]
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            print(f'{stamp-start:6.1f} {data.get("state", ""):<22} {vector(data.get("reference_cmd")):<22} '
                  f'{vector(data.get("selected_cmd")):<22} {vector(data.get("policy_command")):<22} '
                  f'{pose.get("x",0):+.2f},{pose.get("y",0):+.2f},{yaw:+.2f}')


if __name__ == '__main__':
    main()
