#!/usr/bin/env python3
"""Fail if a run lacks physical success or the required ROS publications."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path, nargs='?')
parser.add_argument('--require-navigation', action='store_true')
args = parser.parse_args()
run = args.run or max((p for p in (Path(__file__).resolve().parents[1]/'results/dougie').iterdir()
                      if p.is_dir() and (p/'simulation.json').exists()), key=lambda p:p.stat().st_mtime)
physics = json.loads((run/'simulation.json').read_text())
interfaces = json.loads((run/'interfaces.json').read_text())
assert physics['error'] is None, physics
assert physics['upright'] > .8 and physics['joints'] == 12, physics
for topic in ('/odom', '/scan', '/joint_states', '/imu'):
    assert interfaces['counts'].get(topic, 0) >= 3, (topic, interfaces['counts'])
assert interfaces['latest']['joint_count'] == 12
if args.require_navigation:
    assert interfaces['counts'].get('/map', 0) >= 3
    assert any(event['state'] == 'arrived' for event in interfaces['exploration_events']), interfaces['exploration_events']
print(f'Go2 run verified: {run}')
