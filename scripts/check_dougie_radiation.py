#!/usr/bin/env python3
"""Verify radiation mission artifacts, optionally requiring a completed return."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dougie_sim.mission_evaluation import round_trip_failures

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path)
parser.add_argument('--require-return', action='store_true')
parser.add_argument('--require-round-trip', action='store_true')
parser.add_argument('--require-circuit', action='store_true')
args = parser.parse_args()
result = json.loads((args.run/'radiation_summary.json').read_text())
assert result['error'] is None, result
assert not result['actual_budget_exceeded'], result
assert result['simulation']['upright'] > .8
assert result['gp']['updates'] >= 1 and result['gp']['inducing_points'] <= result['gp']['capacity']
assert result['observed_free_volume_m3'] > 0
assert (args.run/'radiation_map_3d.npz').stat().st_size > 0
assert (args.run/'gp_posterior.npz').stat().st_size > 0
if args.require_return:
    assert result['mission_completed'] and result['end_state'] == 'returned'
if args.require_round_trip:
    failures = round_trip_failures(result)
    if failures:
        raise SystemExit('Round-trip verification failed: '+', '.join(failures))
if args.require_circuit:
    assert result.get('clean_return') and result.get('circuit_mission_completed')
    assert result.get('circuit', {}).get('completed')
    assert result.get('circuit_evaluation', {}).get('all_goals_reached')
    assert len(result['circuit_evaluation']['reached_s']) == 4
    assert all(t is not None for t in result['circuit_evaluation']['reached_s'])
print(f'Radiation run verified: {args.run}')
