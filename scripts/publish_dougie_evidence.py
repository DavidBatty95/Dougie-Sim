#!/usr/bin/env python3
"""Export compact, source-attributed results without raw caches or policies."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runs', nargs='+', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
records = []
for directory in args.runs:
    path = directory/'radiation_summary.json'
    record = json.loads(path.read_text())
    record.pop('events', None)
    record['run'] = str(directory)
    record['artifact_sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(directory.iterdir()) if p.is_file()}
    records.append(record)
result = dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    scope='initial native Go2 simulation qualification; one facility, three detector-noise seeds; not a SOTA comparison',
    docker_image=subprocess.check_output(['docker', 'image', 'inspect', 'dougie-sim:humble', '--format', '{{.Id}}'], text=True).strip(),
    radiation_runs=records,
    gp_benchmark=json.loads((ROOT/'results/dougie/gp_benchmark.json').read_text()),
    stairs=json.loads((ROOT/'results/dougie/stair_traversal.json').read_text()),
    uneven_ground=json.loads((ROOT/'results/dougie/uneven_terrain.json').read_text()))
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2)+'\n')
print(args.output)
