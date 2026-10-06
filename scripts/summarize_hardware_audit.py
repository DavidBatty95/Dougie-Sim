#!/usr/bin/env python3
"""Publish portable copies of hardware-contract audit reports with source hashes."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    result = dict(schema_version=1,scope='simulation hardware-contract audit; no physical hardware measurements',reports=[])
    for path in args.reports:
        raw = path.read_bytes()
        result['reports'].append(dict(source=str(path.resolve().relative_to(ROOT)),sha256=hashlib.sha256(raw).hexdigest(),
                                      report=json.loads(raw)))
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(f'Published {len(result["reports"])} reports to {args.output}')


if __name__=='__main__':
    main()
