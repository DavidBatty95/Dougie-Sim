#!/usr/bin/env python3
"""Read-only shadow evaluation of the production count-derived trend guard."""
import argparse
import json
from pathlib import Path

from dougie_sim.exposure_trend import ExposureTrend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements',type=Path)
    args = parser.parse_args()
    samples = [json.loads(line) for line in args.measurements.read_text().splitlines()]
    trend = ExposureTrend()
    warnings = []
    for sample in samples:
        trend.observe(sample)
        if trend.active:
            warnings.append(dict(time_s=sample['stamp_s'],position=sample['position'],rate=sample['rate_usv_h'],gradient=trend.gradient.tolist()))
    peaks = sorted(samples,key=lambda s:-s['rate_usv_h'])[:5]
    result = dict(warnings=len(warnings),peak_windows=[dict(time_s=s['stamp_s'],rate=s['rate_usv_h'],position=s['position'],
        prior_warnings=[w for w in warnings if s['stamp_s']-4 <= w['time_s'] < s['stamp_s']]) for s in peaks])
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
