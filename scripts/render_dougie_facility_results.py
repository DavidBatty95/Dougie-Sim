#!/usr/bin/env python3
"""Portable, source-attributed SVG telemetry figure (not a simulator screenshot)."""
import argparse
import hashlib
from html import escape
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', nargs='+', type=Path, help='replace portable input using retained run archives')
    parser.add_argument('--data', type=Path, default=ROOT/'docs/media/dougie/facility-telemetry.json')
    parser.add_argument('--output', type=Path, default=ROOT/'docs/media/dougie/facility-results.svg')
    args = parser.parse_args()
    if args.runs:
        runs = []
        for directory in args.runs:
            raw = (directory/'measurements.jsonl').read_bytes()
            measurements = [json.loads(line) for line in raw.splitlines()]
            summary_raw = (directory/'radiation_summary.json').read_bytes()
            summary = json.loads(summary_raw)
            step = max(1, len(measurements)//400)
            sampled = measurements[::step]
            if sampled[-1] != measurements[-1]:
                sampled.append(measurements[-1])
            runs.append(dict(run=directory.name, measurement_sha256=hashlib.sha256(raw).hexdigest(),
                summary_sha256=hashlib.sha256(summary_raw).hexdigest(), seed=summary['seed'],
                round_trip=summary.get('round_trip_completed', False),
                physical_witness=summary['physical_witness'], clean_return=summary['clean_return'],
                duration_s=summary['simulation']['simulation_time_s'], dose_usv=summary['true_dose_usv'],
                points=[[r['stamp_s'], *r['position']] for r in sampled]))
        args.data.write_text(json.dumps(dict(scope='downsampled actual detector positions; not video or complete coverage', runs=runs), indent=2)+'\n')
    data = json.loads(args.data.read_text())
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="680" viewBox="0 0 1200 680">',
           '<title>Full-height MuJoCo facility mission telemetry</title>',
           '<desc>Recorded detector heights and XY paths, including the initial clean return without an upper-floor witness.</desc>',
           '<rect width="1200" height="680" fill="#0b1828"/>',
           '<g font-family="DejaVu Sans,sans-serif" fill="#e2edf7">']
    def text(x, y, value, size=16, color='#e2edf7'):
        svg.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}">{escape(value)}</text>')
    text(45, 44, 'FULL-HEIGHT FACILITY / RECORDED MUJOCO TELEMETRY', 25)
    text(45, 76, 'Smooth tracking + spatial GP · baseline gait · layout 0 · 5 µSv budget', 16, '#91aec5')
    text(55, 120, 'Detector height over simulated time', 18)
    text(780, 120, 'Actual sampled XY trajectories', 18)
    for height in (0., 1., 2., 3.):
        y = 440-height*90
        svg.append(f'<path d="M70 {y} H690" stroke="#294058"/>')
        text(30, y+5, f'{height:g} m', 13)
    for seconds in (0, 600, 1200, 1800, 2400):
        x = 70+seconds/2400*620
        svg.append(f'<path d="M{x} 155 V440" stroke="#203348"/>')
        text(x-12, 467, str(seconds), 13)
    text(300, 493, 'Simulated time (s)', 14)
    for x in (0, 5, 10, 15):
        px = 800+(x+3)*18
        svg.append(f'<path d="M{px} 150 V440" stroke="#203348"/>')
        text(px-5, 467, str(x), 13)
    text(945, 493, 'X (m)', 14)
    colors = ['#8395a7', '#3ad5c0', '#ffbb57']
    for i, run in enumerate(data['runs']):
        color = colors[i % len(colors)]
        points = run['points']
        height_line = ' '.join(f'{70+p[0]/2400*620:.2f},{440-p[3]*90:.2f}' for p in points)
        xy_line = ' '.join(f'{800+(p[1]+3)*18:.2f},{440-(p[2]+4)*18:.2f}' for p in points)
        for line in (height_line, xy_line):
            svg.append(f'<polyline points="{line}" fill="none" stroke="{color}" stroke-width="2" opacity=".85"/>')
        verdict = 'UPSTAIRS + DOWNSTAIRS + HOME' if run['round_trip'] else 'NO VERTICAL ROUND TRIP'
        text(55, 535+i*30, f'Seed {run["seed"]} · {verdict} · {run["duration_s"]:.1f} s · {run["dose_usv"]:.3f} µSv', 16, color)
    text(55, 648, 'Development runs, not coverage certification. Height is the detector position; XY projection overlaps floors.', 14, '#91aec5')
    svg.extend(['</g>', '</svg>'])
    args.output.write_text('\n'.join(svg)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
