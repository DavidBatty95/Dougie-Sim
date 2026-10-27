#!/usr/bin/env python3
"""Publish all six paired smoke outcomes and regenerate README graphs from JSON."""
import argparse
import hashlib
from html import escape
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'docs/results/dougie_comparison_20261027.json'
MEDIA = ROOT/'docs/media/dougie'
COLORS = {'blind': '#ffb454', 'aware': '#48daca'}
WORLDS = ('reactor', 'mezzanine', 'facility')
METRIC = 'physically_reached_3d_floor_v2'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def collect(paths):
    experiments = []
    for path in paths:
        config = load(path/'comparison_config.json')
        if config.get('coverage_metric') != METRIC:
            raise ValueError(f'{path}: refusing to mix earlier coverage metrics')
        codes = load(path/'comparison_status.json')
        experiment = dict(world=config['world'], run=path.name, configuration=config, runs={})
        for mode in COLORS:
            directory = path/mode
            summary = load(directory/'radiation_summary.json')
            report = load(directory/'survey_report.json')
            rows = [json.loads(line) for line in (directory/'comparison.jsonl').read_text().splitlines() if line]
            if report.get('coverage_metric') != METRIC:
                raise ValueError(f'{directory}: inconsistent metric')
            with np.load(directory/'minimap_final.npz') as data:
                levels = data['slice_heights_m']
                bounds = (levels[1:]+levels[:-1])/2
                layers = np.searchsorted(bounds, data['xyz'][:, 2], side='right')
                slices = [dict(height_m=float(height), samples=int((layers == i).sum()),
                    explored_samples=int(data['seen'][layers == i].sum()),
                    explored_percent=float(100.*data['seen'][layers == i].mean())) for i, height in enumerate(levels)]
            experiment['runs'][mode] = dict(
                exit_code=codes[mode], configuration=load(directory/'run_config.json'),
                final=dict(time_s=rows[-1]['time_s'], dose_usv=summary['true_dose_usv'],
                    explored_percent=report['explored_percent'], end_state=summary['end_state'],
                    outcome=report['outcome'], reason=report['reason'], error=summary['error'],
                    clean_return=summary['clean_return'], path_length_m=summary['path_length_m'],
                    nonfoot_impulse_ns=summary['nonfoot_impulse_ns']),
                survey_report=report, slices=slices, samples=rows,
                input_sha256={name: sha(directory/name) for name in
                    ('run_config.json', 'radiation_summary.json', 'survey_report.json', 'comparison.jsonl', 'minimap_final.npz', 'frame.jpg')})
        experiment['comparison_config_sha256'] = sha(path/'comparison_config.json')
        experiments.append(experiment)
    if sorted(e['world'] for e in experiments) != sorted(WORLDS):
        raise ValueError('supply exactly one comparison for each world')
    return dict(date='2026-10-27', coverage_metric=METRIC,
        scope='Three paired development smoke runs, one pair per world, 60 s configured cap. Not full exploration or a matched-coverage dose-reduction study.',
        verification=dict(command='./dougie test', passed=355, date='2026-10-27',
                          scope='local full tests/dougie suite; not a claim that remote CI has run these changes'),
        experiments=sorted(experiments, key=lambda e: WORLDS.index(e['world'])))


def validate(data):
    if data['coverage_metric'] != METRIC:
        raise ValueError('unexpected coverage metric')
    for experiment in data['experiments']:
        settings = experiment['configuration']
        if (settings['duration'], settings['seed'], settings['sensor_seed'], settings['layout_seed']) != (60., 1, 0, 0):
            raise ValueError('this publication is the declared 60 s / 1,0,0 cohort')
        for mode, run in experiment['runs'].items():
            config = run['configuration']
            if config['radiation_awareness'] != mode or config['localization'] != 'fused':
                raise ValueError('unexpected awareness/localization mode')
            for key in ('world', 'duration', 'seed', 'sensor_seed', 'layout_seed', 'dose_budget', 'rate_limit'):
                if config[key] != settings[key]:
                    raise ValueError(f'worker/pair setting mismatch: {key}')
            rows, final = run['samples'], run['final']
            for a, b in zip(rows, rows[1:]):
                if any(b[key] < a[key]-1e-10 for key in ('time_s', 'true_dose_usv', 'explored_percent')):
                    raise ValueError(f'{experiment["world"]}/{mode}: nonmonotonic trace')
            if not all(math.isfinite(row[key]) for row in rows for key in ('time_s', 'true_dose_usv', 'explored_percent')):
                raise ValueError('nonfinite graph data')
            if not all(0 <= row['explored_percent'] <= 100 for row in rows):
                raise ValueError('invalid coverage')
            for key, column in (('dose_usv', 'true_dose_usv'), ('explored_percent', 'explored_percent')):
                if not math.isclose(final[key], rows[-1][column], abs_tol=1e-9):
                    raise ValueError('summary/trace disagreement')
            count = sum(s['samples'] for s in run['slices'])
            seen = sum(s['explored_samples'] for s in run['slices'])
            if not math.isclose(100.*seen/count, final['explored_percent'], abs_tol=1e-9):
                raise ValueError('slice/total coverage disagreement')


def text(x, y, label, size=14, color='#b8c8d8', anchor='start', weight='400'):
    return f'<text x="{x}" y="{y}" fill="{color}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}">{escape(str(label))}</text>'


def svg_start(width, height, title):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
            f'<title>{escape(title)}</title>', '<g font-family="DejaVu Sans,Arial,sans-serif">',
            f'<rect width="{width}" height="{height}" rx="16" fill="#0b1724"/>']


def plot(parts, experiment, x, y, key, title, coverage=False):
    width, height = 500, 170
    xmax = math.ceil(max(r['samples'][-1]['time_s'] for r in experiment['runs'].values())/10)*10
    maximum = max(row[key] for run in experiment['runs'].values() for row in run['samples'])
    ymax = 100. if coverage else max(.001, maximum*1.12)
    parts.append(text(x, y-15, title, 16, '#e7eef5', weight='600'))
    for i in range(5):
        yy, xx = y+height-i*height/4, x+width*i/4
        parts.append(f'<path d="M{x} {yy} H{x+width}" stroke="#2b3d50"/>')
        parts.append(text(x-10, yy+5, f'{ymax*i/4:.0f}' if coverage else f'{ymax*i/4:.4f}', 12, anchor='end'))
        parts.append(text(xx, y+height+20, f'{xmax*i/4:g}', 12, anchor='middle'))
    parts.append(text(x+width/2, y+height+39, 'Simulation time (s)', 12, anchor='middle'))
    for mode, run in experiment['runs'].items():
        points = ' '.join(f'{x+width*r["time_s"]/xmax:.3f},{y+height-height*r[key]/ymax:.3f}' for r in run['samples'])
        dash = ' stroke-dasharray="7 4"' if mode == 'blind' else ''
        parts.append(f'<polyline points="{points}" fill="none" stroke="{COLORS[mode]}" stroke-width="2.5"{dash}/>')
        last = run['samples'][-1]
        parts.append(f'<circle cx="{x+width*last["time_s"]/xmax:.3f}" cy="{y+height-height*last[key]/ymax:.3f}" r="3.5" fill="{COLORS[mode]}"/>')


def graphs(data):
    parts = svg_start(1240, 1000, 'Radiation-blind and aware comparison: true dose and physically reached 3D floor coverage')
    parts += [text(32, 42, 'Two robots. Three worlds. Measured trajectories.', 25, '#e7eef5', weight='700'),
              text(32, 70, '27 October 2026 / paired smoke runs / 60 s configured cap / detector 1, sensor 0, layout 0', 14),
              text(32, 99, '--- Radiation-blind', 14, COLORS['blind']), text(230, 99, '— Radiation-aware', 14, COLORS['aware'])]
    for i, experiment in enumerate(data['experiments']):
        y = 150+i*275
        parts.append(text(32, y, experiment['world'].upper(), 17, '#e7eef5', weight='700'))
        plot(parts, experiment, 90, y+40, 'true_dose_usv', 'Cumulative true dose (µSv)')
        plot(parts, experiment, 690, y+40, 'explored_percent', '3D floor exploration (%)', True)
    parts += [text(32, 968, 'Curves end at each mission endpoint. Early returns reflect the time reserve; these runs do not establish full coverage or dose savings.', 13), '</g></svg>']
    (MEDIA/'comparison-results.svg').write_text('\n'.join(parts)+'\n')
    parts = svg_start(1040, 470, 'Unvisited upper floors remain at zero exploration in the current 3D coverage metric')
    parts += [text(28, 40, 'Exploration follows the robot into 3D.', 25, '#e7eef5', weight='700'),
              text(28, 70, 'Final floor-slice coverage / fresh scans from physically reached support heights', 14)]
    for column, experiment in enumerate(data['experiments'][1:]):
        left = 28+column*510
        parts.append(text(left, 115, experiment['world'].upper(), 18, '#e7eef5', weight='700'))
        for index, layer in enumerate(experiment['runs']['blind']['slices']):
            y = 150+index*83
            parts.append(text(left, y, f'Z = {layer["height_m"]:.2f} m', 13))
            for offset, mode in enumerate(COLORS):
                percent = experiment['runs'][mode]['slices'][index]['explored_percent']
                yy = y+8+offset*22
                parts.append(f'<rect x="{left}" y="{yy}" width="390" height="13" rx="3" fill="#26374b"/>')
                if percent:
                    parts.append(f'<rect x="{left}" y="{yy}" width="{3.9*percent:.3f}" height="13" rx="3" fill="{COLORS[mode]}"/>')
                parts.append(text(left+402, yy+12, f'{percent:.1f}%', 13, COLORS[mode]))
    parts += [text(28, 425, 'Orange: blind   /   Teal: aware. Upper levels stayed at 0% because neither robot reached them in these short runs.', 13),
              text(28, 449, 'A floor visit enables fresh local-height observations; it never grants automatic whole-floor coverage.', 13), '</g></svg>']
    (MEDIA/'comparison-levels.svg').write_text('\n'.join(parts)+'\n')


def preview(paths):
    from PIL import Image, ImageDraw, ImageFont
    path = next(p for p in paths if load(p/'comparison_config.json')['world'] == 'facility')
    image = Image.new('RGB', (1320, 490), '#0b1724')
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    title = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 25)
    draw.text((20, 14), 'DOUGIE / TWO INDEPENDENT SIMULATIONS', font=title, fill='white')
    for i, mode in enumerate(COLORS):
        x = 12+i*656
        draw.text((x+8, 53), f'Radiation-{mode}', font=font, fill=COLORS[mode])
        with Image.open(path/mode/'frame.jpg') as frame:
            image.paste(frame.convert('RGB'), (x, 84))
    draw.text((20, 457), 'Facility / retained final MuJoCo frames / 3D coverage + sliced route minimaps / 60 s capped smoke run', font=font, fill='#b8c8d8')
    image.save(MEDIA/'comparison-preview.png')


def report(data):
    lines = ['# Current comparison results — 27 October 2026', '',
        '[Project home](../README.md) · [Comparison guide](DOUGIE_COMPARISON.md) · [Portable data](results/dougie_comparison_20261027.json)', '',
        '## Protocol', '', data['scope'], '',
        'All pairs use detector seed 1, sensor seed 0, layout seed 0, a 2 µSv dose budget, a 100 µSv/h rate limit, fused localization, and fixed 0.5 s simulated planning latency. The 60 s cap includes return reserve; runs can finish earlier.', '',
        'Coverage uses `physically_reached_3d_floor_v2`. All six outcomes are included. No curve is extended past the end of its run. These results verify comparison integration and coverage semantics; they do not establish a radiation-awareness advantage at matched work.', '',
        '## Per-run results', '',
        '| World | Mode | End (s) | Dose (µSv) | Explored (%) | Path (m) | End state / survey outcome |',
        '|---|---|---:|---:|---:|---:|---|']
    for experiment in data['experiments']:
        for mode, run in experiment['runs'].items():
            f = run['final']
            lines.append(f'| {experiment["world"]} | {mode} | {f["time_s"]:.3f} | {f["dose_usv"]:.6f} | {f["explored_percent"]:.2f} | {f["path_length_m"]:.3f} | {f["end_state"]} / {f["outcome"]} |')
    lines += ['', '![Dose and exploration traces](media/dougie/comparison-results.svg)', '',
        '![Per-level exploration](media/dougie/comparison-levels.svg)', '', '## Verification and provenance', '',
        '`./dougie test`: **355 passed** on 27 October 2026 (full local Go2 suite). This is not a new remote-CI or field-hardware result.', '',
        'The portable JSON retains time-series samples, configurations, outcomes, slice counts and SHA-256 identities of raw inputs. The media manifest records the exported JSON and image hashes. The preview is a composition of actual final renders, not a browser screenshot or a fabricated trajectory.', '',
        'Source comparisons:', '']
    lines.extend(f'- `{e["run"]}` — {e["world"]}' for e in data['experiments'])
    lines += ['', 'Regenerate both SVG graphs and this report from the committed portable data:', '',
        '```bash', 'docker compose run --rm -T dougie-sim python3 scripts/publish_dougie_comparison.py', '```', '',
        'To import fresh data and regenerate the preview, add `--runs` followed by exactly one complete comparison directory for each world. The publisher rejects old coverage metrics and mismatched protocol settings. See the script for the declared cohort.', '']
    (ROOT/'docs/DOUGIE_COMPARISON_RESULTS.md').write_text('\n'.join(lines))
    print('\n'.join(lines[lines.index('| World | Mode | End (s) | Dose (µSv) | Explored (%) | Path (m) | End state / survey outcome |'):][:8]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs=3)
    parser.add_argument('--check', action='store_true', help='validate portable data, media hashes and SVG syntax without writing')
    args = parser.parse_args()
    if args.check and args.runs:
        parser.error('--check validates the published portable report; omit --runs')
    data = collect(args.runs) if args.runs else load(REPORT)
    validate(data)
    if args.check:
        manifest = load(MEDIA/'comparison-manifest.json')
        for name, digest in manifest['assets'].items():
            path = ROOT/name
            if sha(path) != digest:
                raise ValueError(f'media hash mismatch: {name}')
            if path.suffix == '.svg':
                ET.parse(path)
        print('Verified all six outcomes, matched settings, monotonic traces, slice totals, media hashes and SVG syntax.')
        return
    if args.runs:
        REPORT.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
        preview(args.runs)
    graphs(data)
    report(data)
    files = [REPORT, MEDIA/'comparison-results.svg', MEDIA/'comparison-levels.svg', MEDIA/'comparison-preview.png']
    (MEDIA/'comparison-manifest.json').write_text(json.dumps(dict(
        date=data['date'], scope=data['scope'], generator='scripts/publish_dougie_comparison.py',
        assets={str(path.relative_to(ROOT)): sha(path) for path in files}), indent=2)+'\n')


if __name__ == '__main__':
    main()
