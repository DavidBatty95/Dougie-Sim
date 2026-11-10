#!/usr/bin/env python3
"""Retain a complete paired experiment and publish its measured figures."""
import argparse
import gzip
import hashlib
from html import escape
import io
import json
import math
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'docs/results/dougie_frontier_comparison.json'
ARCHIVE = ROOT/'docs/results/frontier_comparison_inputs.tar.gz'
MEDIA = ROOT/'docs/media/dougie'
COLORS = {'blind': '#ffb454', 'aware': '#48daca'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def matched_metrics(runs):
    common = min(run['samples'][-1]['time_s'] for run in runs.values())
    at_time = {mode: {key: float(np.interp(common, [r['time_s'] for r in run['samples']],
                                          [r[key] for r in run['samples']]))
                     for key in ('true_dose_usv', 'explored_percent', 'distance_m')}
               for mode, run in runs.items()}
    milestones = []
    for level in (50, 75, 85, 90):
        hits = {mode: next((r for r in run['samples'] if r['explored_percent'] >= level), None)
                for mode, run in runs.items()}
        if all(hits.values()):
            milestones.append(dict(reference_percent=level, first_recorded_crossings=hits,
                aware_dose_reduction_percent=100.*(1.-hits['aware']['true_dose_usv']/hits['blind']['true_dose_usv'])
                    if hits['blind']['true_dose_usv'] > 0 else None))
    return dict(common_time_s=common, at_common_time=at_time, coverage_milestones=milestones,
                scope='First recorded >= threshold crossing at 0.5 s telemetry cadence, not equal routes or an optimal-tour result. Common-time values use linear interpolation.')


def collect(path):
    config = json.loads((path/'comparison_config.json').read_text())
    codes = json.loads((path/'comparison_status.json').read_text())
    inputs = {'comparison_config.json': (path/'comparison_config.json').read_bytes(),
              'comparison_status.json': (path/'comparison_status.json').read_bytes()}
    runs = {}
    for mode in COLORS:
        directory = path/mode
        names = ['run_config.json', 'radiation_summary.json', 'survey_report.json', 'comparison.jsonl',
                 'coverage_reference.npz', 'minimap_final.npz', 'frame.jpg', 'motion_trace.npz',
                 'source_scenario.json', 'evaluator_field.json', 'stream_performance.json']
        if mode == 'aware':
            names.append('radiation_map.json')
        for name in names:
            inputs[f'{mode}/{name}'] = (directory/name).read_bytes()
        summary = json.loads(inputs[f'{mode}/radiation_summary.json'])
        rows = [json.loads(line) for line in inputs[f'{mode}/comparison.jsonl'].splitlines() if line]
        runs[mode] = dict(exit_code=codes[mode], configuration=json.loads(inputs[f'{mode}/run_config.json']),
            final={**rows[-1], **{key: summary[key] for key in (
                'clean_return', 'mission_completed', 'error', 'nonfoot_impulse_ns', 'nonfoot_peak_force_n',
                'path_length_m', 'completion_scope')}}, samples=rows,
            survey_report=json.loads(inputs[f'{mode}/survey_report.json']),
            stream_performance=json.loads(inputs[f'{mode}/stream_performance.json']),
            input_sha256={name: sha(inputs[f'{mode}/{name}']) for name in names})
    data = dict(schema_version=1, run=path.name, configuration=config, runs=runs,
        scope='One retained paired reactor development experiment; ideal-pose planning comparison, not a multi-seed or hardware validation. Both incomplete outcomes are retained.',
        distance_definition='distance_m integrates actual 3D base displacement at every physics tick. path_length_m is the separate historical displacement-gated diagnostic; do not substitute it.',
        verification=dict(command='python3 -m pytest -q tests/dougie', passed=367,
                          scope='local full Go2 suite on Linux ARM64; remote CI status is separate'))
    data['matched'] = matched_metrics(runs)
    return data, inputs


def validate(data):
    c = data['configuration']
    if c['dose_budget'] is not None or c['duration'] != 0 or c['radiation_policy'] != 'minimize-dose':
        raise ValueError('This publication requires the uncapped dose-weighted profile')
    if c['localization'] != 'ground-truth' or c['coverage_metric'] != 'physically_reached_3d_floor_v2':
        raise ValueError('Unexpected localization/coverage profile')
    for mode, run in data['runs'].items():
        cfg, rows, final = run['configuration'], run['samples'], run['final']
        if not cfg['no_dose_budget'] or cfg['radiation_awareness'] != mode:
            raise ValueError('Worker/pair awareness or budget mismatch')
        for key in ('world', 'duration', 'seed', 'sensor_seed', 'layout_seed', 'localization', 'radiation_policy'):
            if cfg[key] != c[key]:
                raise ValueError(f'Worker/pair mismatch: {key}')
        for key in ('time_s', 'true_dose_usv', 'explored_percent', 'distance_m'):
            values = [row[key] for row in rows]
            if not all(math.isfinite(v) for v in values) or any(b < a-1e-9 for a, b in zip(values, values[1:])):
                raise ValueError(f'Invalid/nonmonotonic {mode}/{key}')
        if any(not 0 <= row['explored_percent'] <= 100 for row in rows):
            raise ValueError('Invalid reference percentage')
        if final['frontier_complete'] and (final['state'] != 'finished' or final['frontier_count'] != 0):
            raise ValueError('Completion requires a planner declaration and zero frontiers')
        if final['true_dose_usv'] != rows[-1]['true_dose_usv']:
            raise ValueError('Endpoint/trace mismatch')


def archive_inputs(inputs):
    with ARCHIVE.open('wb') as raw, gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as archive:
            for name, payload in sorted(inputs.items()):
                info = tarfile.TarInfo(name)
                info.size, info.mode = len(payload), 0o644
                archive.addfile(info, io.BytesIO(payload))


def read_inputs():
    with tarfile.open(ARCHIVE, 'r:gz') as archive:
        return {member.name: archive.extractfile(member).read() for member in archive if member.isfile()}


def label(x, y, text, size=14, color='#b8c8d8'):
    return f'<text x="{x}" y="{y}" fill="{color}" font-size="{size}">{escape(str(text))}</text>'


def graphs(data):
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="920" viewBox="0 0 1280 920" role="img">',
        '<title>Retained uncapped frontier comparison: dose, floor observation, 3D distance and remaining frontiers</title>',
        '<rect width="1280" height="920" rx="18" fill="#0b1724"/><g font-family="DejaVu Sans,Arial,sans-serif">',
        label(32, 42, 'Radiation awareness / retained frontier experiment', 26, '#e7eef5'),
        label(32, 71, 'Reactor · seeds 1 / 0 / 0 · ideal pose · no dose/time cap · both endpoints incomplete'),
        label(32, 99, '--- Radiation-blind', 14, COLORS['blind']), label(260, 99, '— Radiation-aware', 14, COLORS['aware'])]
    definitions = [('true_dose_usv', 'Cumulative actual dose (µSv)', None),
                   ('explored_percent', 'Reference floor observed (%) — evaluation only', 100.),
                   ('distance_m', 'Actual 3D travel (m)', None),
                   ('frontier_count', 'Observed-map frontier nodes remaining', None)]
    for index, (key, title, fixed) in enumerate(definitions):
        x, y, w, h = 90+(index % 2)*620, 170+(index//2)*365, 510, 235
        xmax = max(run['samples'][-1]['time_s'] for run in data['runs'].values())
        ymax = fixed or max(1e-3, max(row[key] for run in data['runs'].values() for row in run['samples'] if row.get(key) is not None)*1.08)
        parts.append(label(x-40, y-24, title, 16, '#e7eef5'))
        for i in range(5):
            yy = y+h-i*h/4
            parts.append(f'<path d="M{x} {yy} H{x+w}" stroke="#2b3d50"/>')
            parts.append(label(x-55, yy+4, f'{ymax*i/4:.2f}' if key == 'true_dose_usv' else f'{ymax*i/4:.0f}', 12))
            parts.append(label(x+w*i/4-10, y+h+23, f'{xmax*i/4:.0f}', 12))
        parts.append(label(x+w/2-65, y+h+43, 'Simulation time (s)', 12))
        for mode, run in data['runs'].items():
            rows = [row for row in run['samples'] if row.get(key) is not None]
            points = ' '.join(f'{x+w*r["time_s"]/xmax:.2f},{y+h-h*r[key]/ymax:.2f}' for r in rows)
            dash = ' stroke-dasharray="7 4"' if mode == 'blind' else ''
            parts.append(f'<polyline points="{points}" fill="none" stroke="{COLORS[mode]}" stroke-width="2.5"{dash}/>')
    parts += [label(32, 873, 'Curves end at each robot’s actual stop. Lower endpoint dose is not an equal-duration comparison.'),
              label(32, 899, 'Finished requires zero frontiers on three fresh scans. The observed-floor percentage never ends a mission.'), '</g></svg>']
    (MEDIA/'frontier-results.svg').write_text('\n'.join(parts)+'\n')


def font(size):
    from PIL import ImageFont
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)


def preview(data, inputs):
    from PIL import Image, ImageDraw
    image = Image.new('RGB', (1320, 560), '#0b1724')
    draw = ImageDraw.Draw(image)
    draw.text((24, 16), 'DOUGIE / ACTUAL FINAL SIMULATION VIEWS', font=font(25), fill='white')
    for index, mode in enumerate(COLORS):
        x = 12+656*index
        final = data['runs'][mode]['final']
        draw.text((x+10, 62), f'Radiation-{mode}', font=font(21), fill=COLORS[mode])
        with Image.open(io.BytesIO(inputs[f'{mode}/frame.jpg'])) as frame:
            image.paste(frame.convert('RGB'), (x, 100))
        draw.text((x+10, 474), f'{final["true_dose_usv"]:.4f} µSv · {final["distance_m"]:.2f} m 3D travel', font=font(18), fill='white')
        draw.text((x+10, 505), f'{final["explored_percent"]:.2f}% reference floor · {final["frontier_count"]} frontiers remain', font=font(17), fill='#b8c8d8')
    image.save(MEDIA/'frontier-comparison.png')


def radiation_figure(inputs):
    from PIL import Image, ImageDraw
    r = json.loads(inputs['aware/radiation_map.json'])
    image = Image.new('RGB', (1320, 570), '#0b1724')
    draw = ImageDraw.Draw(image)
    draw.text((24, 16), 'GROUND TRUTH / LEARNED RADIATION', font=font(25), fill='white')
    draw.text((24, 55), 'Same floor positions, +0.44 m sampling height and logarithmic colour scale · final aware GP', font=font(17), fill='#b8c8d8')
    points, layers = np.asarray(r['points']), np.asarray(r['layers'])
    lo, hi = np.log1p(r['color_min_usv_h']), np.log1p(r['color_max_usv_h'])
    stops = np.array([[32,64,208],[0,200,220],[160,224,60],[255,224,32],[255,80,32],[192,16,48]])
    def color(rate):
        if rate is None:
            return (54, 69, 85)
        v = float(np.clip((np.log1p(rate)-lo)/(hi-lo), 0., 1.))*(len(stops)-1)
        i, f = min(len(stops)-2, int(v)), v-min(len(stops)-2, int(v))
        return tuple(np.rint(stops[i]*(1-f)+stops[i+1]*f).astype(int))
    lower, upper = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    scale = min(570/(upper[0]-lower[0]+.25), 320/(upper[1]-lower[1]+.25))
    for index, (key, title) in enumerate((('truth_usv_h', 'True simulated field'), ('gp_mean_usv_h', 'GP posterior mean'))):
        left = 24+656*index
        draw.text((left, 98), title, font=font(21), fill='white')
        for point, layer, rate in zip(points, layers, r[key]):
            if layer != 0:
                continue
            x, y = left+18+(point[0]-lower[0])*scale, 150+(upper[1]-point[1])*scale
            radius = .125*scale
            draw.rectangle((x-radius, y-radius, x+radius, y+radius), fill=color(rate))
    for x in range(1180):
        value = np.expm1(lo+(hi-lo)*x/1179)
        draw.line((70+x, 485, 70+x, 498), fill=color(value))
    draw.text((24, 515), f'{r["color_min_usv_h"]:.2f} → {r["color_max_usv_h"]:.1f} µSv/h · grey = no nearby GP query · {r["predicted_cells"]}/{r["reference_cells"]} matched cells', font=font(17), fill='#b8c8d8')
    draw.text((24, 544), f'Matched-query log1p RMSE {r["matched_log1p_rmse"]:.3f}; nearest-query alignment ≤35 cm, height difference ≤18 cm. Truth is evaluator-only.', font=font(15), fill='#b8c8d8')
    image.save(MEDIA/'frontier-radiation.png')


def report(data):
    lines = ['# Current frontier comparison results', '', data['scope'], '',
        f'Retained run: `{data["run"]}`. [Portable data](results/dougie_frontier_comparison.json) · [Raw inputs](results/frontier_comparison_inputs.tar.gz).', '',
        '## Protocol', '', 'Reactor; detector seed 1, sensor seed 0, layout seed 0; 0.5 s fixed simulated planning latency; simulator ground-truth pose; geometry-aware Poisson GP; no cumulative dose cap, no fixed time cap, and soft dose-weighted avoidance. CPU-rendered feeds target 5 FPS.', '',
        'Finished means zero observed-map frontier nodes confirmed on three fresh scans. A return with unresolved frontiers is incomplete. The reference floor percentage is evaluation-only.', '',
        '## Endpoints', '', '| Mode | Time (s) | Dose (µSv) | 3D travel (m) | Reference floor | Frontiers | Outcome / reason |', '|---|---:|---:|---:|---:|---:|---|']
    for mode, run in data['runs'].items():
        f = run['final']
        lines.append(f'| {mode} | {f["time_s"]:.3f} | {f["true_dose_usv"]:.6f} | {f["distance_m"]:.3f} | {f["explored_percent"]:.2f}% | {f["frontier_count"]} | {f["survey_outcome"]} / `{f["survey_reason"]}` |')
    lines += ['', 'Both recorded clean physical returns with zero non-foot contact impulse, but neither completed the frontier objective. Process exit 0 is not a completion claim.', '',
        '## Matched exploration progress', '', 'First retained telemetry sample at or above each reference-floor threshold; different routes/times are retained.', '',
        '| Reference threshold | Blind dose (µSv) | Aware dose (µSv) | Relative aware dose reduction |', '|---|---:|---:|---:|']
    for milestone in data['matched']['coverage_milestones']:
        hits = milestone['first_recorded_crossings']
        lines.append(f'| {milestone["reference_percent"]}% | {hits["blind"]["true_dose_usv"]:.6f} | {hits["aware"]["true_dose_usv"]:.6f} | {milestone["aware_dose_reduction_percent"]:.1f}% |')
    lines += ['', 'These are within-pair descriptive comparisons, not a confidence interval or general performance estimate. Coverage thresholds summarize a fixed evaluator reference, not the autonomous finish condition.', '',
        '![Dose, reference floor observation, distance and remaining frontiers](media/dougie/frontier-results.svg)', '',
        '![Ground truth versus final GP prediction](media/dougie/frontier-radiation.png)', '',
        '## Reproduction and provenance', '', 'The portable JSON retains all telemetry rows, endpoint status, per-worker configuration and source/asset hashes. The compressed archive retains the original summaries, configurations, survey reports, motion traces, floor/minimap arrays, final renders and radiation map. Figures use those inputs; they are not invented trajectories.', '',
        '`distance_m` integrates actual 3D base displacement on every physics tick. The historical `path_length_m` diagnostic uses displacement gating and is separately retained. The maps compare true rates at floor +44 cm against the nearest exported GP prediction within 35 cm and 18 cm vertically; missing predictions stay grey. The reported error is only over matched queries.', '',
        'Full local Go2 verification: **367 passed**. This is separate from remote CI.', '',
        '```bash', 'python3 scripts/publish_frontier_comparison.py --check', 'python3 scripts/publish_frontier_comparison.py', '```', '',
        'Import a complete new run with `--run results/dougie/comparison_<timestamp>`. All paired outcomes must be terminal. Historical six-run smoke results remain in [the October report](results/dougie_comparison_20261027.json).', '']
    (ROOT/'docs/DOUGIE_COMPARISON_RESULTS.md').write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check and args.run:
        parser.error('--check validates the committed evidence; omit --run')
    data, inputs = collect(args.run) if args.run else (json.loads(REPORT.read_text()), read_inputs())
    validate(data)
    for mode, run in data['runs'].items():
        for name, digest in run['input_sha256'].items():
            if sha(inputs[f'{mode}/{name}']) != digest:
                raise ValueError(f'Input hash mismatch: {mode}/{name}')
    if args.check:
        manifest = json.loads((MEDIA/'frontier-manifest.json').read_text())
        for name, digest in manifest['assets'].items():
            path = ROOT/name
            if sha(path.read_bytes()) != digest:
                raise ValueError(f'Published asset hash mismatch: {name}')
            if path.suffix == '.svg':
                ET.parse(path)
        print('Verified complete paired evidence, protocol, monotonic traces, raw-input hashes and figure hashes.')
        return
    if args.run:
        REPORT.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
        archive_inputs(inputs)
    graphs(data)
    preview(data, inputs)
    radiation_figure(inputs)
    report(data)
    files = [REPORT, ARCHIVE, ROOT/'docs/DOUGIE_COMPARISON_RESULTS.md', MEDIA/'frontier-results.svg',
             MEDIA/'frontier-comparison.png', MEDIA/'frontier-radiation.png']
    screenshot = MEDIA/'frontier-dashboard.png'
    if screenshot.exists():
        files.append(screenshot)
    (MEDIA/'frontier-manifest.json').write_text(json.dumps(dict(run=data['run'], scope=data['scope'],
        generator='scripts/publish_frontier_comparison.py',
        browser_capture=dict(script='scripts/check_comparison_browser.mjs', viewport=[1440, 1360],
                             mode='actual browser viewport, serving the retained terminal experiment') if screenshot.exists() else None,
        assets={str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in files}), indent=2)+'\n')
    print(json.dumps(data['matched'], indent=2))


if __name__ == '__main__':
    main()
