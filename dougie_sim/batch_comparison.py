"""Resumable paired-seed experiments, retaining failures and unpadded statistics."""
import argparse
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from datetime import datetime, timezone
import csv
import hashlib
from html import escape
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import threading
import time
import zipfile
from types import SimpleNamespace

import numpy as np

from .comparison import MODES, ROOT, mission_command, read_rows, stop_processes


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def source_hashes():
    paths = [*sorted((ROOT/'dougie_sim').glob('*.py')), ROOT/'config/dougie_assets.json',
             ROOT/'config/dougie_radiation_sources.json', ROOT/'docker/requirements-dougie.txt']
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def seed_settings(seed, protocol):
    offset = protocol['seeds'].index(seed)
    return dict(seed=seed if protocol['seed_axis'] in ('detector', 'both') else protocol['detector_seed'],
                sensor_seed=seed if protocol['seed_axis'] in ('sensor', 'both') else protocol['sensor_seed'],
                layout_seed=protocol['layout_seed']+offset if protocol['vary_layout'] else protocol['layout_seed'])


def statistics(values, bootstrap=1000):
    array = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    result = dict(n=len(array), mean=None, sd=None, median=None, mean_ci95=None)
    if len(array):
        result.update(mean=float(array.mean()), median=float(np.median(array)))
    if len(array) > 1:
        rng = np.random.default_rng(1729)
        means = array[rng.integers(0, len(array), (bootstrap, len(array)))].mean(axis=1)
        result.update(sd=float(array.std(ddof=1)), mean_ci95=np.quantile(means, [.025, .975]).tolist())
    return result


def row_at_time(rows, stamp):
    if not rows or stamp < rows[0]['time_s'] or stamp > rows[-1]['time_s']:
        return None
    times = [row['time_s'] for row in rows]
    right = int(np.searchsorted(times, stamp))
    if right == 0 or times[right] == stamp:
        return rows[right]
    a, b = rows[right-1], rows[right]
    weight = (stamp-a['time_s'])/(b['time_s']-a['time_s'])
    return {key: a[key]+weight*(b[key]-a[key]) for key in ('true_dose_usv', 'explored_percent', 'distance_m')}


def load_pairs(output, protocol):
    pairs = []
    for seed in protocol['seeds']:
        directory = output/f'seed_{seed:05d}'
        path = directory/'pair_status.json'
        status = json.loads(path.read_text()) if path.exists() else dict(state='pending', censored=False)
        pair = dict(seed=seed, settings=seed_settings(seed, protocol), status=status, runs={})
        for mode in MODES:
            run_dir = directory/mode
            summary_path = run_dir/'radiation_summary.json'
            retained = status['state'] in ('terminal', 'timeout', 'interrupted', 'failed')
            artifact_errors = []
            try:
                summary = json.loads(summary_path.read_text()) if retained and summary_path.exists() else None
            except json.JSONDecodeError:
                summary = None  # Retain the corrupt file; report the missing usable summary.
                artifact_errors.append('invalid radiation_summary.json')
            rows = read_rows(run_dir/'comparison.jsonl') if status['state'] in ('terminal', 'timeout', 'interrupted', 'failed') else []
            slices = {}
            if retained and (run_dir/'minimap_final.npz').exists():
                try:
                    with np.load(run_dir/'minimap_final.npz') as data:
                        heights, xyz, seen = data['slice_heights_m'], data['xyz'], data['seen']
                        layers = np.searchsorted((heights[1:]+heights[:-1])/2, xyz[:, 2], side='right')
                        slices = {f'{height:.2f}': float(100.*seen[layers == i].mean())
                                  for i, height in enumerate(heights) if np.any(layers == i)}
                except (ValueError, OSError, KeyError, zipfile.BadZipFile):
                    artifact_errors.append('invalid minimap_final.npz')
            pair['runs'][mode] = dict(summary=summary, rows=rows, slices=slices,
                artifact_errors=artifact_errors,
                exit_code=status.get('exit_codes', {}).get(mode))
        pairs.append(pair)
    return pairs


def aggregate(pairs, protocol):
    terminal = [p for p in pairs if p['status']['state'] in ('terminal', 'timeout', 'failed')]
    report = dict(protocol=protocol, requested_pairs=len(pairs), terminal_pairs=len(terminal),
        running_seeds=[p['seed'] for p in pairs if p['status']['state'] == 'running'],
        pending_seeds=[p['seed'] for p in pairs if p['status']['state'] in ('pending', 'interrupted')],
        censored_pairs=sum(p['status'].get('censored', False) for p in pairs), modes={}, milestones=[], curves=[])
    report['uncertainty'] = dict(method='pointwise percentile bootstrap of the mean', confidence=.95,
        rng_seed=1729, endpoint_and_milestone_resamples=1000, curve_resamples=500,
        sample_sd_ddof=1, analysis_unit='paired seed; paired differences are formed before resampling')
    report['analysis_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['seed_outcomes'] = []
    for pair in pairs:
        for mode, run in pair['runs'].items():
            last = run['rows'][-1] if run['rows'] else {}
            report['seed_outcomes'].append(dict(pair_seed=pair['seed'], settings=pair['settings'], mode=mode,
                pair_state=pair['status']['state'], censored=pair['status'].get('censored', False),
                exit_code=run['exit_code'], mission_state=last.get('state'), survey_outcome=last.get('survey_outcome'),
                survey_reason=last.get('survey_reason'), clean_return=(run['summary'] or {}).get('clean_return'),
                error=(run['summary'] or {}).get('error'), artifact_errors=run.get('artifact_errors', []),
                per_level_observed_percent=run['slices']))
    keys = ('true_dose_usv', 'distance_m', 'explored_percent', 'time_s', 'frontier_count')
    for mode in MODES:
        runs = [p['runs'][mode] for p in terminal]
        endpoints = [r['rows'][-1] for r in runs if r['rows']]
        complete = sum(bool(r['summary'] and r['summary'].get('frontier_complete') and
                           r['summary'].get('frontier_count') == 0 and
                           r['summary'].get('frontier_confirmation_count', 0) >= 3 and
                           r['summary'].get('end_state') == 'finished' and not r['summary'].get('error') and r['exit_code'] == 0 and
                           not p['status'].get('censored', False)) for p in terminal for r in [p['runs'][mode]])
        levels = sorted({level for r in runs for level in r['slices']}, key=float)
        report['modes'][mode] = dict(confirmed_frontier_completions=complete,
            completion_denominator=len(terminal), endpoint_observations=len(endpoints),
            process_errors=sum(r['exit_code'] not in (None, 0) or bool((r['summary'] or {}).get('error')) for r in runs),
            missing_summaries=sum(r['summary'] is None for r in runs),
            artifact_errors=sum(bool(r.get('artifact_errors')) for r in runs),
            upper_landing_witnessed=sum((r['summary'] or {}).get('physical_witness', {}).get('upper_landing_witness_s') is not None for r in runs),
            endpoint={key: statistics([r.get(key) for r in endpoints]) for key in keys},
            per_level_observed_percent={level: statistics([r['slices'].get(level) for r in runs]) for level in levels})
    paired = [p for p in terminal if all(p['runs'][mode]['rows'] for mode in MODES)]
    for threshold in protocol['milestones']:
        hits = []
        for pair in paired:
            crossings = {mode: next((r for r in pair['runs'][mode]['rows'] if r['explored_percent'] >= threshold), None)
                         for mode in MODES}
            if all(crossings.values()):
                hits.append(dict(seed=pair['seed'], **crossings))
        blind = [hit['blind']['true_dose_usv'] for hit in hits]
        aware = [hit['aware']['true_dose_usv'] for hit in hits]
        report['milestones'].append(dict(reference_percent=threshold, eligible_pairs=len(hits),
            excluded_or_not_reached_pairs=len(pairs)-len(hits), crossings=hits,
            blind_dose=statistics(blind), aware_dose=statistics(aware),
            paired_aware_minus_blind_dose=statistics([a-b for a, b in zip(aware, blind)]),
            paired_relative_reduction_percent=statistics([100.*(1-a/b) for a, b in zip(aware, blind) if b > 0]),
            ratio_of_mean_dose_reduction_percent=100.*(1-float(np.mean(aware))/float(np.mean(blind))) if hits and np.mean(blind) > 0 else None))
    maximum = max((min(p['runs'][m]['rows'][-1]['time_s'] for m in MODES) for p in paired), default=0.)
    for stamp in np.arange(0., maximum+1e-9, protocol['curve_step_s']):
        samples = []
        for pair in paired:
            values = {mode: row_at_time(pair['runs'][mode]['rows'], float(stamp)) for mode in MODES}
            if all(values.values()):
                samples.append(values)
        if samples:
            report['curves'].append(dict(time_s=float(stamp), paired_n=len(samples),
                **{mode: {key: statistics([value[mode][key] for value in samples], bootstrap=500)
                          for key in ('true_dose_usv', 'explored_percent', 'distance_m')} for mode in MODES}))
    report['scope'] = ('Detector/sensor Monte Carlo repeats on declared layouts; not independent environment validation unless layout variation is selected. '
        'Endpoint averages include incomplete outcomes and are not matched work. Milestones require both trajectories to reach the threshold. '
        'Curves use only pairs with observations at that time; no endpoint carry-forward, extrapolation or padded zero dose. '
        'The changing paired_n is reported explicitly. CIs are fixed-seed pointwise percentile bootstrap intervals for the mean, not simultaneous bands or mission safety guarantees.')
    return report


def mean_curves_svg(report):
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="560" viewBox="0 0 1100 560" role="img">',
        '<title>Matched-time paired means with percentile-bootstrap 95% intervals; no endpoint padding</title>',
        '<rect width="1100" height="560" fill="#0b1724"/><g font-family="sans-serif" fill="#e7eef5">',
        '<text x="30" y="38" font-size="23">Paired seed study / matched-time means</text>']
    curves = report['curves']
    for index, (key, title) in enumerate((('true_dose_usv', 'True dose (µSv)'), ('explored_percent', 'Reference floor observed (%)'))):
        x, y, width, height = 75+index*535, 105, 440, 310
        xmax = max((r['time_s'] for r in curves), default=1.) or 1.
        ymax = 100. if key == 'explored_percent' else max([.01]+[(r[m][key]['mean_ci95'] or [0., r[m][key]['mean']])[1] for r in curves for m in MODES])*1.1
        parts.append(f'<text x="{x}" y="{y-25}" font-size="16">{title}</text>')
        for tick in range(5):
            yy = y+height-height*tick/4
            parts.append(f'<path d="M{x} {yy} H{x+width}" stroke="#293e50"/><text x="{x-55}" y="{yy+4}" font-size="12">{ymax*tick/4:.2f}</text>')
            parts.append(f'<text x="{x+width*tick/4-10}" y="{y+height+23}" font-size="12">{xmax*tick/4:.0f}</text>')
        for mode, color in (('blind', '#ffb454'), ('aware', '#48daca')):
            valid = [r for r in curves if r[mode][key]['mean_ci95'] is not None]
            if valid:
                upper = [(x+width*r['time_s']/xmax, y+height-height*r[mode][key]['mean_ci95'][1]/ymax) for r in valid]
                lower = [(x+width*r['time_s']/xmax, y+height-height*r[mode][key]['mean_ci95'][0]/ymax) for r in reversed(valid)]
                points = ' '.join(f'{a:.2f},{b:.2f}' for a, b in upper+lower)
                parts.append(f'<polygon points="{points}" fill="{color}" opacity=".15"/>')
            points = ' '.join(f'{x+width*r["time_s"]/xmax:.2f},{y+height-height*r[mode][key]["mean"]/ymax:.2f}' for r in curves)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        parts.append(f'<text x="{x+120}" y="{y+height+45}" font-size="12">Simulation time (s)</text>')
    counts = f'{curves[0]["paired_n"]} → {curves[-1]["paired_n"]}' if curves else 'no terminal pairs yet'
    parts += [f'<text x="30" y="495" font-size="14">Orange: blind · Teal: aware · Shading: bootstrap mean CI · paired n: {escape(counts)}</text>',
        '<text x="30" y="525" font-size="13">Later means condition on pairs still observed; see JSON for n at every point. Endpoints are never padded.</text>', '</g></svg>']
    return '\n'.join(parts)+'\n'


def write_reports(output, protocol):
    pairs = load_pairs(output, protocol)
    report = aggregate(pairs, protocol)
    write_json(output/'batch_summary.json', report)
    with (output/'runs.csv').open('w', newline='') as stream:
        metric_keys = ['frontier_complete', 'time_s', 'true_dose_usv', 'distance_m', 'explored_percent', 'frontier_count', 'survey_reason', 'survey_outcome']
        columns = ['pair_seed', 'mode', 'pair_state', 'censored', 'exit_code', 'mission_state', 'clean_return', 'error', *metric_keys, 'artifact_errors']
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for pair in pairs:
            for mode, run in pair['runs'].items():
                row = run['rows'][-1] if run['rows'] else {}
                writer.writerow(dict(pair_seed=pair['seed'], mode=mode, pair_state=pair['status']['state'],
                    censored=pair['status'].get('censored', False), exit_code=run['exit_code'],
                    mission_state=row.get('state'), clean_return=(run['summary'] or {}).get('clean_return'),
                    error=(run['summary'] or {}).get('error'),
                    artifact_errors='; '.join(run.get('artifact_errors', [])),
                    **{key: row.get(key) for key in metric_keys}))
    fmt = lambda value: '—' if value is None else f'{value:.4f}'
    estimate = lambda stat: f'{fmt(stat["mean"])} ± {fmt(stat["sd"])} (n={stat["n"]})'
    lines = ['# Paired multi-seed comparison', '',
        f'World: **{protocol["world"]}** · requested pairs: **{len(pairs)}** · terminal: **{report["terminal_pairs"]}** · censored: **{report["censored_pairs"]}**', '',
        f'Seed axis: `{protocol["seed_axis"]}`; vary layout: `{protocol["vary_layout"]}`. Active seeds: {report["running_seeds"]}. Pending/resumable: {report["pending_seeds"]}.', '',
        '## Observed endpoints — descriptive, not matched work', '',
        '| Mode | Confirmed frontier completion | Dose mean ± SD (µSv) | 3D distance mean ± SD (m) | Floor observed mean ± SD (%) | Process errors / missing summaries / artifact-error runs |',
        '|---|---|---|---|---|---|']
    for mode, group in report['modes'].items():
        lines.append(f'| {mode} | {group["confirmed_frontier_completions"]}/{group["completion_denominator"]} terminal pairs | {estimate(group["endpoint"]["true_dose_usv"])} | {estimate(group["endpoint"]["distance_m"])} | {estimate(group["endpoint"]["explored_percent"])} | {group["process_errors"]} / {group["missing_summaries"]} / {group["artifact_errors"]} |')
    lines += ['', '## Matched reference milestones', '',
        '| Threshold | Eligible pairs / requested | Blind dose mean ± SD | Aware dose mean ± SD | Paired aware−blind dose mean ± SD |', '|---|---|---|---|---|']
    for milestone in report['milestones']:
        lines.append(f'| ≥{milestone["reference_percent"]:g}% | {milestone["eligible_pairs"]}/{len(pairs)} | {estimate(milestone["blind_dose"])} | {estimate(milestone["aware_dose"])} | {estimate(milestone["paired_aware_minus_blind_dose"])} |')
    lines += ['', '## Per-elevation observation', '']
    for mode, group in report['modes'].items():
        lines.append(f'- **{mode}**: upper-landing physical witnesses {group["upper_landing_witnessed"]}; '+
                     '; '.join(f'Z {level} m: {estimate(stat)}%' for level, stat in group['per_level_observed_percent'].items()))
    lines += ['', report['scope'], '',
        'Frontier completion requires a planner finish declaration, not a floor percentage. Errors, missing summaries and timeouts are retained. A timeout is censored, not frontier completion. All per-seed raw files stay in seed directories; interrupted attempts are archived before retry on resume.', '',
        'See `batch_summary.json` for mean, sample SD, median, bootstrap 95% mean intervals, paired differences, eligibility counts and unpadded curves. `runs.csv` retains individual endpoint outcomes.', '']
    (output/'REPORT.md').write_text('\n'.join(lines))
    (output/'batch_mean_curves.svg').write_text(mean_curves_svg(report))
    table = lambda headers, rows: '<table><thead><tr>'+''.join(f'<th>{escape(h)}</th>' for h in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join(f'<td>{escape(str(v))}</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table>'
    endpoints = [[mode, f'{g["confirmed_frontier_completions"]}/{g["completion_denominator"]}', estimate(g['endpoint']['true_dose_usv']),
                  estimate(g['endpoint']['distance_m']), estimate(g['endpoint']['explored_percent']),
                  f'{g["process_errors"]} / {g["missing_summaries"]} / {g["artifact_errors"]}'] for mode, g in report['modes'].items()]
    milestones = [[f'≥{r["reference_percent"]:g}%', f'{r["eligible_pairs"]}/{len(pairs)}', estimate(r['blind_dose']),
                   estimate(r['aware_dose']), estimate(r['paired_aware_minus_blind_dose'])] for r in report['milestones']]
    page = ('<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><meta http-equiv="refresh" content="15">'
        '<title>Dougie / paired seed study</title><style>body{background:#0b1724;color:#e7eef5;font:15px/1.6 system-ui;margin:40px auto;max-width:1200px;padding:20px}h1{font-size:36px}p{color:#b8c8d8}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:13px;border-bottom:1px solid #34485b}img{width:100%}.scroll{overflow:auto}a{color:#48daca}</style>'
        f'<h1>DOUGIE / Paired seed study</h1><p>{escape(protocol["world"])} · {report["terminal_pairs"]}/{len(pairs)} terminal pairs · {report["censored_pairs"]} censored · seed axis {protocol["seed_axis"]}</p>'
        '<h2>Observed endpoints</h2><p>Mean ± sample SD. Endpoints can have different work and duration; they are not the effectiveness headline.</p><div class="scroll">'+
        table(['Mode', 'Confirmed completion', 'Dose / µSv', '3D travel / m', 'Floor observed / %', 'Errors / missing / artifact runs'], endpoints)+'</div>'
        '<h2>Dose at shared observation milestones</h2><p>Only pairs where both robots reached a threshold are included; the denominator is always shown. Negative aware−blind dose favours awareness.</p><div class="scroll">'+
        table(['Threshold','Eligible / requested','Blind dose / µSv','Aware dose / µSv','Paired aware−blind / µSv'], milestones)+'</div>'
        '<h2>Matched-time means and uncertainty</h2><img src="batch_mean_curves.svg" alt="Paired mean curves with bootstrap intervals and changing sample counts">'
        f'<p>{escape(report["scope"])}</p><p>Frontier completion is independent of the reference floor percentage. This report refreshes every 15 seconds.</p>'
        '<a href="batch_summary.json">Full statistics + 95% intervals</a> · <a href="runs.csv">Per-seed CSV</a> · <a href="REPORT.md">Detailed outcomes and floor levels</a>')
    temporary = output/'report.html.tmp'
    temporary.write_text(page)
    temporary.replace(output/'report.html')
    return report


def run_pair(seed, output, protocol, expected_sources, stop, wall_timeout):
    directory = output/f'seed_{seed:05d}'
    previous = directory/'pair_status.json'
    if previous.exists() and json.loads(previous.read_text())['state'] in ('terminal', 'timeout', 'failed'):
        return
    if directory.exists():
        archive = output/'interrupted_attempts'
        archive.mkdir(exist_ok=True)
        directory.rename(archive/(directory.name+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')))
    if stop.is_set():
        return
    directory.mkdir(parents=True)
    processes, logs = {}, []
    status = dict(state='running', censored=False, exit_codes={}, seed=seed)
    write_json(directory/'pair_status.json', status)
    try:
        if source_hashes() != expected_sources:
            raise ValueError('Runtime sources changed during the cohort; use a new output directory')
        args = SimpleNamespace(**{k: protocol[k] for k in ('world', 'duration', 'localization', 'radiation_policy', 'rate_limit')},
                               **seed_settings(seed, protocol), dose_budget=None, feed_fps=1, no_video=True,
                               gp_mode=protocol.get('gp_mode', 'windowed'),
                                exposure_policy=protocol.get('exposure_policy','responsive'),
                                frontier_rate_limit=protocol.get('frontier_rate_limit',protocol['rate_limit']),
                                frontier_confidence=protocol.get('frontier_confidence',.95),
                                frontier_view_policy=protocol.get('frontier_view_policy','raw'),
                               gp_expert_capacity=protocol.get('gp_expert_capacity',32), gp_max_experts=protocol.get('gp_max_experts',16))
        commands = {mode: mission_command(args, mode, directory) for mode in MODES}
        write_json(directory/'comparison_config.json', {**vars(args), 'commands': commands,
            'coverage_metric': 'physically_reached_3d_floor_v2', 'pair_seed': seed,
            'completion_rule': 'zero observed-map frontiers on three fresh scans'})
        for mode in MODES:
            log = (directory/(mode+'.log')).open('w')
            logs.append(log)
            processes[mode] = subprocess.Popen(commands[mode], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, env={**os.environ, 'OMP_NUM_THREADS':'1', 'OPENBLAS_NUM_THREADS':'1'})
        print(f'Seed {seed}: started blind + aware (no video)', flush=True)
        started = time.monotonic()
        while any(p.poll() is None for p in processes.values()):
            if stop.is_set() or (wall_timeout and time.monotonic()-started >= wall_timeout):
                status.update(state='interrupted' if stop.is_set() else 'timeout', censored=True)
                stop_processes(processes)
                break
            time.sleep(.2)
        if status['state'] == 'running':
            status['state'] = 'terminal'
        status['exit_codes'] = {mode:p.poll() for mode, p in processes.items()}
        write_json(directory/'comparison_status.json', status['exit_codes'])
    except Exception as exc:
        stop_processes(processes)
        status.update(state='failed', error=str(exc), exit_codes={m:p.poll() for m,p in processes.items()})
    finally:
        for log in logs:
            log.close()
        write_json(directory/'pair_status.json', status)
        print(f'Seed {seed}: {status["state"]}; exit codes {status["exit_codes"]}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='facility')
    parser.add_argument('--seed-count', type=int, default=30)
    parser.add_argument('--seed-start', type=int, default=1)
    parser.add_argument('--seeds', type=int, nargs='+')
    parser.add_argument('--seed-axis', choices=('detector', 'sensor', 'both'), default='detector')
    parser.add_argument('--detector-seed', type=int, default=1)
    parser.add_argument('--sensor-seed', type=int, default=0)
    parser.add_argument('--layout-seed', type=int, default=0)
    parser.add_argument('--vary-layout', action='store_true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--localization', choices=('ground-truth', 'fused'), default='ground-truth')
    parser.add_argument('--radiation-policy', choices=('minimize-dose', 'rate-limited'), default='minimize-dose')
    parser.add_argument('--exposure-policy',choices=('standard','responsive','pathwise'),default='responsive')
    parser.add_argument('--rate-limit', type=float, default=100.)
    parser.add_argument('--frontier-rate-limit',type=float,help='confidence-qualified survey hotspot threshold; default rate-limit, 0 disables')
    parser.add_argument('--frontier-confidence',type=float,default=.95)
    parser.add_argument('--frontier-view-policy',choices=('raw','marginal'),default='raw')
    parser.add_argument('--gp-mode', choices=('windowed', 'geometry', 'spatial', 'global'), default='windowed')
    parser.add_argument('--gp-expert-capacity', type=int, default=32)
    parser.add_argument('--gp-max-experts', type=int, default=16)
    parser.add_argument('--workers', type=int, default=1, help='concurrent seed pairs; each pair has two simulations and planner workers')
    parser.add_argument('--wall-timeout', type=float, default=0., help='optional wall seconds per pair; 0 disables, timed-out pairs are censored')
    parser.add_argument('--curve-step', type=float, default=10.)
    parser.add_argument('--milestones', type=float, nargs='+', default=[10., 25., 50., 75., 90.])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--report-only', action='store_true')
    args = parser.parse_args()
    if args.frontier_rate_limit is None:
        args.frontier_rate_limit = args.rate_limit
    if not math.isfinite(args.frontier_rate_limit) or args.frontier_rate_limit < 0 or not math.isfinite(args.frontier_confidence) or not .5 < args.frontier_confidence < 1.:
        parser.error('frontier threshold must be finite and nonnegative; confidence must be between .5 and 1')
    if args.exposure_policy == 'pathwise' and args.radiation_policy != 'minimize-dose':
        parser.error('pathwise policy requires minimize-dose routing')
    output = args.output.resolve()
    if args.report_only:
        saved = json.loads((output/'batch_config.json').read_text())
        report = write_reports(output, saved['protocol'])
        print(f'{report["terminal_pairs"]}/{report["requested_pairs"]} terminal pairs; report: {output/"report.html"}')
        return
    seeds = args.seeds or list(range(args.seed_start, args.seed_start+args.seed_count))
    if (not 1 <= args.seed_count <= 10000 or not seeds or len(set(seeds)) != len(seeds)
            or min(*seeds, args.seed_start, args.detector_seed, args.sensor_seed, args.layout_seed) < 0
            or not 1 <= args.workers <= 16 or not math.isfinite(args.duration) or args.duration < 0
            or not math.isfinite(args.wall_timeout) or args.wall_timeout < 0
            or not math.isfinite(args.curve_step) or args.curve_step <= 0
            or not math.isfinite(args.rate_limit) or args.rate_limit <= 0
            or not all(math.isfinite(v) and 0 < v <= 100 for v in args.milestones)):
        parser.error('invalid seeds, worker count, durations, rate or milestones')
    if args.vary_layout and args.world != 'facility':
        parser.error('--vary-layout currently requires facility')
    protocol = {k:getattr(args,k) for k in ('world', 'seed_axis', 'detector_seed', 'sensor_seed', 'layout_seed',
        'vary_layout', 'duration', 'localization', 'radiation_policy', 'rate_limit', 'milestones',
        'gp_mode', 'gp_expert_capacity', 'gp_max_experts','exposure_policy','frontier_rate_limit','frontier_confidence','frontier_view_policy')}
    protocol.update(seeds=seeds, curve_step_s=args.curve_step, wall_timeout_s=args.wall_timeout,
                    completion='planner frontier exhaustion', dose_budget=None, rendering=False)
    sources = source_hashes()
    if output.exists():
        if not args.resume:
            parser.error('output exists; use --resume with the identical cohort protocol')
        saved = json.loads((output/'batch_config.json').read_text())
        if saved['protocol'] != protocol or saved['source_sha256'] != sources:
            parser.error('resume protocol or runtime source hashes differ; choose a new output directory')
    else:
        output.mkdir(parents=True)
        write_json(output/'batch_config.json', dict(protocol=protocol, source_sha256=sources))
        with tarfile.open(output/'source_snapshot.tar.gz', 'w:gz') as archive:
            for path in sources:
                archive.add(ROOT/path, arcname=path)
    stop = threading.Event()
    from .assets import ensure_assets
    ensure_assets()  # Serialize asset verification before launching two workers.
    executor = ThreadPoolExecutor(max_workers=args.workers)
    futures = {executor.submit(run_pair, seed, output, protocol, sources, stop, args.wall_timeout) for seed in seeds}
    print(f'{len(seeds)} paired seeds · {args.workers} concurrent pair(s) · report: {output/"report.html"}', flush=True)
    try:
        while futures:
            done, futures = wait(futures, timeout=5., return_when=FIRST_COMPLETED)
            for future in done:
                future.result()
            write_reports(output, protocol)
    except KeyboardInterrupt:
        print('Stopping active pairs; interrupted attempts will be retained for resume.', flush=True)
        stop.set()
    finally:
        stop.set()
        executor.shutdown(wait=True, cancel_futures=True)
        report = write_reports(output, protocol)
    print(f'{report["terminal_pairs"]}/{len(seeds)} terminal pairs. Full results: {output/"batch_summary.json"}', flush=True)
    if report['censored_pairs'] or any(g['process_errors'] or g['missing_summaries'] or g['artifact_errors'] for g in report['modes'].values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
