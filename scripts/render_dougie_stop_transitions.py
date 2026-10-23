#!/usr/bin/env python3
"""Render all paired stop-transition trials as labelled evaluator telemetry."""
import hashlib
import json
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'docs/results/dougie_stop_transitions_20261022.json'
MEDIA = ROOT/'docs/media/dougie'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=REPORT)
    parser.add_argument('--prefix', choices=('stop-transitions', 'position-hold', 'range-hold'), default='stop-transitions')
    args = parser.parse_args()
    report_path = args.report.resolve()
    report = json.loads(report_path.read_text())
    results = report['results']
    actual = [(r['angle_deg'], r['mode'], r['sensor_seed'], r['direction']) for r in results]
    if actual != [tuple(case) for case in report['cases']] or len(actual) != 20:
        raise ValueError('the animation requires the complete declared 20-case cohort')
    modes = {result['mode'] for result in results}-{'zero_policy'}
    if len(modes) != 1 or not modes <= {'guarded_target_hold', 'position_hold', 'range_position_hold'}:
        raise ValueError('expected one supported comparator paired with zero policy')
    comparator = next(iter(modes))
    feedback = comparator != 'guarded_target_hold'
    range_feedback = comparator == 'range_position_hold'
    hold_results = [result for result in results if result['mode'] == comparator]
    ramp_drift = [result['max_horizontal_drift_m'] for result in hold_results if result['angle_deg'] > 0]
    contact_clean = all(result['nonfoot_impulse_ns'] <= .1 and result['nonfoot_peak_force_n'] <= 25.
                        for result in hold_results)
    rows = [(0., 'up', 'Flat'), (8.53, 'up', '8.53 deg uphill'), (10., 'up', '10 deg uphill'),
            (8.53, 'down', '8.53 deg downhill'), (10., 'down', '10 deg downhill')]
    series = []
    for result in results:
        samples = [p for p in result['trace'] if p['time_s'] >= result['stop_requested_s']-1e-6]
        times = np.array([p['time_s']-result['stop_requested_s'] for p in samples])
        xy = np.array([p['evaluator_position'][:2] for p in samples])
        series.append((result, times, np.maximum.accumulate(np.linalg.norm(xy-xy[0], axis=1))))
    fonts = {size: ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
             for size in (12, 14, 18, 22)}
    frames, stamps = [], np.linspace(0., report['duration_s'], 81)
    for stamp in stamps:
        image = Image.new('RGB', (800, 590), '#0b1724')
        draw = ImageDraw.Draw(image)
        def text(x, y, value, size=14, color='#ecf5ff'):
            draw.text((x, y), value, font=fonts[size], fill=color)
        title = 'DOUGIE / RANGE-OBSERVABLE HOLD' if range_feedback else 'DOUGIE / ACTIVE HOLD SCREEN' if feedback else 'DOUGIE / WALK-TO-STOP SCREEN'
        text(20, 14, title, 22, '#48daca')
        text(20, 47, 'Evaluator telemetry, not video / all 20 '+('healthy-range ' if range_feedback else '')+'trials / two seeds per pair', 14)
        text(20, 73, f'Time after stop request: {stamp:5.1f} s   |   120 s dwell, friction 0.4', 14)
        label = 'range-registered position feedback' if range_feedback else 'IMU/leg position feedback' if feedback else 'guarded target hold'
        text(20, 103, 'Amber: zero-command policy     Teal: '+label, 14)
        text(570, 130, 'FINAL contact outcome', 12)
        for row_index, (angle, direction, label) in enumerate(rows):
            y = 154+row_index*72
            text(20, y, label, 14)
            for mode_index, mode in enumerate(('zero_policy', comparator)):
                group = [(r, t, d) for r, t, d in series
                         if r['angle_deg'] == angle and r['direction'] == direction and r['mode'] == mode]
                value = max(float(d[min(len(d)-1, max(0, np.searchsorted(t, stamp, side='right')-1))])
                            for _, t, d in group)
                yy = y+mode_index*25
                color = '#ffbc66' if mode_index == 0 else '#48daca'
                draw.rectangle((185, yy, 505, yy+15), fill='#1d3547')
                draw.rectangle((185, yy, 185+min(2., value)*160, yy+15), fill=color)
                text(515, yy-2, f'{value:.2f}m', 12)
                failed = sum(r['nonfoot_impulse_ns'] > .1 or r['nonfoot_peak_force_n'] > 25.
                             or r['error'] is not None or r['hold_entry_failed'] for r, _, _ in group)
                text(585, yy-2, f'{failed}/2 FAIL' if failed else '0/2 contact failures', 12,
                     '#ff7c80' if failed else '#a7bdcf')
        text(185, 520, 'Bars: max XY drift so far across two seeds / common 0–2 m scale', 12)
        warning = (f"{'Contact-clean' if contact_clean else 'Contact failures'}; ramp drift {min(ramp_drift):.2f}–{max(ramp_drift):.2f} m: NOT qualified."
                   if feedback else 'Downhill hold reduces travel but causes body contact: NOT a qualified stop controller.')
        text(20, 545, warning, 14, '#ff7c80')
        text(20, 570, '1 Hz trace replay; exact full-rate maxima are in the linked report. Approx. 15x playback.', 12)
        frames.append(image)
    palette = frames[-1].quantize(colors=96)
    encoded = [image.quantize(palette=palette, dither=0) for image in frames]
    path = MEDIA/(args.prefix+'.gif')
    encoded[0].save(path, save_all=True, append_images=encoded[1:], duration=100, loop=0, disposal=2)
    frames[-1].save(MEDIA/(args.prefix+'.png'))
    manifest = dict(kind='downsampled evaluator telemetry, not physics video', report=str(report_path.relative_to(ROOT)),
        report_sha256=digest(report_path), renderer_sha256=digest(Path(__file__)), pillow_version=pillow_version,
        sha256=digest(path), bytes=path.stat().st_size, size=[800, 590], frame_duration_ms=100,
        frame_times_after_stop_s=stamps.tolist(), cases=len(results),
        contact_limits=dict(impulse_ns=.1, peak_force_n=25.))
    (MEDIA/(args.prefix+'.json')).write_text(json.dumps(manifest, indent=2)+'\n')
    with Image.open(path) as decoded:
        assert decoded.n_frames == len(frames) and decoded.size == (800, 590)
        for index in range(decoded.n_frames):
            decoded.seek(index)
            decoded.load()
            assert decoded.info['duration'] == 100
    print(f'Encoded and decoded {len(frames)} frames: {path}')


if __name__ == '__main__':
    main()
