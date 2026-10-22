#!/usr/bin/env python3
"""Render all paired stop-transition trials as labelled evaluator telemetry."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'docs/results/dougie_stop_transitions_20261022.json'
MEDIA = ROOT/'docs/media/dougie'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    report = json.loads(REPORT.read_text())
    results = report['results']
    actual = [(r['angle_deg'], r['mode'], r['sensor_seed'], r['direction']) for r in results]
    if actual != [tuple(case) for case in report['cases']] or len(actual) != 20:
        raise ValueError('the animation requires the complete declared 20-case cohort')
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
        text(20, 14, 'DOUGIE / WALK-TO-STOP SCREEN', 22, '#48daca')
        text(20, 47, 'Evaluator telemetry, not video / all 20 trials / two sensor seeds per pair', 14)
        text(20, 73, f'Time after zero command: {stamp:5.1f} s   |   120 s dwell, friction 0.4', 14)
        text(20, 103, 'Amber: zero-command policy     Teal: guarded target hold', 14)
        text(570, 130, 'FINAL contact outcome', 12)
        for row_index, (angle, direction, label) in enumerate(rows):
            y = 154+row_index*72
            text(20, y, label, 14)
            for mode_index, mode in enumerate(('zero_policy', 'guarded_target_hold')):
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
        text(20, 545, 'Downhill hold reduces travel but causes body contact: NOT a qualified stop controller.', 14, '#ff7c80')
        text(20, 570, '1 Hz trace replay; exact full-rate maxima are in the linked report. Approx. 15x playback.', 12)
        frames.append(image)
    palette = frames[-1].quantize(colors=96)
    encoded = [image.quantize(palette=palette, dither=0) for image in frames]
    path = MEDIA/'stop-transitions.gif'
    encoded[0].save(path, save_all=True, append_images=encoded[1:], duration=100, loop=0, disposal=2)
    frames[-1].save(MEDIA/'stop-transitions.png')
    manifest = dict(kind='downsampled evaluator telemetry, not physics video', report=str(REPORT.relative_to(ROOT)),
        report_sha256=digest(REPORT), renderer_sha256=digest(Path(__file__)), pillow_version=pillow_version,
        sha256=digest(path), bytes=path.stat().st_size, size=[800, 590], frame_duration_ms=100,
        frame_times_after_stop_s=stamps.tolist(), cases=len(results),
        contact_limits=dict(impulse_ns=.1, peak_force_n=25.))
    (MEDIA/'stop-transitions.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with Image.open(path) as decoded:
        assert decoded.n_frames == len(frames) and decoded.size == (800, 590)
        for index in range(decoded.n_frames):
            decoded.seek(index)
            decoded.load()
            assert decoded.info['duration'] == 100
    print(f'Encoded and decoded {len(frames)} frames: {path}')


if __name__ == '__main__':
    main()
