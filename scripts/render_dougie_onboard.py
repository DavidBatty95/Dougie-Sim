#!/usr/bin/env python3
"""Publish actual onboard-run recording and a portable pose/count-window replay.

The native GIF is copied unchanged after verifying its recording manifest. The
second GIF is explicitly an evaluator telemetry replay, not camera footage.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT/'docs/media/dougie'
BG, INK, MUTED, TEAL, AMBER = '#0b1724', '#ecf5ff', '#a7bdcf', '#48daca', '#ffbc66'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def draw_text(draw, xy, message, size=15, color=INK):
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
    draw.text(xy, message, font=font, fill=color)


def replay(data):
    poses, samples = data['poses'], data['measurements']
    xy = np.array([p['position'][:2] for p in poses]+[p['evaluator_true_position'][:2] for p in poses])
    lo, hi = xy.min(axis=0)-.35, xy.max(axis=0)+.35
    centre = (lo+hi)/2
    scale = min(330/(hi[0]-lo[0]), 280/(hi[1]-lo[1]))
    def point(position):
        p = (np.asarray(position[:2])-centre)*scale
        return int(207+p[0]), int(246-p[1])
    frames, stamps = [], []
    for count in np.linspace(1, len(poses), min(100, len(poses)), dtype=int):
        pose = poses[count-1]
        stamp = pose['stamp_s']
        sample = next((s for s in reversed(samples) if s['stamp_s'] <= stamp), None)
        image = Image.new('RGB', (640, 490), BG)
        draw = ImageDraw.Draw(image)
        title = 'DOUGIE / POSE-GRAPH FUSION' if data['config'].get('slam_backend') == 'pose-graph' else 'DOUGIE / ONBOARD FUSION'
        draw_text(draw, (18, 12), title, 21, TEAL)
        draw_text(draw, (18, 43), 'Recorded telemetry / estimated pose vs independent evaluator', 13, MUTED)
        draw.rounded_rectangle((18, 78, 398, 425), radius=12, fill='#122638', outline='#31566a')
        for x in np.arange(np.ceil(lo[0]*2)/2, hi[0], .5):
            px, _ = point((x, 0))
            draw.line((px, 92, px, 408), fill='#244052')
        for y in np.arange(np.ceil(lo[1]*2)/2, hi[1], .5):
            _, py = point((0, y))
            draw.line((30, py, 386, py), fill='#244052')
        estimated = [point(p['position']) for p in poses[:count]]
        actual = [point(p['evaluator_true_position']) for p in poses[:count]]
        if count > 1:
            draw.line(actual, fill=MUTED, width=3)
            draw.line(estimated, fill=TEAL, width=2)
        for p, color in ((actual[-1], INK), (estimated[-1], TEAL)):
            draw.ellipse((p[0]-4, p[1]-4, p[0]+4, p[1]+4), fill=color)
        if sample:
            for position in sample['window_positions']:
                px, py = point(position)
                draw.ellipse((px-2, py-2, px+2, py+2), fill=AMBER)
        draw_text(draw, (415, 87), f't = {stamp:.1f} s', 21)
        draw_text(draw, (415, 128), 'IMU + leg + LiDAR', 14, TEAL)
        draw_text(draw, (415, 159), f"Pose error {pose['evaluator_error_m']*100:.1f} cm", 15)
        draw_text(draw, (415, 187), f"z: {pose['position'][2]:.2f} / {pose['evaluator_true_position'][2]:.2f} m", 14, MUTED)
        draw_text(draw, (415, 210), f"Scan updates {pose['scan_updates']}", 14, MUTED)
        draw_text(draw, (415, 237), 'ONE moving count window', 13, AMBER)
        if sample:
            draw_text(draw, (415, 266), f"{sample['counts']} counts / {sample['integration_s']:.2f} s", 14)
            draw_text(draw, (415, 296), f"{len(sample['window_positions'])} motion quadrature bins", 13, MUTED)
            draw_text(draw, (415, 349), f"Dose {sample['dose_usv']:.4f} uSv", 16)
        draw_text(draw, (18, 440), 'Teal: estimate   White: evaluator truth   Amber: detector window', 13, MUTED)
        draw_text(draw, (18, 465), '0.5 m grid / accelerated replay / evaluator truth is never a planner input', 12, MUTED)
        frames.append(image)
        stamps.append(stamp)
    palette = frames[len(frames)//2].quantize(colors=96)
    encoded = [frame.quantize(palette=palette, dither=0) for frame in frames]
    path = MEDIA/'onboard-fusion.gif'
    encoded[0].save(path, save_all=True, append_images=encoded[1:], duration=100, loop=0, disposal=2)
    frames[len(frames)*2//3].save(MEDIA/'onboard-fusion.png')
    return dict(kind='estimated-pose and detector-window telemetry replay, not video',
        frame_times_s=stamps, frame_duration_ms=100, size=[640, 490],
        sha256=digest(path), bytes=path.stat().st_size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, help='new recorded onboard run; omit to regenerate the portable telemetry replay')
    args = parser.parse_args()
    portable = MEDIA/'onboard-replay.json'
    native = MEDIA/'onboard-mission.gif'
    if args.run:
        config = json.loads((args.run/'run_config.json').read_text())
        summary = json.loads((args.run/'radiation_summary.json').read_text())
        capture = json.loads((args.run/'demo_manifest.json').read_text())
        if config['localization'] != 'fused' or config['gp_mode'] != 'geometry' or config['detector_likelihood'] != 'window':
            raise ValueError('latest gallery requires fused localization, observed geometry and moving-window inference')
        if digest(args.run/'demo.gif') != capture['sha256']:
            raise ValueError('native recording checksum mismatch')
        names = ('run_config.json', 'radiation_summary.json', 'demo_manifest.json', 'measurements.jsonl', 'localization_evaluation.jsonl')
        poses = records(args.run/'localization_evaluation.jsonl')
        indices = np.unique(np.linspace(0, len(poses)-1, min(len(poses), 300), dtype=int))
        data = dict(origin=str(args.run), input_sha256={name: digest(args.run/name) for name in names},
            config=config, capture=capture, summary={key: summary.get(key) for key in
                ('clean_return', 'true_dose_usv', 'measured_dose_usv', 'path_length_m', 'localization_evaluation',
                  'simulation', 'gp', 'nonfoot_contacts', 'information_contract', 'source_sha256',
                  'round_trip_completed', 'round_trip_failures', 'physical_witness')},
            poses=[poses[i] for i in indices], measurements=records(args.run/'measurements.jsonl'))
        portable.write_text(json.dumps(data, separators=(',', ':'), allow_nan=False)+'\n')
        shutil.copyfile(args.run/'demo.gif', native)
        with Image.open(native) as image:
            image.seek(image.n_frames*2//3)
            image.convert('RGB').save(MEDIA/'onboard-mission.png')
    data = json.loads(portable.read_text())
    manifest = dict(renderer_sha256=digest(Path(__file__)), portable_sha256=digest(portable),
        pillow_version=pillow_version, runtime_source_sha256=data['config']['source_sha256'],
        native_recording=dict(**data['capture'], bytes=native.stat().st_size),
        fusion_replay=replay(data))
    if digest(native) != manifest['native_recording']['sha256']:
        raise ValueError('committed native GIF does not match portable provenance')
    (MEDIA/'onboard-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({key: value for key, value in manifest.items() if key != 'runtime_source_sha256'}, indent=2))


if __name__ == '__main__':
    main()
