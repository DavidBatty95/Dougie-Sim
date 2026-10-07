#!/usr/bin/env python3
"""Render real Go2 physics and a portable detector-log replay for the README.

Run in the Dougie container with MUJOCO_GL=egl. See docs/DOUGIE_MEDIA.md.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version

from dougie_sim.simulation import Simulation

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / 'docs/media/dougie'
BG, INK, MUTED, CYAN, GOLD = '#0b1724', '#ecf5ff', '#a7bdcf', '#48daca', '#ffbc66'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)


def text(draw, xy, value, size=16, color=INK):
    draw.text(xy, value, font=font(size), fill=color)


def save_gif(frames, name, duration):
    path = MEDIA / name
    palette = frames[len(frames)//2].quantize(colors=64)
    frames = [frame.quantize(palette=palette, dither=0) for frame in frames]
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=duration,
                   loop=0, optimize=True, disposal=1)
    return {'sha256': digest(path), 'frames': len(frames), 'size': list(frames[0].size),
            'frame_duration_ms': duration, 'bytes': path.stat().st_size}


def walking():
    sim = Simulation('room')
    camera = mujoco.MjvCamera()
    camera.distance, camera.elevation, camera.azimuth = 1.9, -22., 135.
    frames = []
    with mujoco.Renderer(sim.model, height=320, width=640) as renderer:
        for i in range(120):
            for _ in range(10):
                sim.step((.3, 0., 0.))
            camera.lookat[:] = sim.data.qpos[:3]
            renderer.update_scene(sim.data, camera=camera)
            image = Image.new('RGB', (640, 420), BG)
            image.paste(Image.fromarray(renderer.render()), (0, 60))
            draw = ImageDraw.Draw(image)
            text(draw, (20, 10), 'DOUGIE / TORQUE-DRIVEN WALKING', 20, CYAN)
            text(draw, (20, 37), 'MuJoCo recording / room / commanded vx = 0.30 m/s', 13, MUTED)
            text(draw, (20, 389), f't = {sim.data.time:4.2f} s    |    200 Hz physics / 50 Hz policy', 15)
            frames.append(image)
    return save_gif(frames, 'walking.gif', 50), sim.metrics()


def replay(data):
    samples = data['measurements']
    traces = data['trace']
    xy = np.array([s['position'][:2] for s in samples])
    lo = xy.min(axis=0) - .4
    hi = xy.max(axis=0) + .4
    scale = min(330 / (hi[0] - lo[0]), 250 / (hi[1] - lo[1]))
    center = (lo + hi) / 2

    def point(pos):
        p = (np.asarray(pos[:2]) - center) * scale
        return (int(208 + p[0]), int(227 - p[1]))

    frames = []
    for n in np.linspace(1, len(samples), 90, dtype=int):
        sample = samples[n-1]
        trace = next((t for t in reversed(traces) if t['time_s'] <= sample['stamp_s']), None)
        image = Image.new('RGB', (640, 420), BG)
        draw = ImageDraw.Draw(image)
        text(draw, (20, 10), 'DOUGIE / RADIATION MISSION', 20, CYAN)
        text(draw, (20, 38), 'Recorded detector path / XY projection / seed 3 / accelerated replay', 13, MUTED)
        draw.rounded_rectangle((20, 78, 397, 371), 12, fill='#122638', outline='#24475b')
        for x in range(int(np.ceil(lo[0])), int(hi[0])+1):
            px, _ = point((x, 0))
            draw.line((px, 90, px, 355), fill='#244052')
            text(draw, (px+3, 347), str(x), 11, MUTED)
        for y in range(int(np.ceil(lo[1])), int(hi[1])+1):
            _, py = point((0, y))
            draw.line((32, py, 385, py), fill='#244052')
            text(draw, (32, py+2), str(y), 11, MUTED)
        points = [point(s['position']) for s in samples[:n]]
        if len(points) > 1:
            draw.line(points, fill=CYAN, width=3)
        home = point((0, 0))
        draw.rectangle((home[0]-5, home[1]-5, home[0]+5, home[1]+5), outline=INK, width=2)
        p = points[-1]
        draw.ellipse((p[0]-6, p[1]-6, p[0]+6, p[1]+6), fill=GOLD)
        text(draw, (415, 88), f"t = {sample['stamp_s']:.1f} s", 22)
        text(draw, (415, 124), trace['state'] if trace else 'initializing', 17, CYAN)
        text(draw, (415, 173), 'Measured dose / budget', 13, MUTED)
        dose, budget = sample['dose_usv'], data['summary']['budget_usv']
        text(draw, (415, 195), f'{dose:.4f} / {budget:g} uSv', 17)
        draw.rectangle((415, 227, 616, 238), fill='#244052')
        draw.rectangle((415, 227, 415+int(201*min(dose/budget, 1)), 238), fill=GOLD)
        text(draw, (415, 264), 'Detector rate', 13, MUTED)
        text(draw, (415, 286), f"{sample['rate_usv_h']:.2f} uSv/h", 21)
        text(draw, (415, 329), f'{n} counting samples', 14, MUTED)
        text(draw, (20, 389), 'Metre grid / square = home / path from measurements, not a GP map', 13, MUTED)
        frames.append(image)
    return save_gif(frames, 'radiation-replay.gif', 100)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, help='export a raw seed-3 run to the portable replay JSON')
    args = parser.parse_args()
    MEDIA.mkdir(parents=True, exist_ok=True)
    portable = MEDIA / 'radiation-replay.json'
    if args.run:
        names = ('measurements.jsonl', 'trace.jsonl', 'radiation_summary.json', 'run_config.json')
        data = {'origin': str(args.run), 'input_sha256': {n: digest(args.run/n) for n in names},
                'measurements': [json.loads(s) for s in (args.run/names[0]).read_text().splitlines()],
                'trace': [json.loads(s) for s in (args.run/names[1]).read_text().splitlines()],
                'summary': json.loads((args.run/names[2]).read_text()),
                'config': json.loads((args.run/names[3]).read_text())}
        portable.write_text(json.dumps(data, separators=(',', ':')) + '\n')
    data = json.loads(portable.read_text())
    walk, metrics = walking()
    manifest = {'renderer_sha256': digest(Path(__file__)),
                'source_sha256': {str(p.relative_to(ROOT)): digest(p) for p in
                                  [ROOT/'dougie_sim/simulation.py', ROOT/'config/dougie_assets.json']},
                'versions': {'mujoco': mujoco.__version__, 'numpy': np.__version__, 'pillow': pillow_version},
                'walking.gif': walk, 'walking_final_metrics': metrics,
                'radiation-replay.gif': replay(data), 'replay_input_sha256': digest(portable)}
    (MEDIA/'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
