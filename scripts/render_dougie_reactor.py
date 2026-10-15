#!/usr/bin/env python3
"""Render the actual MuJoCo reactor model for the project overview."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from dougie_sim.simulation import Simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, help='replay the archived final GP prediction and detector evidence')
parser.add_argument('--output', type=Path, default=Path('docs/media/dougie-reactor.png'))
args = parser.parse_args()
sim = Simulation('reactor', model_xml=(args.run/'scene.xml').read_text() if args.run else None)
if args.run:
    with np.load(args.run/'motion_trace.npz') as motion:
        sim.data.qpos[:] = motion['rows'][-1, 1:20]
    mujoco.mj_forward(sim.model, sim.data)
else:
    for _ in range(200):
        sim.step()
sim.model.vis.global_.offwidth = 1280
sim.model.vis.global_.offheight = 800
camera = mujoco.MjvCamera()
camera.lookat[:] = [1., .0, .6]
camera.distance, camera.elevation, camera.azimuth = 18., -48., 115.
with mujoco.Renderer(sim.model, height=800, width=1280) as renderer:
    options = mujoco.MjvOption()
    options.sitegroup[5] = 0
    renderer.update_scene(sim.data, camera=camera, scene_option=options)
    if args.run:
        from dougie_sim.demo_overlay import DemoControls, draw_overlay
        from dougie_sim.radiation import RadiationSample
        controls = DemoControls()
        controls.highlights.background = json.loads((args.run/'evaluator_field.json').read_text())['background_usv_h']
        samples = [json.loads(line) for line in (args.run/'measurements.jsonl').read_text().splitlines()]
        for sample in samples:
            controls.highlights.observe(RadiationSample(**sample))
        with np.load(args.run/'radiation_map_3d.npz') as archive:
            volume = tuple(archive[key] for key in ('xyz', 'mean_usv_h', 'std_log_rate', 'upper_usv_h'))
        draw_overlay(renderer.scene, volume, {}, [s['position'] for s in samples], controls, 100., reset=False)
    image = renderer.render()
    image = Image.fromarray(image)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    draw.rectangle((0, 0, 1280, 48), fill='#0b1828')
    caption = 'REACTOR / archived GP prediction and detector trajectory' if args.run else 'REACTOR / actual MuJoCo model with industrial service detail'
    draw.text((20, 12), caption, font=font, fill='#e2edf7')
    image.save(args.output)
    inputs = [args.run/name for name in ('scene.xml', 'motion_trace.npz', 'radiation_map_3d.npz', 'measurements.jsonl', 'evaluator_field.json')] if args.run else []
    args.output.with_suffix('.json').write_text(json.dumps(dict(
        scope='archived state and inferred-field replay, not live footage' if args.run else 'model render after one second of settling',
        run=str(args.run) if args.run else None, mujoco=mujoco.__version__,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scene_sha256=hashlib.sha256(sim.scene_xml.encode()).hexdigest(),
        input_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        image_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()), indent=2)+'\n')
    print(args.output)
