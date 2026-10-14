#!/usr/bin/env python3
"""Render the actual facility model; cutaway and all-target display are diagnostic."""
import hashlib
import json
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont

from dougie_sim.assets import ROOT
from dougie_sim.simulation import Simulation


def main():
    sim = Simulation('facility', landmark_count=8, landmark_seed=7)
    for _ in range(200):
        sim.step()
    for i in range(sim.model.ngeom):
        if sim.model.geom(i).name.startswith(('outer_', 'upper_floor_', 'room_partition_1_')):
            sim.model.geom_rgba[i, 3] = .2
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = 1280, 800
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [6., 4., 1.2]
    camera.distance, camera.elevation, camera.azimuth = 25., -48., 120.
    option = mujoco.MjvOption()
    option.sitegroup[5] = 0
    with mujoco.Renderer(sim.model, height=800, width=1280) as renderer:
        renderer.update_scene(sim.data, camera=camera, scene_option=option)
        image = Image.fromarray(renderer.render())
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    draw.rectangle((0, 0, 1280, 76), fill='#0b1828')
    draw.text((24, 12), 'MUJOCO FACILITY / DIAGNOSTIC CUTAWAY', font=font, fill='#e2edf7')
    draw.text((24, 43), '2.4 m floors · switchback ramps · seeded cyan targets · all-target view is evaluator-only', font=font, fill='#9ed9df')
    output = ROOT/'docs/media/dougie/facility-cutaway.png'
    image.save(output)
    manifest = dict(scope='actual MuJoCo model render after 1 s settling; diagnostic transparency and all targets, not a mission recording',
        world='facility', layout_seed=0, landmark_count=8, landmark_seed=7, mujoco=mujoco.__version__,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scene_sha256=hashlib.sha256(sim.scene_xml.encode()).hexdigest(),
        image_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    output.with_suffix('.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(output)


if __name__ == '__main__':
    main()
