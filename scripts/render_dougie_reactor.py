#!/usr/bin/env python3
"""Render the actual MuJoCo reactor model for the project overview."""
from pathlib import Path

import mujoco
from PIL import Image

from dougie_sim.simulation import Simulation

sim = Simulation('reactor')
for _ in range(200):
    sim.step()
sim.model.vis.global_.offwidth = 1280
sim.model.vis.global_.offheight = 800
camera = mujoco.MjvCamera()
camera.lookat[:] = [1., .0, .6]
camera.distance, camera.elevation, camera.azimuth = 18., -48., 115.
with mujoco.Renderer(sim.model, height=800, width=1280) as renderer:
    renderer.update_scene(sim.data, camera=camera)
    image = renderer.render()
    path = Path('docs/media/dougie-reactor.png')
    Image.fromarray(image).save(path)
    print(path)
