"""Optional offscreen recording of actual physics plus the live planning overlay."""
import hashlib
import json

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .demo_overlay import draw_overlay


class DemoRecorder:
    def __init__(self, model, output, duration):
        self.output = output
        self.renderer = mujoco.Renderer(model, height=360, width=640)
        self.camera = mujoco.MjvCamera()
        self.camera.distance, self.camera.elevation, self.camera.azimuth = 6., -45., 115.
        self.options = mujoco.MjvOption()
        self.frames, self.stamps = [], []
        self.interval = max(.5, duration/180.)
        self.next_frame = 0.
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15)

    def capture(self, sim, map_view, metrics, trail, controls, sample, budget, rate_limit, force=False):
        if not force and sim.data.time < self.next_frame:
            return
        if len(self.frames) >= 181:
            return
        self.next_frame = sim.data.time+self.interval
        self.camera.lookat[:] = sim.data.qpos[:3]
        if controls.camera == 'overview' and map_view is not None:
            points = map_view[0]
            self.camera.lookat[:] = (points.max(axis=0)+points.min(axis=0))/2
            self.camera.distance = max(10., float(np.max(np.ptp(points[:, :2], axis=0)))*1.35)
            self.camera.elevation = -65.
        self.options.sitegroup[5] = int(controls.show_sources)
        self.renderer.update_scene(sim.data, camera=self.camera, scene_option=self.options)
        draw_overlay(self.renderer.scene, map_view, metrics, trail, controls, rate_limit, reset=False)
        image = Image.new('RGB', (640, 490), '#0b1724')
        image.paste(Image.fromarray(self.renderer.render()), (0, 42))
        draw = ImageDraw.Draw(image)
        draw.text((14, 10), f"DOUGIE / {metrics.get('planner', 'baseline')} / {metrics.get('state', 'survey')}", fill='#48daca', font=self.font)
        dose, rate = (sample.dose_usv, sample.rate_usv_h) if sample is not None else (0., 0.)
        draw.text((14, 410), f't={sim.data.time:.1f}s   measured {rate:.2f} uSv/h   dose {dose:.3f}/{budget:g} uSv', fill='white', font=self.font)
        draw.text((14, 438), f"Observed surface proxy {metrics.get('mapped_surface_area_m2', 0.):.1f} m2 | cyan route / green return", fill='#a7bdcf', font=self.font)
        draw.text((14, 464), f"{controls.layer} field | region markers | {self.interval/.1:.1f}x simulated-time playback", fill='#a7bdcf', font=self.font)
        self.frames.append(image)
        self.stamps.append(float(sim.data.time))

    def close(self):
        self.renderer.close()
        if not self.frames:
            return
        palette = self.frames[len(self.frames)//2].quantize(colors=96)
        frames = [frame.quantize(palette=palette, dither=0) for frame in self.frames]
        path = self.output/'demo.gif'
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=100, loop=0, disposal=2)
        (self.output/'demo_manifest.json').write_text(json.dumps(dict(
            kind='actual native MuJoCo run with live planner/GP overlay, accelerated playback',
            frame_times_s=self.stamps, frame_duration_ms=100, size=[640, 490],
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            note='Model truth is rendered to the operator; robot planning uses observed geometry and detector samples.'), indent=2)+'\n')
