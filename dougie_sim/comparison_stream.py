"""Evaluator-only floor coverage and atomic live frames for paired missions."""
import json
import multiprocessing
import signal
import time

import mujoco
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

from .comparison_minimap import ComparisonMinimap
from .survey_comparison import outcome, project_admission, STATUS_NAMES


class FloorCoverage:
    """3-D floor observations credited only from physically reached elevations.

    Geometry and true scan transforms stay entirely outside the planner. Floor
    levels count separately; equipment tops and exterior ground are excluded.
    This is observed floor coverage, not a claim of traversability/completion.
    """
    resolution = .25
    metric = 'physically_reached_3d_floor_v2'
    observation_height_tolerance = .15
    arrival_height_tolerance = .08
    arrival_radius = .35

    def __init__(self, sim, world):
        model, data = sim.model, sim.data
        bounds = {'reactor': (-9.8, 9.8, -5.8, 5.8),
                  'facility': (-2.85, 15.85, -3.85, 11.85),
                  'mezzanine': (-2.85, 10.85, -2.05, 6.05)}[world]
        x0, x1, y0, y1 = bounds
        points = [[x, y, 0.] for x in np.arange(x0+.125, x1, .25)
                  for y in np.arange(y0+.125, y1, .25)]
        flat = [True]*len(points)
        prefixes = ('upper_floor_', 'service_ramp_', 'intermediate_landing',
                    'industrial_step_', 'mezzanine_slab', 'access_ramp_',
                    'step_up_', 'step_down_', 'inspection_deck')
        for i in range(model.ngeom):
            name = model.geom(i).name
            if not name.startswith(prefixes) or any(s in name for s in ('rail', 'post')):
                continue
            size, R = model.geom_size[i], data.geom_xmat[i].reshape(3, 3)
            for x in np.arange(-size[0]+.125, size[0], .25):
                for y in np.arange(-size[1]+.125, size[1], .25):
                    points.append((data.geom_xpos[i]+R@np.array([x, y, size[2]])).tolist())
                    flat.append(bool(np.allclose(R[:, 2], [0., 0., 1.], atol=1e-6)))
        # Keep exposed floor samples: the nearest downward hit must be that
        # floor, not equipment, walls, or a higher floor/ramp covering it.
        group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        geom = np.zeros(1, dtype=np.int32)
        valid, valid_flat = [], []
        for point, is_flat in zip(points, flat):
            p = np.asarray(point)
            distance = mujoco.mj_ray(model, data, p+[0., 0., .55],
                                    np.array([0., 0., -1.]), group, True, -1, geom)
            if abs(distance-.55) < .035 and model.geom_bodyid[geom[0]] == 0:
                valid.append(point)
                valid_flat.append(is_flat)
        self.points = np.asarray(valid).reshape(-1, 3)
        if not len(self.points):
            raise ValueError('no reference floor samples')
        self.tree = cKDTree(self.points)
        self.seen = np.zeros(len(self.points), dtype=bool)
        self.flat = np.asarray(valid_flat, dtype=bool)
        self.level_heights, ids = np.unique(np.round(self.points[self.flat, 2], 4), return_inverse=True)
        self.level_ids = np.full(len(self.points), -1, dtype=int)
        self.level_ids[self.flat] = ids
        self.reached_levels = np.zeros(len(self.level_heights), dtype=bool)

    def observe(self, points, *, support_position):
        """Use capture-time true foot-support XYZ, never sensor/body height.

        A flat elevation unlocks only beside its physical reference surface.
        Fresh scans can then credit cells within 15 cm of the current support
        height. Slopes use that continuous height gate without unlocking distant
        landings. Seeing another storey through a stairwell cannot survey it.
        """
        support = np.asarray(support_position, dtype=float)
        if support.shape != (3,) or not np.isfinite(support).all():
            raise ValueError('coverage requires a finite capture-time support XYZ')
        nearby = np.asarray(self.tree.query_ball_point(support, self.arrival_radius), dtype=int)
        arrived = nearby[self.flat[nearby] &
                         (np.abs(self.points[nearby, 2]-support[2]) <= self.arrival_height_tolerance)]
        self.reached_levels[self.level_ids[arrived]] = True
        points = np.asarray(points).reshape(-1, 3)
        if len(points):
            distance, index = self.tree.query(points)
            matched = index[(distance <= .19) &
                            (np.abs(self.points[index, 2]-support[2]) <= self.observation_height_tolerance)]
            eligible = ~self.flat[matched]
            flat = self.flat[matched]
            eligible[flat] = self.reached_levels[self.level_ids[matched[flat]]]
            self.seen[matched[eligible]] = True

    @property
    def percent(self):
        return float(100.*self.seen.mean())


def render_feed(model, output, snapshot, stop, feed_fps, map_state, points, world, start, awareness):
    """Separate OpenGL owner, using a single latest-pose slot (no frame queue)."""
    # The mission handles Ctrl+C and publishes its last pose before closing us.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=360, width=640)
    camera = mujoco.MjvCamera()
    camera.distance, camera.elevation, camera.azimuth = 7., -45., 115.
    options = mujoco.MjvOption()
    options.sitegroup[5] = 0
    minimap = ComparisonMinimap(points, world, start, awareness)
    route_shape = (len(minimap.levels), minimap.map_height, minimap.width)
    started = time.monotonic()
    frames, render_seconds, previous = 0, 0., 0.
    try:
        while True:
            # Read stop before the snapshot so the final published pose cannot
            # be skipped when shutdown arrives during a render.
            finished = stop.is_set()
            with snapshot.get_lock():
                values = np.frombuffer(snapshot.get_obj()).copy()
                map_values = np.frombuffer(map_state, dtype=np.uint8).copy()
            tick = time.monotonic()
            if values[0] != previous:
                previous = values[0]
                data.qpos[:] = values[2:]
                mujoco.mj_forward(model, data)
                camera.lookat[:] = data.qpos[:3]
                renderer.update_scene(data, camera=camera, scene_option=options)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                # Render-only cutaway; sensor/physics geometry is untouched.
                for geom in renderer.scene.geoms[:renderer.scene.ngeom]:
                    index = geom.objid
                    if (geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and 0 <= index < model.ngeom
                            and model.geom_bodyid[index] == 0 and geom.pos[2] > data.qpos[2]+.2):
                        geom.rgba[3] = min(float(geom.rgba[3]), .08)
                temporary = output/'frame.tmp'
                w, x, y, z = data.qpos[3:7]
                yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                overlay = minimap.draw(map_values[:len(points)], map_values[2*len(points):].reshape(route_shape),
                                       data.qpos[:2], values[1], yaw, map_values[len(points):2*len(points)])
                image = Image.fromarray(renderer.render())
                image.paste(overlay, (image.width-overlay.width-8, 8))
                image.save(temporary, format='JPEG', quality=85)
                temporary.replace(output/'frame.jpg')
                frames += 1
                render_seconds += time.monotonic()-tick
            if finished:
                break
            stop.wait(max(0., 1./feed_fps-(time.monotonic()-tick)))
    finally:
        elapsed = time.monotonic()-started
        (output/'stream_performance.json').write_text(json.dumps(dict(
            target_fps=feed_fps, frames=frames, wall_time_s=elapsed,
            average_fps=frames/max(elapsed, 1e-9), render_seconds=render_seconds), indent=2)+'\n')
        renderer.close()


class ComparisonStream:
    def __init__(self, sim, output, world, feed_fps=15, awareness='aware'):
        self.output = output
        self.coverage = FloorCoverage(sim, world)
        np.savez_compressed(output/'coverage_reference.npz', xyz=self.coverage.points,
                            resolution_m=self.coverage.resolution, flat=self.coverage.flat,
                            level_ids=self.coverage.level_ids, level_heights_m=self.coverage.level_heights,
                            coverage_metric=self.coverage.metric)
        self.log = (output/'comparison.jsonl').open('w')
        self.frame_interval = 1./feed_fps
        self.next_frame = self.next_metrics = 0.
        start = np.array([*sim.data.qpos[:2], self.support_height(sim)])
        self.minimap = ComparisonMinimap(self.coverage.points, world, start, awareness)
        self.admission = np.zeros(len(self.coverage.points), dtype=np.uint8)
        self.survey_report = {}
        context = multiprocessing.get_context('spawn')
        self.snapshot = context.Array('d', sim.model.nq+2)
        self.map_state = context.Array('B', 2*len(self.coverage.points)+self.minimap.route_masks().size, lock=False)
        self.stop = context.Event()
        self.process = context.Process(target=render_feed,
            args=(sim.model, output, self.snapshot, self.stop, feed_fps, self.map_state,
                  self.coverage.points, world, start, awareness), name='comparison-renderer')
        self.process.start()

    @staticmethod
    def support_height(sim):
        feet = [sim.data.body(name+'_foot').xpos[2]-.022 for name in ('FR', 'FL', 'RR', 'RL')]
        return float(np.mean(sorted(feet)[:2]))

    def update_admission(self, metrics, rotation, translation):
        admission = metrics.get('survey_admission', {})
        self.admission = project_admission(self.coverage.points, admission.get('positions', []),
                                           admission.get('statuses', []), rotation, translation)

    def capture(self, sim, dose, rate, state, force=False, metrics=None):
        support_height = self.support_height(sim)
        self.minimap.record(sim.data.qpos[:2], support_height)
        # Scoring uses simulated time; video uses wall time so slow physics does
        # not reduce the feed to one image per several seconds. Never queue old
        # frames or attempt catch-up rendering after a planner pause.
        if force or sim.data.time >= self.next_metrics:
            self.next_metrics = float(sim.data.time)+.5
            metrics = metrics or {}
            reason = metrics.get('terminal_stop', {}).get('reason') or metrics.get('decision_reason', '')
            self.survey_report = dict(
                outcome=outcome(bool(self.coverage.seen.all()), state, reason,
                                state == 'stopped_unrecoverable', state if state.startswith('failed:') else None),
                reason=reason, explored_percent=self.coverage.percent,
                reference_cells=len(self.coverage.points), explored_cells=int(self.coverage.seen.sum()),
                coverage_metric=self.coverage.metric,
                physically_reached_heights_m=self.coverage.level_heights[self.coverage.reached_levels].tolist(),
                admission_counts={name: int((self.admission == i).sum()) for i, name in enumerate(STATUS_NAMES)},
                scope='3D floor observations from physically reached elevations; admissibility only where known support/return routes exist')
            row = dict(time_s=float(sim.data.time), true_dose_usv=float(dose),
                       rate_usv_h=float(rate), explored_percent=self.coverage.percent, state=state,
                       survey_outcome=self.survey_report['outcome'], survey_reason=reason)
            self.log.write(json.dumps(row)+'\n')
            self.log.flush()
        now = time.monotonic()
        if not force and now < self.next_frame:
            return
        self.next_frame = now+self.frame_interval
        if not self.process.is_alive():
            raise RuntimeError(f'comparison renderer exited with {self.process.exitcode}')
        with self.snapshot.get_lock():
            values = np.frombuffer(self.snapshot.get_obj())
            values[0] += 1
            values[1] = support_height
            values[2:] = sim.data.qpos
            map_values = np.frombuffer(self.map_state, dtype=np.uint8)
            map_values[:len(self.coverage.points)] = self.coverage.seen
            map_values[len(self.coverage.points):2*len(self.coverage.points)] = self.admission
            map_values[2*len(self.coverage.points):] = self.minimap.route_masks().ravel()

    def close(self):
        (self.output/'survey_report.json').write_text(json.dumps(self.survey_report, indent=2)+'\n')
        np.savez_compressed(self.output/'minimap_final.npz', xyz=self.coverage.points,
                            seen=self.coverage.seen, admission=self.admission, route_masks=self.minimap.route_masks(),
                            slice_heights_m=self.minimap.heights, coverage_metric=self.coverage.metric,
                            reached_levels=self.coverage.reached_levels, level_heights_m=self.coverage.level_heights)
        self.log.close()
        self.stop.set()
        self.process.join(timeout=30.)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=5.)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
        if self.process.exitcode:
            raise RuntimeError(f'comparison renderer exited with {self.process.exitcode}')
