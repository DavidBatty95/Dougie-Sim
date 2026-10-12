"""Operator-only MuJoCo overlays from measured samples and planner snapshots."""
import math

import mujoco
import numpy as np


class DemoControls:
    def __init__(self, camera='follow', layer='mean', show_sources=False):
        self.camera, self.layer = camera, layer
        self.regions, self.paths = True, True
        self.show_sources = show_sources
        self.applied_camera = None

    def key(self, keycode):
        if keycode == ord('C'):
            self.camera = 'overview' if self.camera == 'follow' else 'follow'
        elif keycode == ord('M'):
            self.layer = 'uncertainty' if self.layer == 'mean' else 'mean'
        elif keycode == ord('G'):
            self.regions = not self.regions
        elif keycode == ord('P'):
            self.paths = not self.paths
        elif keycode == ord('S'):
            self.show_sources = not self.show_sources


def hud_lines(metrics, sample, stamp, budget, layer, sources_visible=False):
    dose = 0. if sample is None else sample.dose_usv
    rate = 0. if sample is None else sample.rate_usv_h
    reserve = metrics.get('return_reserve_usv', 0.)
    used = min(20, max(0, round(20*dose/budget)))
    left = '\n'.join(('DOUGIE / RADIATION SURVEY', 'Mission', 'Decision', 'Simulation time',
        'Measured rate', 'Measured dose / budget', 'Estimated return reserve', 'Dose used',
        'Observed surface proxy', 'GP dictionary', 'Region candidates', 'View',
        'Source markers', 'Controls'))
    right = '\n'.join((metrics.get('planner', 'baseline'), metrics.get('state', 'initial_survey'),
        metrics.get('decision_reason', 'survey / route / return'), f'{stamp:6.1f} s',
        f'{rate:.2f} uSv/h', f'{dose:.4f} / {budget:g} uSv', f'{reserve:.4f} uSv',
        '['+'#'*used+'.'*(20-used)+']', f"{metrics.get('mapped_surface_area_m2', 0.):.1f} m2",
        str(metrics.get('gp', {}).get('inducing_points', 0)), str(metrics.get('candidate_count', 0)),
        layer, 'TRUTH / DIAGNOSTIC' if sources_visible else 'hidden',
        'C camera | M field/uncertainty | G regions | P paths | S sources'))
    return left, right


def draw_overlay(scene, map_view, metrics, trail, controls, rate_limit, reset=True):
    if reset:
        scene.ngeom = 0
    identity = np.eye(3).reshape(-1)
    def sphere(point, radius, color):
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius]*3), np.asarray(point, dtype=float), identity, np.asarray(color, dtype=np.float32))
        scene.ngeom += 1
    def lines(points, color, width=3.):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        for start, end in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom:
                break
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), start, identity, np.asarray(color, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, start+[0, 0, .10], end+[0, 0, .10])
            scene.ngeom += 1
    # Draw decisions before the optional field so a geometry cap cannot hide them.
    if controls.paths:
        for edge in metrics.get('blocked_edges', []):
            lines(edge, [1., .2, .2, .9], 5.)
        lines(metrics.get('return_path', []), [0.2, 1., .4, .85], 3.)
        lines(metrics.get('planned_path', []), [.1, .85, 1., 1.], 5.)
        if len(trail) > 1:
            lines(trail[::max(1, len(trail)//180)], [.9, .9, .9, .6], 2.)
        target = metrics.get('selected_target')
        if target is not None:
            sphere(np.asarray(target)+[0, 0, .2], .12, [1., .75, .05, 1.])
    if controls.regions:
        palette = ((.2, .8, 1.), (.8, .35, 1.), (.3, 1., .6), (1., .55, .2), (.9, .35, .6))
        for i, cluster in enumerate(metrics.get('clusters', [])[:96]):
            sphere(np.asarray(cluster['anchor'])+[0, 0, .12], .07, [*palette[i % len(palette)], .8])
    if map_view:
        query, mean, sigma, _ = map_view
        for i in range(0, len(query), max(1, len(query)//350)):
            if controls.layer == 'uncertainty':
                value = min(1., max(0., float(sigma[i])/1.5))
                color = [.2+.65*value, .7*(1.-value), 1., .24]
            else:
                value = min(1., math.log1p(float(mean[i]))/math.log1p(rate_limit))
                color = [value, 1.-value, .2, .24]
            sphere(query[i], .04, color)


def update_viewer(viewer, position, map_view, metrics, trail, controls, sample, stamp, budget, rate_limit):
    with viewer.lock():
        viewer.opt.sitegroup[5] = int(controls.show_sources)
        if controls.camera == 'follow':
            viewer.cam.lookat[:] = position
            if controls.applied_camera != controls.camera:
                viewer.cam.distance, viewer.cam.elevation = 5., -35.
        elif map_view and len(map_view[0]):
            points = map_view[0]
            viewer.cam.lookat[:] = (points.min(axis=0)+points.max(axis=0))/2
            if controls.applied_camera != controls.camera:
                viewer.cam.distance = max(9., float(np.max(np.ptp(points[:, :2], axis=0)))*1.35)
                viewer.cam.elevation = -65.
        controls.applied_camera = controls.camera
        draw_overlay(viewer.user_scn, map_view, metrics, trail, controls, rate_limit)
    left, right = hud_lines(metrics, sample, stamp, budget, controls.layer, controls.show_sources)
    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
