"""Operator-only MuJoCo overlays from measured samples and planner snapshots."""
import math

import mujoco
import numpy as np


class DemoControls:
    def __init__(self, camera='follow', layer='mean', show_sources=False):
        from .radiation_display import RadiationHighlights
        self.highlights = RadiationHighlights()
        self.field_display = 'detected'
        self.camera, self.layer = camera, layer
        self.regions, self.paths = False, True
        self.show_sources = show_sources
        self.applied_camera = None
        self.sources = []

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
    enabled = metrics.get('dose_budget_enabled', True)
    used = min(20, max(0, round(20*dose/budget))) if enabled else 0
    left = '\n'.join(('DOUGIE / RADIATION SURVEY', 'Mission', 'Decision', 'Simulation time',
        'Measured rate', 'Measured dose / budget', 'Estimated return reserve', 'Dose used',
        'Observed surface proxy', 'GP dictionary', 'Region candidates', 'View',
        'Source markers', 'Known targets pending', 'Agenda tasks', 'Return forecast', 'Controls'))
    right = '\n'.join((metrics.get('planner', 'baseline'), metrics.get('state', 'initial_survey'),
        metrics.get('decision_reason', 'survey / route / return'), f'{stamp:6.1f} s',
        f'{rate:.2f} uSv/h', f'{dose:.4f} / {budget:g} uSv' if enabled else f'{dose:.4f} uSv / NO CAP', f'{reserve:.4f} uSv',
        '['+'#'*used+'.'*(20-used)+']' if enabled else 'MONITOR / MINIMIZE', f"{metrics.get('mapped_surface_area_m2', 0.):.1f} m2",
        str(metrics.get('gp', {}).get('inducing_points', 0)), str(metrics.get('candidate_count', 0)),
        layer+' (robot visibility fade)', 'TRUTH / DIAGNOSTIC' if sources_visible else 'hidden',
        str(metrics.get('known_landmarks_pending', 0)), str(metrics.get('exploration_agenda', {}).get('size', 0)),
        f"{metrics.get('route_forecast', {}).get('return_duration_s', 0.):.0f} s",
        'C camera | M field/uncertainty | G regions | P paths | S sources'))
    return left, right


def draw_overlay(scene, map_view, metrics, trail, controls, rate_limit, reset=True, robot_position=None):
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
    if controls.show_sources:
        for name, point, color, height in controls.sources:
            beacon = np.array([point[0], point[1], height])
            sphere(point, .25, color)
            before = scene.ngeom
            sphere(beacon, .225, color)
            if scene.ngeom > before:
                scene.geoms[scene.ngeom-1].label = 'TRUE SOURCE: '+name
            lines([point, beacon], color, 6.)
            ring = [beacon+[.6*np.cos(a), .6*np.sin(a), 0.] for a in np.linspace(0., 2*np.pi, 33)]
            lines(ring, color, 4.)
    if controls.paths:
        for edge in metrics.get('blocked_edges', []):
            lines(edge, [1., .2, .2, .9], 5.)
        lines(metrics.get('return_path', []), [1., .65, .1, .9], 4.)
        plan = np.asarray(metrics.get('planned_path', []), dtype=float).reshape(-1, 3)
        lines(plan, [.1, 1., .25, 1.], 6.)
        # Direction chevrons spaced by route distance, rather than point density.
        travelled = 0.
        for first, last in zip(plan[:-1], plan[1:]):
            delta = last-first
            length = np.linalg.norm(delta[:2])
            travelled += length
            if travelled < .8 or length < 1e-6:
                continue
            travelled = 0.
            forward = delta/length
            side = np.array([-forward[1], forward[0], 0.])
            tip = (first+last)/2
            lines([tip-.25*forward+.12*side, tip, tip-.25*forward-.12*side], [.1, 1., .25, 1.], 5.)
        if len(trail) > 1:
            lines(trail[::max(1, len(trail)//180)], [.9, .9, .9, .6], 2.)
        target = metrics.get('selected_target')
        if target is not None:
            centre = np.asarray(target)+[0, 0, .12]
            ring = [centre+[.3*np.cos(a), .3*np.sin(a), 0.] for a in np.linspace(0., 2*np.pi, 25)]
            lines(ring, [.1, 1., .25, 1.], 5.)
            sphere(centre, .08, [.1, 1., .25, 1.])
    if controls.regions:
        for cluster in metrics.get('clusters', [])[:8]:
            sphere(np.asarray(cluster['anchor'])+[0, 0, .12], .08, [.2, .8, 1., .8])
    if map_view:
        from .radiation_display import CLOUD_RADII, fade_near_robot
        query, colors = controls.highlights.cloud(map_view, controls.field_display, controls.layer)
        colors = fade_near_robot(query, colors, robot_position)
        for point, color in zip(query, colors):
            if color[3] < .001:
                continue
            if scene.ngeom >= scene.maxgeom:
                break
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                CLOUD_RADII, point, identity, color.astype(np.float32))
            scene.geoms[scene.ngeom].specular = 0.
            scene.geoms[scene.ngeom].shininess = 0.
            scene.geoms[scene.ngeom].category = mujoco.mjtCatBit.mjCAT_DECOR
            scene.geoms[scene.ngeom].emission = .4
            scene.ngeom += 1


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
        draw_overlay(viewer.user_scn, map_view, metrics, trail, controls, rate_limit, robot_position=position)
    left, right = hud_lines(metrics, sample, stamp, budget, controls.layer, controls.show_sources)
    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
