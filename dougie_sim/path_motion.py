"""Observed-support route shortening, certified corner rounding and tracking.

The graph remains the exploration/return admission model. Execution shortcuts
must pass continuous sampled clearance, support/connectivity and GP-dose checks.
This is a conservative discretized check, not a continuous safety proof.
"""
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


def sample_polyline(points, spacing=.05):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) < 2:
        return points.copy()
    chunks = [points[:1]]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(float(np.linalg.norm(b-a))/spacing))
        chunks.append(a+(b-a)*np.linspace(0., 1., n+1)[1:, None])
    return np.concatenate(chunks)


def path_length(points):
    return float(np.linalg.norm(np.diff(np.asarray(points).reshape(-1, 3), axis=0), axis=1).sum())


def turn_sum(points):
    delta = np.diff(np.asarray(points).reshape(-1, 3), axis=0)[:, :2]
    delta = delta[np.linalg.norm(delta, axis=1) > 1e-8]
    if len(delta) < 2:
        return 0.
    headings = np.arctan2(delta[:, 1], delta[:, 0])
    return float(np.abs(np.arctan2(np.sin(np.diff(headings)), np.cos(np.diff(headings)))).sum())


class RouteGeometry:
    def __init__(self, terrain):
        self.terrain = terrain
        self.columns = {}
        for key in terrain.nodes:
            self.columns.setdefault(key[:2], []).append(key)
        self.hits = np.asarray(list(terrain.hits.values())).reshape(-1, 3)
        self.tree = cKDTree(self.hits[:, :2]) if len(self.hits) else None
        self.planes = getattr(terrain, 'planes', {})
        self.spacing = min(.05, terrain.resolution/4.)

    def support(self, point, direction=None):
        scaled = point[:2]/self.terrain.resolution-.5
        cell = np.floor(scaled).astype(int)
        fraction = scaled-cell
        selected = {}
        height, gradient = 0., np.zeros(2)
        for dx, wx in ((0, 1.-fraction[0]), (1, fraction[0])):
            for dy, wy in ((0, 1.-fraction[1]), (1, fraction[1])):
                weight = wx*wy
                if weight < 1e-9:
                    continue
                column = int(cell[0]+dx), int(cell[1]+dy)
                choices = self.columns.get(column, ())
                if not choices:
                    return None
                def predicted(key):
                    node = self.terrain.nodes[key]
                    plane = self.planes.get(key)
                    return node[2] if plane is None else node[2]+plane[:2]@(point[:2]-node[:2])
                key = min(choices, key=lambda k: abs(predicted(k)-point[2]))
                z = predicted(key)
                if abs(z-point[2]) > .08:
                    return None
                selected[column] = key
                height += weight*z
                gradient += weight*(self.planes[key][:2] if key in self.planes else 0.)
        if not self.planes and selected:
            heights = [self.terrain.nodes[k][2] for k in selected.values()]
            if max(heights)-min(heights) > .02+1e-8:
                return None  # Do not turn an admitted discrete step into a ramp.
        # Bilinear support requires the intermediate cells, not diagonal corner
        # cutting. Also retain one-way or temporarily blocked edge restrictions.
        for (x, y), key in selected.items():
            for axis, other_column in ((0, (x+1, y)), (1, (x, y+1))):
                other = selected.get(other_column)
                if other is None:
                    continue
                if direction is None or abs(direction[axis]) < 1e-9:
                    if other not in self.terrain.graph.get(key, ()) or key not in self.terrain.graph.get(other, ()):
                        return None
                else:
                    a, b = (key, other) if direction[axis] > 0 else (other, key)
                    if b not in self.terrain.graph.get(a, ()):
                        return None
        return float(height), gradient

    def project(self, points, *, margin=.06, direction=None):
        points = np.asarray(points, dtype=float).reshape(-1, 3).copy()
        gradients = []
        for point in points:
            support = self.support(point, direction)
            if support is None:
                return None
            point[2], gradient = support
            gradients.append(gradient)
        if self.tree is not None:
            neighbours = self.tree.query_ball_point(points[:, :2], self.terrain.radius+margin)
            threshold = .10 if self.planes else .12
            for point, gradient, indices in zip(points, gradients, neighbours):
                hits = self.hits[indices]
                relative = hits[:, 2]-point[2]-(hits[:, :2]-point[:2])@gradient
                if np.any((relative > threshold) & (relative < .7)):
                    return None
        return points

    def line(self, a, b, margin=.06):
        if np.linalg.norm(np.asarray(b)[:2]-np.asarray(a)[:2]) < 1e-8 and abs(b[2]-a[2]) > .001:
            return None
        return self.project(sample_polyline([a, b], self.spacing), margin=margin, direction=np.asarray(b)-a)

    def corner(self, a, b, c):
        before, after = np.linalg.norm(b[:2]-a[:2]), np.linalg.norm(c[:2]-b[:2])
        trim = min(.45, .35*before, .35*after)
        if trim < .28:
            return None
        entry, exit = b+(a-b)*(trim/before), b+(c-b)*(trim/after)
        # Certify the convex triangle, not just the arc: lookahead chords inside
        # this corner piece must also remain in supported, clear space.
        n = max(2, math.ceil(2*trim/self.spacing))
        hull = [entry+(b-entry)*(i/n)+(exit-entry)*(j/n)
                for i in range(n+1) for j in range(n+1-i)]
        if self.project(hull) is None:
            return None
        t = np.linspace(0., 1., n+1)[:, None]
        curve = self.project((1-t)**2*entry+2*(1-t)*t*b+t*t*exit)
        if curve is None:
            return None
        lengths = np.linalg.norm(np.diff(curve[:, :2], axis=0), axis=1)
        angles = np.arctan2(np.diff(curve[:, 1]), np.diff(curve[:, 0]))
        curvature = np.abs(np.arctan2(np.sin(np.diff(angles)), np.cos(np.diff(angles))))/np.maximum((lengths[1:]+lengths[:-1])/2, 1e-8)
        return curve if not len(curvature) or curvature.max() <= 5. else None


@dataclass
class MotionRoute:
    points: list
    spans: list
    metrics: dict


def route_risk(points, gp, terrain):
    points = sample_polyline(points, .1)
    if len(points) < 2:
        return 0., 0.
    _, _, rates = gp.predict(points+[0., 0., .44])
    delta = np.diff(points, axis=0)
    distance = np.linalg.norm(delta, axis=1)
    if hasattr(terrain, 'max_slope'):
        slope = delta[:, 2]/np.maximum(np.linalg.norm(delta[:, :2], axis=1), 1e-8)
        speed = np.where(slope < -.05, .08, .1)
    else:
        speed = np.where(np.abs(delta[:, 2]) > 1e-6, .06, .1)
    return float(np.sum(np.maximum(rates[:-1], rates[1:])*distance/speed)/3600.), float(rates.max())


def smooth_route(raw_points, terrain, gp, rate_limit=100., returning=False):
    started = time.perf_counter()
    raw = np.asarray(raw_points, dtype=float).reshape(-1, 3)
    if len(raw):
        raw = raw[np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > 1e-7]]
    if len(raw) < 2:
        return MotionRoute([], [], dict(mode='empty', smoothing_ms=0.))
    geometry = RouteGeometry(terrain)
    raw_dose, raw_peak = route_risk(raw, gp, terrain)
    anchors = [raw[0]]
    certified = []
    i, checks = 0, 0
    while i < len(raw)-1:
        chosen, valid = i+1, False
        for j in range(min(len(raw)-1, i+12), i, -1):
            if checks >= 160:
                break
            checks += 1
            if geometry.line(raw[i], raw[j]) is not None:
                chosen, valid = j, True
                break
        anchors.append(raw[chosen])
        certified.append(valid)
        i = chosen
    anchors = np.asarray(anchors)
    corners = {}
    for i in range(1, len(anchors)-1):
        if certified[i-1] and certified[i] and turn_sum(anchors[i-1:i+2]) > .15:
            curve = geometry.corner(*anchors[i-1:i+2])
            if curve is not None:
                corners[i] = curve
    result, spans = [anchors[0].copy()], []
    def append(piece):
        start = len(result)-1
        result.extend(p.copy() for p in piece[1:] if np.linalg.norm(p-result[-1]) > 1e-7)
        if len(result)-1 > start:
            spans.append([start, len(result)-1])
    geometry_failed = False
    for i in range(len(anchors)-1):
        end = corners[i+1][0] if i+1 in corners else anchors[i+1]
        piece = geometry.line(result[-1], end, margin=.06 if certified[i] else 0.)
        if piece is None and certified[i]:
            geometry_failed = True
            break
        append(piece if piece is not None else [result[-1], end])
        if i+1 in corners:
            append(corners[i+1])
    dose, peak = route_risk(result, gp, terrain)
    fallback = None
    if geometry_failed or dose > raw_dose+1e-9 or peak > (max(rate_limit, raw_peak) if returning else rate_limit)+1e-9:
        fallback = 'geometry_revalidation_failed' if geometry_failed else 'predicted_exposure_or_rate_increase'
        result = [p.copy() for p in raw]
        spans = [[i, i+1] for i in range(len(result)-1)]
        dose = raw_dose
    return MotionRoute(result, spans, dict(mode='grid_fallback' if fallback else 'support_checked',
        fallback_reason=fallback, raw_waypoints=len(raw), execution_waypoints=len(result),
        raw_length_m=path_length(raw), execution_length_m=path_length(result),
        raw_turn_rad=turn_sum(raw), execution_turn_rad=turn_sum(result),
        raw_upper_dose_usv=raw_dose, execution_upper_dose_usv=dose,
        rounded_corners=len(corners) if not fallback else 0, geometry_checks=checks,
        smoothing_ms=(time.perf_counter()-started)*1000.))


class RouteFollower:
    """Arc-length tracking with convex-piece lookahead and command slew limits."""
    def __init__(self):
        self.points = np.empty((0, 3))
        self.spans = []
        self.command = np.zeros(3)
        self.progress = 0.
        self.status = 'idle'
        self.replan_rejections = 0
        self.route_accepted = True

    def set_route(self, points, spans, position):
        self.points = np.asarray(points, dtype=float).reshape(-1, 3)
        if not np.isfinite(self.points).all():
            raise ValueError('nonfinite route')
        self.spans = spans or [[i, i+1] for i in range(max(0, len(points)-1))]
        if len(points) > 1 and (not self.spans or self.spans[0][0] != 0 or self.spans[-1][1] != len(points)-1
            or any(a < 0 or b <= a or b >= len(points) for a, b in self.spans)
            or any(a[1] != b[0] for a, b in zip(self.spans, self.spans[1:]))):
            raise ValueError('route spans must cover the polyline contiguously')
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.progress = 0.
        self.route_accepted = True
        self.curvature = np.zeros(max(0, len(self.points)-1))
        for a, b in self.spans:
            delta = np.diff(self.points[a:b+1, :2], axis=0)
            lengths = np.linalg.norm(delta, axis=1)
            angles = np.arctan2(delta[:, 1], delta[:, 0])
            if len(delta) > 1:
                k = np.abs(np.arctan2(np.sin(np.diff(angles)), np.cos(np.diff(angles))))/np.maximum((lengths[1:]+lengths[:-1])/2, 1e-8)
                self.curvature[a:b-1] = np.maximum(self.curvature[a:b-1], k)
                self.curvature[a+1:b] = np.maximum(self.curvature[a+1:b], k)
        if len(self.points) > 1:
            # A delayed replan may start behind the robot. Join a nearby point
            # already on its observed prefix, instead of commanding backtracking.
            _, distance = self._project(np.asarray(position), prefix=1., joining=True)
            if distance > .12:
                self.route_accepted = False
                self.progress = 0.
                self.replan_rejections += 1
                self.status = 'waiting_for_route_reacquisition'
                self.stop()
                return False
            self.status = 'tracking'
            return True
        self.stop()
        self.status = 'idle'
        return True

    def stop(self):
        self.command[:] = 0.
        return self.command.copy()

    def _project(self, position, prefix=.8, joining=False):
        start = max(0, int(np.searchsorted(self.arc, self.progress, side='right'))-1)
        end = min(len(self.points)-1, int(np.searchsorted(self.arc, self.progress+prefix))+1)
        if joining:
            end = min(end, int(np.searchsorted(self.arc, prefix, side='right')))
        active = next((s for s in self.spans if s[0] <= start < s[1]), self.spans[-1])
        if not joining:
            end = min(end, active[1])
        if start >= end:
            return len(self.points)-2, float(np.linalg.norm(position-self.points[-1]))
        a, delta = self.points[start:end], np.diff(self.points[start:end+1], axis=0)
        t = np.clip(np.sum((position-a)*delta, axis=1)/np.maximum(np.sum(delta*delta, axis=1), 1e-12), 0., 1.)
        if joining:
            # The final segment can extend well beyond the bounded join window.
            maximum = np.clip((prefix-self.arc[start:end])/np.maximum(np.diff(self.arc[start:end+1]), 1e-12), 0., 1.)
            t = np.minimum(t, maximum)
        if not joining:
            t[0] = max(t[0], (self.progress-self.arc[start])/max(self.arc[start+1]-self.arc[start], 1e-12))
        projected = a+t[:, None]*delta
        distance = np.linalg.norm(projected-position, axis=1)
        index = int(np.argmin(distance))
        segment = start+index
        progress = float(self.arc[segment]+t[index]*(self.arc[segment+1]-self.arc[segment]))
        if not joining and progress >= self.arc[active[1]]-1e-9 and np.linalg.norm(position[:2]-self.points[active[1], :2]) >= .06:
            progress = self.arc[active[1]]-1e-7
        self.progress = max(self.progress, progress)
        return segment, float(distance[index])

    def tick(self, position, yaw, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('controller timestep must be positive and finite')
        if len(self.points) < 2:
            self.status = 'idle'
            return self.stop()
        if not self.route_accepted:
            self.status = 'waiting_for_route_reacquisition'
            return self.stop()
        position = np.asarray(position)
        if math.dist(position, self.points[-1]) < .12:
            self.status = 'at_goal'
            return self.stop()
        segment, error_distance = self._project(position)
        if error_distance > .12:
            self.status = 'waiting_for_route_reacquisition'
            return self.stop()
        span = next((s for s in self.spans if s[0] <= segment < s[1]), self.spans[-1])
        if (self.arc[span[1]]-self.progress < .035 and span[1] < len(self.points)-1
                and np.linalg.norm(position[:2]-self.points[span[1], :2]) < .06):
            segment = span[1]
            span = next((s for s in self.spans if s[0] == segment), span)
            self.progress = max(self.progress, self.arc[segment])
        carrot_arc = min(self.progress+.4, self.arc[span[1]])
        j = min(len(self.points)-2, max(0, int(np.searchsorted(self.arc, carrot_arc, side='right'))-1))
        fraction = (carrot_arc-self.arc[j])/max(self.arc[j+1]-self.arc[j], 1e-9)
        target = self.points[j]+fraction*(self.points[j+1]-self.points[j])
        delta = target-position
        error = math.atan2(math.sin(math.atan2(delta[1], delta[0])-yaw), math.cos(math.atan2(delta[1], delta[0])-yaw))
        distance = max(.08, float(np.linalg.norm(delta[:2])))
        curvature = 2.*math.sin(error)/distance
        speed = min(.22, .45/max(abs(curvature), 1e-6))
        preview_end = min(len(self.curvature), int(np.searchsorted(self.arc, self.progress+.65))+1)
        upcoming = float(np.max(self.curvature[segment:preview_end], initial=0.))
        speed = min(speed, .45/max(upcoming, 1e-6))
        if abs(delta[2])/distance > .05:
            speed = min(speed, .16)
        speed = min(speed, math.sqrt(.6*max(0., self.arc[-1]-self.progress)))
        if span[1] < len(self.points)-1:
            turn = turn_sum(self.points[span[1]-1:span[1]+2])
            if turn > .35:
                speed = min(speed, max(.02, self.arc[span[1]]-self.progress)*.8)
        if abs(error) > .7:
            desired = np.array([0., 0., np.clip(1.8*error, -.5, .5)])
        else:
            speed *= max(0., math.cos(error))
            desired = np.array([speed, 0., np.clip(speed*curvature, -.5, .5)])
        limits = np.array([.35 if desired[0] > self.command[0] else .6, 0., 1.]) * dt
        self.command += np.clip(desired-self.command, -limits, limits)
        self.status = 'tracking'
        return self.command.copy()
