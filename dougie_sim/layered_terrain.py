"""Observed slope-aware support graph for the experimental multi-level profile.

Local planes are fit to actual support returns, never simulator geometry. A
smooth ramp and an abrupt ledge with the same average gradient are not equivalent.
Industrial stair admission is deliberately absent until a policy is qualified.
"""
import math
import itertools
from collections import Counter

import numpy as np
from scipy.spatial import cKDTree

from .terrain import SurfaceMap


class VoxelEvidence:
    def __init__(self, resolution=.2, capacity=200000):
        self.resolution, self.capacity = resolution, capacity
        self.evidence = {}
        self.coarse_free = {}
        self.compactions = 0
        self.saturated = False

    @property
    def storage_cells(self):
        return len(self.evidence)+len(self.coarse_free)

    def compact_free(self):
        """Merge only fully observed 2x2x2 free blocks, never occupied/unknown.

        Retain the least confident child value. A later hit expands the block;
        if expansion cannot fit, discard its free claim before retaining the hit.
        """
        groups = {}
        for key, value in self.evidence.items():
            if value <= -3:
                groups.setdefault(tuple(v//2 for v in key), []).append(key)
        merged = 0
        for block, keys in groups.items():
            if len(keys) == 8:
                self.coarse_free[block] = max(self.evidence.pop(key) for key in keys)
                merged += 1
        self.compactions += merged
        return merged

    def add_evidence(self, key, increment):
        block = tuple(v//2 for v in key)
        if block in self.coarse_free:
            if increment < 0:
                return
            value = self.coarse_free.pop(block)
            if self.storage_cells+8 <= self.capacity:
                for offset in itertools.product((0, 1), repeat=3):
                    child = tuple(2*b+d for b, d in zip(block, offset))
                    self.evidence[child] = value
            # A positive return must not remain hidden inside a coarse free block.
        if key not in self.evidence and self.storage_cells >= self.capacity:
            self.saturated = True
            return
        self.evidence[key] = max(-8, min(8, self.evidence.get(key, 0)+increment))

    def update(self, packet):
        if self.storage_cells >= self.capacity*.75:
            self.compact_free()
        directions = packet.directions.reshape(-1, 3)[::8]
        ranges = packet.distances.reshape(-1)[::8]
        hits = (ranges > .05) & (ranges <= packet.max_range_m)
        usable = hits | (ranges == -1.)
        directions, ranges, hits = directions[usable], ranges[usable], hits[usable]
        distances = np.where(hits, ranges, packet.max_range_m)
        steps = np.arange(0., packet.max_range_m, self.resolution*.5)
        # Stop before the hit voxel; no free-space clearing beyond a surface.
        valid = steps[None, :] < (distances[:, None]-self.resolution)
        points = packet.origin+directions[:, None, :]*steps[None, :, None]
        free = np.unique(np.floor(points[valid]/self.resolution).astype(int), axis=0)
        occupied = np.unique(np.floor((packet.origin+directions[hits]*ranges[hits, None])/self.resolution).astype(int), axis=0)
        for keys, increment in ((occupied, 3), (free, -1)):
            for key in map(tuple, keys):
                self.add_evidence(key, increment)


class LayeredSurfaceMap(SurfaceMap):
    def __init__(self, resolution=.2, radius=.45, max_slope_deg=10., residual_limit=.025, support_capacity=160000,
                 support_fusion='maximum'):
        super().__init__(resolution=resolution, radius=radius, max_step=.02)
        if support_fusion not in ('maximum', 'bounded-mean'):
            raise ValueError('unknown support fusion mode')
        self.support_fusion = support_fusion
        self.fusion_span_m = .016
        self._support_moments = {}
        self.max_slope = math.tan(math.radians(max_slope_deg))
        self.residual_limit = residual_limit
        # Fit curvature locally, but certify a wider body/swing-foot footprint.
        # A single plane across the whole footprint rejects valid ramp junctions.
        self.plane_radius = min(radius, .32)
        self.support_returns = {}
        self.planes = {}
        self.edge_kind = {}
        self.occupancy = VoxelEvidence()
        self.last_scan_stamp = None
        self.support_saturated = False
        self.observed_surface_cells = 0
        self.support_capacity = support_capacity
        self.active_viewpoints = True
        self._candidate_cache = {}
        self._dirty_columns = set()
        self.fitted_columns = self.reused_columns = 0
        self._footprint_candidates = {}
        self._footprint_planes = {}
        self._supported_keys = set()
        self.footprints_checked = 0
        self.fit_rejections = {}

    def observe_packet(self, packet):
        if self.last_scan_stamp is not None and packet.stamp_s < self.last_scan_stamp:
            raise ValueError('out-of-order range packet')
        self.update(packet.points, packet.supports, packet.origin)
        self.occupancy.update(packet)
        self.last_scan_stamp = packet.stamp_s

    def update(self, points, supports, origin=None):
        if self._candidate_cache and (len(points) or len(supports)):
            changed = np.vstack((np.asarray(points).reshape(-1, 3), np.asarray(supports).reshape(-1, 3)))
            cells = np.unique(np.floor(changed[:, :2]/self.resolution).astype(int), axis=0)
            margin = math.ceil(self.radius/self.resolution)+1
            offsets = np.array([(x, y) for x in range(-margin, margin+1) for y in range(-margin, margin+1)])
            self._dirty_columns.update(map(tuple, np.unique((cells[:, None, :]+offsets).reshape(-1, 2), axis=0)))
        super().update(points, supports, origin)
        if len(supports):
            if self.support_fusion == 'bounded-mean':
                self._fuse_supports(np.asarray(supports))
                return
            # Exact floor hits can be +/- machine epsilon around a voxel boundary.
            # Stabilize quantization instead of storing two copies of each floor.
            keys, indices = np.unique(np.floor(np.asarray(supports)/.06+1e-8).astype(int), axis=0, return_index=True)
            for key, index in zip(map(tuple, keys), indices):
                if key in self.support_returns or len(self.support_returns) < self.support_capacity:
                    point = np.asarray(supports[index])
                    # In a fine voxel containing both a ramp top and the floor
                    # beneath it, retain the actually observed upper support.
                    if key not in self.support_returns or point[2] > self.support_returns[key][2]:
                        self.support_returns[key] = point.copy()
                else:
                    self.support_saturated = True

    def _fuse_supports(self, supports):
        """Equal-weight capture means, with permanent retention of height jumps.

        A narrow (<=16 mm) height band may be fused inside a 6 cm XYZ voxel.
        Wider bands retain their highest actual return, even if seen only once.
        Extrema include every return, so a small ledge cannot be outvoted by
        repeated floor observations. This is a measurement model, not a change
        to slope, discontinuity, footprint or body-clearance admission limits.
        """
        keys, inverse, counts = np.unique(np.floor(supports/.06+1e-8).astype(int),
                                          axis=0, return_inverse=True, return_counts=True)
        order = np.argsort(inverse, kind='stable')
        starts = np.r_[0, np.cumsum(counts)[:-1]]
        means = np.add.reduceat(supports[order], starts, axis=0)/counts[:, None]
        lows = np.minimum.reduceat(supports[order, 2], starts)
        height_order = np.lexsort((supports[:, 2], inverse))
        peaks = supports[height_order[np.cumsum(counts)-1]]
        for index, key in enumerate(map(tuple, keys)):
            if key not in self.support_returns and len(self.support_returns) >= self.support_capacity:
                self.support_saturated = True
                continue
            previous = self._support_moments.get(key)
            if previous is None:
                count, mean, low, peak = 1, means[index].copy(), float(lows[index]), peaks[index].copy()
            else:
                count, mean, low, peak = previous
                count += 1
                mean = mean+(means[index]-mean)/count
                low = min(low, float(lows[index]))
                if peaks[index, 2] > peak[2]:
                    peak = peaks[index].copy()
            self._support_moments[key] = count, mean, low, peak
            self.support_returns[key] = (mean if peak[2]-low <= self.fusion_span_m else peak).copy()

    def build_graph(self):
        points = np.asarray(list(self.support_returns.values())).reshape(-1, 3)
        if not len(points):
            self.nodes, self.graph, self.planes, self.edge_kind = {}, {}, {}, {}
            return self.nodes, self.graph
        # A thin ramp can expose the floor beneath it from a side viewpoint.
        # Those lower returns are genuine, but not traversable with a Go2 body
        # under the overlying support. Do not fit the two surfaces as roughness.
        fine_xy = np.floor(points[:, :2]/.06+1e-8).astype(int)
        order = np.lexsort((points[:, 2], fine_xy[:, 1], fine_xy[:, 0]))
        same_column = np.all(fine_xy[order[:-1]] == fine_xy[order[1:]], axis=1)
        separation = np.diff(points[order, 2])
        covered = np.zeros(len(points), dtype=bool)
        covered[order[:-1]] = same_column & (separation > .008) & (separation < .7)
        points = points[~covered]
        tree = cKDTree(points[:, :2])
        # Check local continuity independently of the plane residual: the latter
        # needs tolerance at a smooth floor/ramp junction, but must not admit steps.
        neighbours = cKDTree(points).query(points, k=min(8, len(points)))[1]
        delta = points[neighbours]-points[:, None, :]
        horizontal = np.linalg.norm(delta[:, :, :2], axis=2)
        discontinuous = np.any((np.linalg.norm(delta, axis=2) < .15) &
            (np.abs(delta[:, :, 2]) > self.max_slope*horizontal+.008), axis=1)
        jump_excess = np.where(np.linalg.norm(delta, axis=2) < .15,
                               np.abs(delta[:, :, 2])-self.max_slope*horizontal, 0.)
        cells = np.floor(points[:, :2]/self.resolution).astype(int)
        columns, first, inverse, counts = np.unique(cells, axis=0, return_index=True,
                                                     return_inverse=True, return_counts=True)
        grouped = np.argsort(inverse, kind='stable')
        boundaries = np.r_[0, np.cumsum(counts)]
        centres = (columns+.5)*self.resolution
        support_neighbours = tree.query_ball_point(centres, self.plane_radius, return_sorted=False)
        candidates, planes = {}, {}
        body_blocked = set()
        self.observed_surface_cells = 0
        self.fitted_columns = self.reused_columns = 0
        hits = np.asarray(list(self.hits.values())).reshape(-1, 3)
        hit_tree = cKDTree(hits[:, :2]) if len(hits) else None
        hit_neighbours = hit_tree.query_ball_point(centres, self.radius, return_sorted=False) if hit_tree else None
        # Split vertically separated observations before fitting any local plane.
        for column_index in np.argsort(first):
            column = tuple(columns[column_index])
            if column in self._candidate_cache and column not in self._dirty_columns:
                count, cached = self._candidate_cache[column]
                self.observed_surface_cells += count
                for key, point, plane, blocked in cached:
                    candidates[key], planes[key] = point, plane
                    if blocked:
                        body_blocked.add(key)
                self.reused_columns += 1
                continue
            self.fitted_columns += 1
            observations = points[grouped[boundaries[column_index]:boundaries[column_index+1]]]
            if np.ptp(observations[:, 2]) <= .12:
                layers = (observations,)
            else:
                ordered = observations[np.argsort(observations[:, 2], kind='stable')]
                layers = np.split(ordered, np.flatnonzero(np.diff(ordered[:, 2]) > .12)+1)
            self.observed_surface_cells += len(layers)
            entries = []
            self._candidate_cache[column] = len(layers), entries
            rejected = self.fit_rejections[column] = []
            xy = centres[column_index]
            indices = support_neighbours[column_index]
            nearby = points[indices]
            for layer in layers:
                z = float(np.median(layer[:, 2]))
                if np.any(discontinuous[indices] & (np.abs(nearby[:, 2]-z) < .16)):
                    bad = np.asarray(indices)[discontinuous[indices] & (np.abs(nearby[:, 2]-z) < .16)]
                    offender = bad[np.argmax(np.max(jump_excess[bad], axis=1))]
                    neighbour = neighbours[offender, np.argmax(jump_excess[offender])]
                    rejected.append(dict(height_m=z, reason='local_discontinuity',
                        excess_m=float(np.max(jump_excess[offender])),
                        observed_pair=[points[offender].tolist(), points[neighbour].tolist()]))
                    continue
                local = nearby[np.abs(nearby[:, 2]-z) < .16]
                if len(local) < 6:
                    rejected.append(dict(height_m=z, reason='insufficient_returns', returns=len(local)))
                    continue
                design = np.column_stack((local[:, :2]-xy, np.ones(len(local))))
                plane, _, rank, _ = np.linalg.lstsq(design, local[:, 2], rcond=None)
                if rank < 3 or np.linalg.norm(plane[:2]) > self.max_slope+1e-4:
                    rejected.append(dict(height_m=z, reason='rank_or_slope', rank=int(rank),
                                         slope=float(np.linalg.norm(plane[:2]))))
                    continue
                if np.max(np.abs(design@plane-local[:, 2])) > self.residual_limit:
                    rejected.append(dict(height_m=z, reason='plane_residual',
                                         residual_m=float(np.max(np.abs(design@plane-local[:, 2])))))
                    continue
                blocked = False
                if hit_tree is not None:
                    near_hits = hits[hit_neighbours[column_index]]
                    relative = near_hits[:, 2]-(near_hits[:, :2]-xy)@plane[:2]-plane[2]
                    blocked = bool(np.any((relative > .10) & (relative < .7)))
                key = (*column, round(float(plane[2])/.02))
                candidates[key] = np.r_[xy, plane[2]]
                planes[key] = plane
                if blocked:
                    body_blocked.add(key)
                entries.append((key, candidates[key], plane, blocked))
        by_column = {}
        for key in candidates:
            by_column.setdefault(key[:2], []).append(key)
        # Require support over the full body/swing-foot footprint, including
        # diagonals. Four immediate neighbours alone admitted unsafe deck edges.
        reach = math.ceil(self.radius/self.resolution)
        footprint = [(dx, dy) for dx in range(-reach, reach+1) for dy in range(-reach, reach+1)
                      if math.hypot(dx, dy)*self.resolution <= self.radius]
        changed = {key[:2] for key in self._footprint_candidates.keys() | candidates.keys()
                   if key not in candidates or key not in self._footprint_candidates
                   or not np.array_equal(candidates[key], self._footprint_candidates[key])
                   or not np.array_equal(planes[key], self._footprint_planes[key])}
        affected = {(x-dx, y-dy) for x, y in changed for dx, dy in footprint}
        self.footprints_checked = 0
        nodes = {}
        def compatible(a, b):
            delta = candidates[b]-candidates[a]
            return (abs(delta[2]) <= self.max_slope*np.linalg.norm(delta[:2])+.003
                    and max(abs(delta[2]-planes[a][:2]@delta[:2]),
                            abs(delta[2]-planes[b][:2]@delta[:2])) <= .015)
        for key, p in candidates.items():
            # Body clearance is tested once at the robot centre. Neighbouring
            # footprint cells need observed support, not another whole robot's
            # clearance; applying both erodes narrow connectors twice.
            if key in body_blocked:
                continue
            if key[:2] not in affected:
                if key in self._supported_keys:
                    nodes[key] = p
                continue
            self.footprints_checked += 1
            selected = {}
            for dx, dy in footprint:
                xy = (key[0]+dx, key[1]+dy)
                choices = [k for k in by_column.get(xy, [])
                           if abs(candidates[k][2]-p[2]) <= self.max_slope*math.hypot(dx, dy)*self.resolution+.015]
                if not choices:
                    break
                predicted = p[2]+planes[key][:2]@np.array([dx, dy])*self.resolution
                selected[dx, dy] = key if (dx, dy) == (0, 0) else min(choices, key=lambda k: abs(candidates[k][2]-predicted))
            # Piecewise-continuous support permits a measured floor/ramp join.
            # Every adjacent pair must agree, so an abrupt ledge cannot become
            # admissible merely because its two sides connect elsewhere.
            if len(selected) == len(footprint) and all(
                    compatible(k, selected[other]) for (dx, dy), k in selected.items()
                    for other in ((dx+1, dy), (dx, dy+1)) if other in selected):
                nodes[key] = p
        self._footprint_candidates, self._footprint_planes = candidates, planes
        self._supported_keys = set(nodes)
        graph = {key: [] for key in nodes}
        kinds = {}
        for key, p in nodes.items():
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                for other in by_column.get((key[0]+dx, key[1]+dy), []):
                    if other not in nodes:
                        continue
                    q = nodes[other]
                    dz = q[2]-p[2]
                    if abs(dz) > self.max_slope*self.resolution+.003:
                        continue
                    # Both endpoint fits must support this edge, avoiding step shortcuts.
                    if max(abs(dz-planes[key][:2]@(q[:2]-p[:2])),
                           abs(dz-planes[other][:2]@(q[:2]-p[:2]))) > .015:
                        continue
                    graph[key].append(other)
                    kinds[key, other] = 'ramp_up' if dz > .01 else 'ramp_down' if dz < -.01 else 'level'
        self.nodes, self.graph, self.planes, self.edge_kind = nodes, graph, planes, kinds
        self._dirty_columns.clear()
        self._edge_lengths = {(a, b): math.dist(nodes[a], nodes[b]) for a, neighbours in graph.items() for b in neighbours}
        return nodes, graph

    def edge_time(self, start, end, speed=.1):
        kind = self.edge_kind.get((start, end), 'level')
        limit = .08 if kind == 'ramp_down' else .10
        return math.dist(self.nodes[start], self.nodes[end])/min(speed, limit)

    def acquisition_bonus(self, key, sampled_bands):
        band = round(float(self.nodes[key][2])/.2)
        new_height = 6. if band not in sampled_bands else 0.
        plane = self.planes.get(key)
        connector = 4. if plane is not None and np.linalg.norm(plane[:2]) > .05 else 0.
        return new_height+connector

    def support_diagnosis(self, position, limit=16):
        """Bounded, capture-time observed-map diagnostics; never simulator geometry."""
        xy = np.asarray(position)[:2]
        columns = sorted(self._candidate_cache, key=lambda key: (math.dist((np.asarray(key)+.5)*self.resolution, xy), key))
        result = []
        for column in columns[:limit]:
            count, entries = self._candidate_cache[column]
            result.append(dict(xy=((np.asarray(column)+.5)*self.resolution).tolist(), observed_layers=count,
                rejections=self.fit_rejections.get(column, []),
                candidates=[dict(height_m=float(point[2]), admission='body_blocked' if blocked else
                    'admitted' if key in self.nodes else 'footprint_rejected') for key, point, plane, blocked in entries]))
        return dict(position=np.asarray(position).tolist(), columns=result)

    def metrics(self):
        return dict(support_nodes=len(self.nodes), support_returns=len(self.support_returns),
                    support_fusion=self.support_fusion, support_fusion_cells=len(self._support_moments),
                    support_fusion_span_m=self.fusion_span_m,
                    support_peak_retention_cells=sum(bool(peak[2]-low > self.fusion_span_m)
                        for count, mean, low, peak in self._support_moments.values()),
                    fit_rejections=dict(Counter(item['reason'] for column in self._candidate_cache
                        for item in self.fit_rejections.get(column, []))),
                    support_capacity=self.support_capacity, fitted_columns=self.fitted_columns, reused_columns=self.reused_columns,
                    footprints_checked=self.footprints_checked,
                    support_cache_saturated=self.support_saturated,
                    occupied_voxels=sum(v > 0 for v in self.occupancy.evidence.values()),
                    free_voxels=sum(v < 0 for v in self.occupancy.evidence.values())+8*len(self.occupancy.coarse_free),
                    occupancy_storage_cells=self.occupancy.storage_cells,
                    coarse_free_blocks=len(self.occupancy.coarse_free), free_block_compactions=self.occupancy.compactions,
                    occupancy_saturated=self.occupancy.saturated,
                    ramp_edges=sum(k != 'level' for k in self.edge_kind.values()),
                    last_scan_stamp_s=self.last_scan_stamp)
