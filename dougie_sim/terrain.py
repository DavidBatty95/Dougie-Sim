"""Observed 3-D surface graph, built only from range returns and local normals.

Multiple support heights per XY cell are retained. This avoids flattening an
upper inspection deck into the ground floor. Edges enforce measured step limits.
"""
import heapq
import math

import numpy as np
from scipy.spatial import cKDTree


class SurfaceMap:
    def __init__(self, resolution=.25, max_step=.075, radius=.3, max_local_relief=None):
        self.resolution, self.max_step, self.radius = resolution, max_step, radius
        self.surfaces = {}
        self.hits = {}
        self.free_voxels = set()
        self.nodes = {}
        self.graph = {}
        self.version = 0
        self.max_local_relief = max_local_relief
        self._known_columns = None
        self._edge_lengths = {}

    def update(self, points, supports, origin=None):
        self._known_columns = None
        if origin is not None and len(points) and len(self.free_voxels) < 500000:
            rays = np.asarray(points)[::16]-np.asarray(origin)
            fractions = np.linspace(0., .97, 32)
            free = np.asarray(origin)+rays[:, None, :]*fractions[None, :, None]
            self.free_voxels.update(map(tuple, np.unique(np.floor(free.reshape(-1, 3)/.2).astype(int), axis=0)))
        for point in supports:
            key = (*np.floor(point[:2]/self.resolution).astype(int), round(float(point[2])/.02))
            if len(self.surfaces) < 30000 or key in self.surfaces:
                self.surfaces[key] = float(point[2])
        # Bounded observed occupancy cache; keeping old hits is conservative for
        # static reactor structures. Dynamic clearing is a later extension.
        if len(points):
            keys, indices = np.unique(np.floor(points/.12).astype(int), axis=0, return_index=True)
            for key, index in zip(keys, indices):
                if len(self.hits) < 150000 or tuple(key) in self.hits:
                    self.hits[tuple(key)] = points[index].copy()
        self.version += 1

    def build_graph(self):
        candidates = {key: np.array([(key[0]+.5)*self.resolution, (key[1]+.5)*self.resolution, z])
                      for key, z in self.surfaces.items()}
        columns = {}
        for key in candidates:
            columns.setdefault(key[:2], []).append(key)
        self._known_columns = set(columns)
        hit_points = np.asarray(list(self.hits.values())).reshape(-1, 3)
        tree = cKDTree(hit_points[:, :2]) if len(hit_points) else None
        candidate_keys = list(candidates)
        nearby = tree.query_ball_point(np.asarray(list(candidates.values()))[:, :2], self.radius) if tree is not None and candidates else None
        nodes = {}
        footprint = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                     if math.hypot(dx, dy)*self.resolution <= self.radius]
        for index, key in enumerate(candidate_keys):
            point = candidates[key]
            # Require sensed support beneath the conservative local footprint.
            if not all(any(abs(candidates[other][2]-point[2]) <= .15
                           for other in columns.get((key[0]+dx, key[1]+dy), [])) for dx, dy in footprint):
                continue
            if self.max_local_relief is not None:
                # Coarse XY cells may contain both floor and a low ledge. Do not
                # route through the lower hypothesis underneath that ledge.
                nearby_heights = [candidates[other][2] for dx, dy in footprint
                                  for other in columns.get((key[0]+dx, key[1]+dy), [])
                                  if abs(candidates[other][2]-point[2]) <= .2]
                if nearby_heights and max(nearby_heights)-min(nearby_heights) > self.max_local_relief+1e-6:
                    continue
            if tree is not None:
                near = nearby[index]
                relative = hit_points[near, 2]-point[2]
                if np.any((relative > .12) & (relative < .7)):
                    continue
            nodes[key] = point
        graph = {key: [] for key in nodes}
        for key, point in nodes.items():
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                for other in columns.get((key[0]+dx, key[1]+dy), []):
                    if other in nodes and abs(nodes[other][2]-point[2]) <= self.max_step:
                        graph[key].append(other)
        self.nodes, self.graph = nodes, graph
        self._edge_lengths = {(key, other): math.dist(nodes[key], nodes[other])
                              for key, neighbours in graph.items() for other in neighbours}
        return nodes, graph

    def nearest(self, position):
        if not self.nodes:
            return None
        position = np.asarray(position)
        eligible = [(float(np.linalg.norm(point-position)), key) for key, point in self.nodes.items()
                    if abs(point[2]-position[2]) < .18]
        if not eligible:
            return None
        distance, key = min(eligible)
        return key if distance < .75 else None

    def routes(self, start, rates=None, speed=.1, rate_scale=50.):
        costs, dose, length, parent = {start: 0.}, {start: 0.}, {start: 0.}, {start: None}
        queue = [(0., start)]
        while queue:
            cost, key = heapq.heappop(queue)
            if cost > costs[key]+1e-10:
                continue
            for other in self.graph.get(key, []):
                distance = self._edge_lengths.get((key, other))
                if distance is None:
                    distance = math.dist(self.nodes[other], self.nodes[key])
                rate = 0. if rates is None else max(rates[key], rates[other])
                edge_speed = min(speed, .06) if abs(self.nodes[other][2]-self.nodes[key][2]) > .025 else speed
                exposure = rate*distance/edge_speed/3600.
                candidate = cost+distance/edge_speed*(1.+rate/rate_scale)
                if candidate < costs.get(other, math.inf):
                    costs[other], dose[other], length[other], parent[other] = candidate, dose[key]+exposure, length[key]+distance, key
                    heapq.heappush(queue, (candidate, other))
        return costs, dose, length, parent

    @staticmethod
    def path(parent, goal):
        result = [goal]
        while parent[result[-1]] is not None:
            result.append(parent[result[-1]])
        return list(reversed(result))

    def novelty(self, key):
        # Nearby unsensed support columns, not unseen simulator geometry.
        if self._known_columns is None:
            self._known_columns = {item[:2] for item in self.surfaces}
        known = self._known_columns
        return sum((key[0]+dx, key[1]+dy) not in known
                   for dx in range(-5, 6) for dy in range(-5, 6))
