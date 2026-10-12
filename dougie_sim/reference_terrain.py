"""Frozen pre-optimization kernels for matched-workload regression benchmarks.

These reproduce the original SurfaceMap geometry/cost semantics from 7837f41.
They are not used by the robot. Common observation storage is inherited.
"""
import heapq
import math

import numpy as np
from scipy.spatial import cKDTree

from .terrain import SurfaceMap


class ReferenceSurfaceMap(SurfaceMap):
    def build_graph(self):
        candidates = {key: np.array([(key[0]+.5)*self.resolution, (key[1]+.5)*self.resolution, z])
                      for key, z in self.surfaces.items()}
        columns = {}
        for key in candidates:
            columns.setdefault(key[:2], []).append(key)
        hit_points = np.asarray(list(self.hits.values())).reshape(-1, 3)
        tree = cKDTree(hit_points[:, :2]) if len(hit_points) else None
        nodes = {}
        footprint = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                     if math.hypot(dx, dy)*self.resolution <= self.radius]
        for key, point in candidates.items():
            if not all(any(abs(candidates[other][2]-point[2]) <= .15
                           for other in columns.get((key[0]+dx, key[1]+dy), [])) for dx, dy in footprint):
                continue
            if tree is not None:
                near = tree.query_ball_point(point[:2], self.radius)
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
        return nodes, graph

    def routes(self, start, rates=None, speed=.1, rate_scale=50.):
        costs, dose, length, parent = {start: 0.}, {start: 0.}, {start: 0.}, {start: None}
        queue = [(0., start)]
        while queue:
            cost, key = heapq.heappop(queue)
            if cost > costs[key]+1e-10:
                continue
            for other in self.graph.get(key, []):
                distance = float(np.linalg.norm(self.nodes[other]-self.nodes[key]))
                rate = 0. if rates is None else max(rates[key], rates[other])
                edge_speed = min(speed, .06) if abs(self.nodes[other][2]-self.nodes[key][2]) > .025 else speed
                exposure = rate*distance/edge_speed/3600.
                candidate = cost+distance/edge_speed*(1.+rate/rate_scale)
                if candidate < costs.get(other, math.inf):
                    costs[other], dose[other], length[other], parent[other] = candidate, dose[key]+exposure, length[key]+distance, key
                    heapq.heappush(queue, (candidate, other))
        return costs, dose, length, parent

    def novelty(self, key):
        known = {item[:2] for item in self.surfaces}
        return sum((key[0]+dx, key[1]+dy) not in known for dx in range(-5, 6) for dy in range(-5, 6))
