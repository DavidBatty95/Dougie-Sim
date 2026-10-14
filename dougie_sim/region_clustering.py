"""Bounded candidate abstraction on observed terrain and radiation beliefs.

Inspired by POMCGS's decision compression, not its offline POMDP solver.
K-means labels are split by graph connectivity, height, hazard and uncertainty
classes. Representatives are real supported nodes; centroids never become goals.
"""
from collections import deque
import math
import heapq
import warnings

import numpy as np
from scipy.cluster.vq import kmeans2


def distance_from_visits(nodes, graph, visited):
    """Geodesic novelty: a nearby point across a wall is not already visited."""
    distance = {key: 0. for key in visited if key in nodes}
    queue = [(0., key) for key in distance]
    heapq.heapify(queue)
    while queue:
        cost, key = heapq.heappop(queue)
        if cost > distance[key]+1e-10:
            continue
        for other in graph.get(key, []):
            if other not in nodes:
                continue
            candidate = cost+math.dist(nodes[key], nodes[other])
            if candidate < distance.get(other, math.inf):
                distance[other] = candidate
                heapq.heappush(queue, (candidate, other))
    return {key: distance.get(key, math.inf) for key in nodes}


def connected_clusters(nodes, graph, upper, uncertainty, rate_limit, max_clusters=24):
    keys = sorted(nodes)
    if not keys:
        return []
    points = np.asarray([nodes[key] for key in keys])
    risks = np.asarray([upper[key] for key in keys])
    sigmas = np.asarray([uncertainty[key] for key in keys])
    if not np.isfinite(points).all() or not np.isfinite(risks).all() or not np.isfinite(sigmas).all() or np.any(risks < 0):
        raise ValueError('invalid clustering inputs')
    count = min(max_clusters, max(1, math.ceil(len(keys)/80)))
    features = np.column_stack((points/np.array([1.5, 1.5, .25]), np.log1p(risks), sigmas))
    if count == 1:
        labels = np.zeros(len(keys), dtype=int)
    else:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)  # Empty centres are harmless; connectivity splitting still applies.
            _, labels = kmeans2(features, count, iter=6, minit='++', seed=0)
    tags = {key: (int(label), int(round(float(point[2])/.02)//8),
                  bool(risk > rate_limit), int(math.floor(math.log1p(risk)/math.log(2))),
                  int(math.floor(sigma/.5)))
            for key, point, risk, sigma, label in zip(keys, points, risks, sigmas, labels)}
    remaining = set(keys)
    clusters = []
    for seed in keys:
        if seed not in remaining:
            continue
        remaining.remove(seed)
        queue = deque([seed])
        members = []
        while queue:
            key = queue.popleft()
            members.append(key)
            for other in graph.get(key, []):
                if other in remaining and tags[other] == tags[seed]:
                    remaining.remove(other)
                    queue.append(other)
        clusters.append(members)
    return clusters


class ObservedGain:
    """Short-range, occlusion-aware unknown-support proxy, not true map coverage."""
    def __init__(self, terrain):
        self.terrain = terrain
        self.hits = np.asarray(list(terrain.hits.values())).reshape(-1, 3)
        self.layers = {}
        self.cache = {}
        self.offsets = []
        for angle in np.linspace(-math.pi, math.pi, 24, endpoint=False):
            self.offsets.append([(round(distance*math.cos(angle)), round(distance*math.sin(angle))) for distance in range(1, 9)])
        self.rays = np.asarray(self.offsets, dtype=int)
        cells = np.asarray([key[:2] for key in terrain.nodes], dtype=int).reshape(-1, 2)
        self.origin = cells.min(axis=0)-9 if len(cells) else np.zeros(2, dtype=int)
        self.shape = tuple(cells.max(axis=0)-self.origin+10) if len(cells) else (1, 1)
        self.use_grid = math.prod(map(int, self.shape)) <= 1000000
        self.support_cells = np.asarray([key[:2] for key in terrain.surfaces], dtype=int).reshape(-1, 2)
        self.support_heights = np.asarray(list(terrain.surfaces.values()))

    def value(self, key):
        if key in self.cache:
            return self.cache[key]
        layer = round(float(self.terrain.nodes[key][2])/.1)
        if layer not in self.layers:
            z = layer*.1
            known_cells = self.support_cells[np.abs(self.support_heights-z) < .16]
            blocked_cells = np.empty((0, 2), dtype=int)
            if len(self.hits):
                mask = (self.hits[:, 2] > z+.12) & (self.hits[:, 2] < z+.7)
                blocked_cells = np.floor(self.hits[mask, :2]/self.terrain.resolution).astype(int)
            if self.use_grid:
                known, blocked = np.zeros(self.shape, dtype=bool), np.zeros(self.shape, dtype=bool)
                for grid, coordinates in ((known, known_cells), (blocked, blocked_cells)):
                    indices = coordinates-self.origin
                    inside = np.all((indices >= 0) & (indices < self.shape), axis=1)
                    indices = indices[inside]
                    grid[indices[:, 0], indices[:, 1]] = True
            else:
                known, blocked = set(map(tuple, known_cells)), set(map(tuple, blocked_cells))
            self.layers[layer] = known, blocked
        known, blocked = self.layers[layer]
        if self.use_grid:
            coordinates = self.rays+np.asarray(key[:2])-self.origin
            obstacle = blocked[coordinates[..., 0], coordinates[..., 1]]
            stop = obstacle | ~known[coordinates[..., 0], coordinates[..., 1]]
            first = stop.argmax(axis=1)
            visible = stop.any(axis=1) & ~obstacle[np.arange(len(first)), first]
            frontier = coordinates[np.arange(len(first)), first][visible]
            self.cache[key] = len(np.unique(frontier, axis=0))
            return self.cache[key]
        revealed = set()
        for ray in self.offsets:
            for dx, dy in ray:
                cell = key[0]+dx, key[1]+dy
                if cell in blocked:
                    break
                if cell not in known:
                    revealed.add(cell)
                    break
        self.cache[key] = len(revealed)
        return self.cache[key]
