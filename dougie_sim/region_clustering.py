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
        self.witnesses = {}
        self.offsets = []
        for angle in np.linspace(-math.pi, math.pi, 24, endpoint=False):
            self.offsets.append([(round(distance*math.cos(angle)), round(distance*math.sin(angle))) for distance in range(1, 9)])
        self.rays = np.asarray(self.offsets, dtype=int)
        cells = np.asarray([key[:2] for key in terrain.nodes], dtype=int).reshape(-1, 2)
        self.origin = cells.min(axis=0)-9 if len(cells) else np.zeros(2, dtype=int)
        self.shape = tuple(cells.max(axis=0)-self.origin+10) if len(cells) else (1, 1)
        self.use_grid = math.prod(map(int, self.shape)) <= 1000000
        # The layered map's retained support returns are authoritative. The
        # inherited flat-map surface cache can be both incomplete and capped.
        if hasattr(terrain, 'support_returns'):
            support = np.asarray(list(terrain.support_returns.values()), dtype=float).reshape(-1, 3)
        else:
            support = np.asarray([[(key[0]+.5)*terrain.resolution, (key[1]+.5)*terrain.resolution, z]
                                  for key, z in terrain.surfaces.items()], dtype=float).reshape(-1, 3)
        self.support_cells = np.floor(support[:, :2]/terrain.resolution+1e-8).astype(int)
        self.support_heights = support[:, 2]
        heights = np.r_[support[:, 2], self.hits[:, 2], [p[2] for p in terrain.nodes.values()]]
        self.z_resolution = .04
        self.z_origin = math.floor(float(heights.min(initial=0.))-1.)
        self.z_count = int(math.ceil((float(heights.max(initial=0.))+1.-self.z_origin)/self.z_resolution))+1
        self.use_grid = self.use_grid and math.prod(map(int,self.shape))*self.z_count <= 5000000
        self.support_columns, self.hit_columns = {}, {}
        if self.use_grid:
            shape = (*self.shape, self.z_count)
            self.known_grid = np.zeros(shape,dtype=bool)
            self.hit_min, self.hit_max = np.full(shape,np.inf), np.full(shape,-np.inf)
            for is_support, points in ((True,support),(False,self.hits)):
                xy = np.floor(points[:, :2]/terrain.resolution+1e-8).astype(int)-self.origin
                z = np.rint((points[:, 2]-self.z_origin)/self.z_resolution).astype(int)
                inside = np.all((xy >= 0)&(xy < self.shape),axis=1)&(z >= 0)&(z < self.z_count)
                xy, z = xy[inside], z[inside]
                if is_support:
                    self.known_grid[xy[:,0],xy[:,1],z] = True
                else:
                    np.minimum.at(self.hit_min,(xy[:,0],xy[:,1],z),points[inside,2])
                    np.maximum.at(self.hit_max,(xy[:,0],xy[:,1],z),points[inside,2])
        else:
            for columns, points in ((self.support_columns,support),(self.hit_columns,self.hits)):
                for point in points:
                    key = tuple(np.floor(point[:2]/terrain.resolution+1e-8).astype(int))
                    columns.setdefault(key,[]).append(float(point[2]))
        self.support_offsets = np.arange(-4,5)
        self.body_offsets = np.arange(2,19)
        # A previously mapped lower floor does not establish a drop on an
        # overlapping, still-unobserved storey. Require ray-cleared space across
        # the admissible support-height interval before treating it as an edge.
        occupancy = getattr(terrain, 'occupancy', None)
        self.free_resolution = getattr(occupancy, 'resolution', terrain.resolution)
        evidence=getattr(occupancy,'evidence',{})
        coarse_free=getattr(occupancy,'coarse_free',{})
        self.free_grid = None
        self.free_sparse = None
        if self.use_grid:
            lower = np.r_[self.origin*terrain.resolution,self.z_origin-1.]
            upper = np.r_[(self.origin+self.shape)*terrain.resolution,self.z_origin+self.z_count*self.z_resolution+1.]
            self.free_origin = np.floor(lower/self.free_resolution).astype(int)
            free_shape = np.ceil(upper/self.free_resolution).astype(int)-self.free_origin+1
            if math.prod(map(int,free_shape)) <= 5000000:
                from .occupancy_access import dense_confirmed_free
                self.free_grid=dense_confirmed_free(evidence,coarse_free,self.free_origin,free_shape)
        if self.free_grid is None:
            from .occupancy_access import SparseConfirmedFree
            self.free_sparse=SparseConfirmedFree(evidence,coarse_free)
        increments = np.diff(np.concatenate((np.zeros((24,1,2),dtype=int),self.rays),axis=1),axis=1)
        self.allowed_steps = np.linalg.norm(increments,axis=2)*terrain.resolution*getattr(terrain,'max_slope',math.tan(math.radians(10.)))+.06

    def cleared_at(self,cells,height):
        points = np.concatenate(((np.asarray(cells)+.5)*self.terrain.resolution,
            np.asarray(height)[...,None]),axis=-1)
        voxels = np.floor(points/self.free_resolution).astype(int)
        if self.free_grid is None:
            return self.free_sparse.contains(voxels)
        indices = voxels-self.free_origin
        inside = np.all((indices >= 0)&(indices < self.free_grid.shape),axis=-1)
        clipped = np.clip(indices,0,np.asarray(self.free_grid.shape)-1)
        return inside&self.free_grid[tuple(np.moveaxis(clipped,-1,0))]

    def cleared_support(self,cells,height,tolerance):
        # Merely clearing air above a floor, or below a thin rising ramp, is
        # insufficient. Check the entire permitted next-step height envelope.
        return (self.cleared_at(cells,height-tolerance-.02)
            &self.cleared_at(cells,height)
            &self.cleared_at(cells,height+tolerance+.02))

    def value(self, key):
        if key not in self.cache:
            self.values([key])
        return self.cache[key]

    def values(self, keys):
        missing = [key for key in keys if key not in self.cache]
        for start in range(0,len(missing),128):
            group = missing[start:start+128]
            if not self.use_grid:
                for key in group:
                    self.cache[key] = self._sparse_value(key)
                continue
            xy = np.asarray([key[:2] for key in group],dtype=int)
            height = np.repeat(np.asarray([self.terrain.nodes[k][2] for k in group])[:,None],24,axis=1)
            active = np.ones(height.shape,dtype=bool)
            unknown = np.zeros(height.shape,dtype=bool)
            frontier = np.zeros((*height.shape,3),dtype=int)
            for step in range(8):
                cells = xy[:,None,:]+self.rays[None,:,step,:]
                indices = cells-self.origin
                centre = np.rint((height-self.z_origin)/self.z_resolution).astype(int)
                levels = np.clip(centre[...,None]+self.support_offsets,0,self.z_count-1)
                candidates = self.z_origin+levels*self.z_resolution
                observed = self.known_grid[indices[...,0,None],indices[...,1,None],levels]
                difference = np.abs(candidates-height[...,None])
                eligible = observed&(difference <= self.allowed_steps[None,:,step,None])
                choice = np.argmin(np.where(eligible,difference,np.inf),axis=2)
                supported = eligible.any(axis=2)
                next_height = np.take_along_axis(candidates,choice[...,None],axis=2)[...,0]
                next_height = np.where(supported,next_height,height)
                body = np.clip(np.rint((next_height-self.z_origin)/self.z_resolution).astype(int)[...,None]+self.body_offsets,0,self.z_count-1)
                low = self.hit_min[indices[...,0,None],indices[...,1,None],body]
                high = self.hit_max[indices[...,0,None],indices[...,1,None],body]
                blocked = ((high > next_height[...,None]+.12)&(low < next_height[...,None]+.7)).any(axis=2)
                known_drop = ~supported&self.cleared_support(cells,height,self.allowed_steps[None,:,step])
                hit_unknown = active&~blocked&~supported&~known_drop
                unknown |= hit_unknown
                frontier[hit_unknown] = np.concatenate((cells,centre[...,None]),axis=2)[hit_unknown]
                active &= supported&~blocked
                height = np.where(active,next_height,height)
            for i,key in enumerate(group):
                cells = np.unique(frontier[i,unknown[i]],axis=0)
                self.cache[key] = len(cells)
                self.witnesses[key] = frozenset((int(x),int(y),round((self.z_origin+z*self.z_resolution)/.04)) for x,y,z in cells)
        return np.asarray([self.cache[key] for key in keys],dtype=int)

    def _sparse_value(self,key):
        revealed = set()
        for index,ray in enumerate(self.offsets):
            height = float(self.terrain.nodes[key][2])
            for step,(dx,dy) in enumerate(ray):
                cell = key[0]+dx,key[1]+dy
                support = self.support_columns.get(cell,[])
                eligible = [z for z in support if abs(z-height) <= self.allowed_steps[index,step]]
                new_height = min(eligible,key=lambda z:abs(z-height)) if eligible else height
                if any(.12 < z-new_height < .7 for z in self.hit_columns.get(cell,[])):
                    break
                if not eligible:
                    known_drop = bool(self.cleared_support(np.asarray(cell),np.asarray(height),self.allowed_steps[index,step]))
                    if not known_drop:
                        revealed.add((*cell,round(height/self.z_resolution)))
                    break
                height = new_height
        self.witnesses[key] = frozenset(revealed)
        return len(revealed)

    def unknown_cells(self,key):
        self.value(key)
        return self.witnesses[key]
