"""Exact adaptive-radius point connectivity without rescanning visited buckets."""
import math
import ctypes
from collections import deque
import numpy as np

try:
    _library = ctypes.CDLL('libunitree_point_components.so')
    _native = _library.unitree_point_components
    _native.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                        ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_int32)]
    _native.restype = ctypes.c_int
except OSError:
    _native = None


def connected_components(points, radii, grid_size, z_distance):
    """Return index components under max(r_i, r_j) XY and optional Z gates.

    Each grid bucket contains only unvisited points. Dense walls therefore do
    not repeatedly scan thousands of already-assigned points. No point sampling
    or approximation is applied to the clustering connectivity.
    """
    if not points:
        return []
    if _native is not None:
        xyz = np.ascontiguousarray(points, dtype=np.float64)
        epsilon = np.ascontiguousarray(radii, dtype=np.float64)
        if xyz.shape != (len(points), 3) or epsilon.shape != (len(points),):
            raise ValueError('Invalid point/radius dimensions')
        labels = np.empty(len(points), dtype=np.int32)
        count = _native(xyz.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                        epsilon.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), len(points),
                        grid_size, z_distance, labels.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)))
        if count < 0:
            raise ValueError('Invalid cloud or native clustering failed')
        components = [[] for _ in range(count)]
        for index, label in enumerate(labels):
            components[label].append(index)
        return components
    grid_size = max(grid_size, max(radii), 1e-3)
    keys = [(math.floor(x/grid_size), math.floor(y/grid_size)) for x, y, _ in points]
    grid = {}
    for index, key in enumerate(keys):
        grid.setdefault(key, set()).add(index)
    remaining = set(range(len(points)))
    components = []
    while remaining:
        start = remaining.pop()
        grid[keys[start]].remove(start)
        queue, component = deque([start]), [start]
        while queue:
            index = queue.popleft()
            x, y, z = points[index]
            cx, cy = keys[index]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    bucket = grid.get((cx+dx, cy+dy))
                    if not bucket:
                        continue
                    matched = []
                    for other in bucket:
                        ox, oy, oz = points[other]
                        radius = max(radii[index], radii[other])
                        if ((z_distance <= 0 or abs(oz-z) <= z_distance) and
                                (ox-x)**2 + (oy-y)**2 <= radius*radius):
                            matched.append(other)
                    bucket.difference_update(matched)
                    remaining.difference_update(matched)
                    queue.extend(matched)
                    component.extend(matched)
        components.append(component)
    return components
