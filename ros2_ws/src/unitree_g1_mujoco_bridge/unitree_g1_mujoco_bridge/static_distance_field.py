"""Bounded, conservative static-clearance queries without SciPy dependencies."""

import math
import numpy as np


def observed_free_mask(points, pose, ranges, angle_min, angle_increment):
    """Clear remembered returns only when three measured rays see beyond them."""
    points = np.asarray(points)
    if not len(points) or len(ranges) < 3:
        return np.zeros(len(points), dtype=bool)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    local = (points - pose[:2]) @ np.asarray(((c, -s), (s, c)))
    distance = np.linalg.norm(local, axis=1)
    indexes = np.rint((np.arctan2(local[:, 1], local[:, 0]) - angle_min) / angle_increment).astype(int)
    valid = (indexes > 0) & (indexes < len(ranges) - 1)
    bins = np.clip(indexes[:, None] + (-1, 0, 1), 0, len(ranges) - 1)
    measured = np.asarray(ranges)[bins]
    return valid & np.all(np.isfinite(measured) & (measured > distance[:, None] + 0.15), axis=1)


class StaticDistanceField:
    def __init__(self, points, origin, extent, influence, resolution=0.10):
        self.resolution = resolution
        self.origin = np.asarray(origin, dtype=float) - extent
        self.width = math.ceil(2.0 * extent / resolution) + 1
        steps = math.ceil(influence / resolution) + 2
        field = np.full((self.width, self.width), steps + 1, dtype=np.int16)
        cells = np.floor((np.asarray(points) - self.origin) / resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < self.width), axis=1)
        cells = cells[valid]
        field[cells[:, 1], cells[:, 0]] = 0
        for _ in range(steps):
            padded = np.pad(field, 1, constant_values=steps + 1)
            nearby = np.minimum.reduce([padded[y:y + self.width, x:x + self.width]
                                       for y in range(3) for x in range(3)])
            field = np.minimum(field, nearby + 1)
        # Chebyshev distance is a lower bound on Euclidean distance; subtract
        # one cell for unknown sub-cell locations. Never overstate clearance.
        self.distances = np.maximum(0.0, (field.astype(float) - 1.0) * resolution)

    def query(self, points):
        cells = np.floor((np.asarray(points) - self.origin) / self.resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < self.width), axis=1)
        result = np.zeros(len(cells))  # Outside the checked window is not free.
        result[valid] = self.distances[cells[valid, 1], cells[valid, 0]]
        return result
