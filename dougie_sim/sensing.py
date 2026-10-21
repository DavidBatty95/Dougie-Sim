"""Detached range observations; no object identities or hidden scene metadata."""
from dataclasses import dataclass

import numpy as np


def correct_box_plane_rays(model, data, origin, directions, distances, geom_ids):
    """Correct MuJoCo 3.3.6 multiRay's non-conservative angular box bounds.

    Corner-only elevation bounds can miss the middle of a broad ramp/slab.
    Infinite planes also must not be distance-culled by their origin. These exact
    primitive intersections stay inside the simulator sensor, not the mapper.
    """
    import mujoco
    rays = directions.reshape(-1, 3)
    result = distances.reshape(-1)
    for index in geom_ids:
        rotation = data.geom_xmat[index].reshape(3, 3)
        local = (origin-data.geom_xpos[index])@rotation
        vectors = rays@rotation
        size = model.geom_size[index]
        if model.geom_type[index] == mujoco.mjtGeom.mjGEOM_PLANE:
            hit = np.full(len(rays), np.inf)
            np.divide(-local[2], vectors[:, 2], out=hit, where=vectors[:, 2] < -1e-12)
            for axis in (0, 1):
                if size[axis] > 0:
                    hit[np.abs(local[axis]+vectors[:, axis]*hit) > size[axis]] = np.inf
        else:
            parallel = np.abs(vectors) < 1e-12
            divisor = np.where(parallel, 1., vectors)
            a, b = (-size-local)/divisor, (size-local)/divisor
            entry = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=1)
            exit = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=1)
            hit = np.where(entry >= 0, entry, exit)
            hit[(exit < np.maximum(entry, 0.)) | np.any(parallel & (np.abs(local) > size), axis=1)] = np.inf
        nearer = np.isfinite(hit) & (hit >= 0) & ((result < 0) | (hit < result))
        result[nearer] = hit[nearer]


@dataclass(frozen=True)
class RangePacket:
    stamp_s: float
    sequence: int
    origin: np.ndarray
    directions: np.ndarray
    distances: np.ndarray
    max_range_m: float
    support_mask: np.ndarray
    map_revision: int = 0

    def __post_init__(self):
        # Packets crossing a worker boundary own immutable arrays.
        for name in ('origin', 'directions', 'distances', 'support_mask'):
            value = np.array(getattr(self, name), copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def valid(self):
        return (self.distances > .05) & (self.distances <= self.max_range_m)

    @property
    def points(self):
        return (self.origin+self.directions*self.distances[..., None])[self.valid]

    @property
    def supports(self):
        return (self.origin+self.directions*self.distances[..., None])[self.support_mask]


def support_mask(origin, directions, distances, max_range):
    """Orient observed normals toward the sensor: underside != foothold."""
    valid = (distances > .05) & (distances <= max_range)
    points = origin+directions*distances[..., None]
    across = np.roll(points, -1, axis=1)-np.roll(points, 1, axis=1)
    radial = np.roll(points, -1, axis=0)-np.roll(points, 1, axis=0)
    normals = np.cross(across, radial)
    facing = np.sum(normals*(origin-points), axis=2)
    normals *= np.where(facing >= 0., 1., -1.)[..., None]
    lengths = np.linalg.norm(normals, axis=2)
    supported = normals[..., 2]/np.maximum(lengths, 1e-12) > .92
    supported &= valid & np.roll(valid, 1, axis=0) & np.roll(valid, -1, axis=0)
    supported &= np.roll(valid, 1, axis=1) & np.roll(valid, -1, axis=1)
    supported &= np.linalg.norm(radial, axis=2) < .65
    supported[[0, -1], :] = False
    return supported
