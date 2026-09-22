"""Bounded planar scan registration. No simulator, map, or command inputs."""
import math

import numpy as np


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s), (s, c)))


def voxel_points(points, size=0.12):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[indices]


class ScanOdometry:
    """Trimmed scan-to-keyframe ICP with measured IMU heading increments.

    Spatial hashing bounds correspondence work rather than allocating N x M
    distance matrices. Rejected registrations never update the reference/pose.
    """
    def __init__(self):
        self.reference = None
        self.xy = np.zeros(2)
        self.yaw = 0.0
        self.initial_heading = None
        self.reference_xy = np.zeros(2)
        self.cell_size = 0.20
        self.offsets = np.array([(x, y) for x in range(-2, 3) for y in range(-2, 3)])
        self.quality = {}

    @staticmethod
    def _keys(cells):
        return cells[..., 0] * 1000003 + cells[..., 1]

    def _set_reference(self, points):
        cells = np.floor(points / self.cell_size).astype(np.int64)
        keys, indices = np.unique(self._keys(cells), return_index=True)
        self.keys, self.reference = keys, points[indices]
        self.reference_xy = self.xy.copy()

    def update(self, points, heading):
        points = voxel_points(points)
        if len(points) < 60 or not math.isfinite(heading):
            self.quality = {"accepted": False, "reason": "insufficient_returns"}
            return False
        if self.initial_heading is None:
            self.initial_heading = heading
        yaw = math.atan2(math.sin(heading - self.initial_heading), math.cos(heading - self.initial_heading))
        rotated = points @ rotation(yaw).T
        if self.reference is None:
            self._set_reference(rotated)
            self.quality = {"accepted": True, "inliers": len(points), "rmse": 0.0}
            return True
        estimate = self.xy.copy()
        for _ in range(8):
            transformed = rotated + estimate
            cells = np.floor(transformed / self.cell_size).astype(np.int64)
            keys = self._keys(cells[:, None, :] + self.offsets)
            indices = np.searchsorted(self.keys, keys)
            indices = np.minimum(indices, len(self.keys) - 1)
            valid = self.keys[indices] == keys
            residual = self.reference[indices] - transformed[:, None, :]
            distances = np.sum(residual ** 2, axis=2)
            distances[~valid] = np.inf
            nearest = np.argmin(distances, axis=1)
            best = distances[np.arange(len(points)), nearest]
            good = best < 0.35 ** 2
            if good.sum() < 50:
                self.quality = {"accepted": False, "reason": "poor_overlap"}
                return False
            # Moving bodies and changing occlusion edges must not pull ego motion.
            good &= best <= max(0.025 ** 2, np.quantile(best[good], 0.75))
            corrections = residual[np.arange(len(points)), nearest][good]
            delta = np.mean(corrections, axis=0)
            estimate += delta
            if np.linalg.norm(delta) < 0.001:
                break
        rmse = float(np.sqrt(np.mean(best[good])))
        accepted = rmse < 0.18 and np.linalg.norm(estimate - self.xy) < 0.30
        self.quality = {"accepted": bool(accepted), "inliers": int(good.sum()), "rmse": rmse}
        if not accepted:
            return False
        self.xy, self.yaw = estimate, yaw
        if np.linalg.norm(self.xy - self.reference_xy) > 0.45:
            self._set_reference(rotated + self.xy)
        return True
