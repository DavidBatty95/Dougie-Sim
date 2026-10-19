"""Positive-definite observed-surface feature kernel with bounded exact replay.

Geometry only changes statistical correlation; a wall does not imply a known
attenuation coefficient or a safe dose behind it. New features rebuild the
posterior from retained onboard count windows, never reuse an incompatible basis.
"""
import math
import time

import numpy as np

from .gp import PoissonOnlineGP


class GeometryPoissonGP(PoissonOnlineGP):
    def __init__(self, capacity=128, max_planes=16, replay_capacity=8192, strength=1.5):
        super().__init__(capacity)
        self.max_planes, self.replay_capacity = max_planes, replay_capacity
        self.strength = strength
        self.planes = []  # observed centre XYZ, unit normal XYZ, local radius
        self.records = []
        self.last_geometry_stamp = -math.inf
        self.geometry_rebuilds = 0
        self.rebuild_ms = 0.
        self.saturated = False

    def features(self, points):
        points = np.asarray(points).reshape(-1, 3)
        if not self.planes:
            return np.empty((len(points), 0))
        planes = np.asarray(self.planes)
        delta = points[:, None, :]-planes[None, :, :3]
        signed = np.sum(delta*planes[None, :, 3:6], axis=2)
        tangential2 = np.maximum(0., np.sum(delta*delta, axis=2)-signed*signed)
        return self.strength*np.tanh(signed/.12)*np.exp(-.5*tangential2/planes[None, :, 6]**2)

    def kernel(self, a, b):
        base = super().kernel(a, b)
        if not self.planes:
            return base
        fa, fb = self.features(a), self.features(b)
        # Product of two PSD kernels; arbitrary segment-obstruction multipliers
        # need not be PSD and would invalidate posterior covariance arithmetic.
        squared = np.maximum(0., np.sum(fa*fa, axis=1)[:, None]+np.sum(fb*fb, axis=1)[None, :]-2.*fa@fb.T)
        return base*np.exp(-.5*squared)

    def observe_geometry(self, packet):
        if packet.stamp_s-self.last_geometry_stamp < 10. or len(self.planes) >= self.max_planes:
            return
        self.last_geometry_stamp = packet.stamp_s
        points = packet.origin+packet.directions*packet.distances[..., None]
        a = np.roll(points, 1, axis=1)-np.roll(points, -1, axis=1)
        b = np.roll(points, 1, axis=0)-np.roll(points, -1, axis=0)
        n = np.cross(a, b)
        norm = np.linalg.norm(n, axis=2)
        valid = packet.valid & np.roll(packet.valid, 1, axis=0) & np.roll(packet.valid, -1, axis=0)
        valid &= np.roll(packet.valid, 1, axis=1) & np.roll(packet.valid, -1, axis=1)
        valid &= (norm > 1e-5) & (np.linalg.norm(a, axis=2) < .8) & (np.linalg.norm(b, axis=2) < .8)
        valid[[0, -1]] = False
        p, n = points[valid], n[valid]/norm[valid, None]
        if not len(p):
            return
        n *= np.where(n[np.arange(len(n)), np.argmax(np.abs(n), axis=1)] < 0., -1., 1.)[:, None]
        tags = np.column_stack((np.round(n/.2), np.round(np.sum(p*n, axis=1)/.25))).astype(int)
        _, inverse, counts = np.unique(tags, axis=0, return_inverse=True, return_counts=True)
        added = False
        for index in np.argsort(-counts)[:24]:
            if counts[index] < 40 or len(self.planes) >= self.max_planes:
                continue
            group = p[inverse == index]
            centre = np.median(group, axis=0)
            _, _, vectors = np.linalg.svd(group-centre, full_matrices=False)
            normal = vectors[-1]
            residual = np.abs((group-centre)@normal)
            if np.quantile(residual, .95) > .025:
                continue
            radius = float(np.clip(np.quantile(np.linalg.norm(group-centre, axis=1), .9), .5, 4.))
            if any(abs(normal@old[3:6]) > .98 and abs((centre-old[:3])@normal) < .15
                   and np.linalg.norm(centre-old[:3]) < old[6]*1.5 for old in self.planes):
                continue
            self.planes.append(np.r_[centre, normal, radius])
            added = True
        if added and self.records:
            started = time.perf_counter()
            PoissonOnlineGP.__init__(self, self.capacity, self.lengthscales.copy(), self.variance, math.exp(self.prior))
            for points, durations, counts, sensitivity in self.records:
                PoissonOnlineGP.update_counts_window(self, points, durations, counts, sensitivity)
            self.geometry_rebuilds += 1
            self.rebuild_ms = (time.perf_counter()-started)*1000.

    def update_counts(self, point, counts, integration_s, counts_per_usv=50000.):
        self.update_counts_window([point], [integration_s], counts, counts_per_usv)

    def update_counts_window(self, points, durations, counts, counts_per_usv=50000.):
        if len(self.records) >= self.replay_capacity:
            self.saturated = True
            return
        PoissonOnlineGP.update_counts_window(self, points, durations, counts, counts_per_usv)
        self.records.append((np.array(points, dtype=float, copy=True), np.array(durations, dtype=float, copy=True), int(counts), float(counts_per_usv)))

    def metrics(self):
        result = super().metrics()
        result.update(model='observed_geometry_gp', kernel='Matern32-ARD x RBF(observed localized plane-side features)',
            observed_planes=len(self.planes), geometry_rebuilds=self.geometry_rebuilds, last_rebuild_ms=self.rebuild_ms,
            retained_count_windows=len(self.records), replay_capacity=self.replay_capacity, saturated=self.saturated,
            information_contract='estimated onboard range surfaces and count windows; no material/source metadata')
        return result

    def save(self, path):
        lengths = [len(record[0]) for record in self.records]
        np.savez_compressed(path, model_kind='geometry', schema_version=1, likelihood='poisson',
            Z=self.Z, mean=self.mean, covariance=self.cov, cholesky=self.L,
            lengthscales=self.lengthscales, variance=self.variance, prior_log_rate=self.prior,
            capacity=self.capacity, updates=self.updates, planes=np.asarray(self.planes).reshape(-1, 7),
            strength=self.strength, max_planes=self.max_planes, replay_capacity=self.replay_capacity,
            last_geometry_stamp=self.last_geometry_stamp, geometry_rebuilds=self.geometry_rebuilds,
            saturated=self.saturated, window_offsets=np.r_[0, np.cumsum(lengths)],
            window_positions=np.concatenate([r[0] for r in self.records]) if lengths else np.empty((0, 3)),
            window_durations=np.concatenate([r[1] for r in self.records]) if lengths else np.empty(0),
            window_counts=np.asarray([r[2] for r in self.records], dtype=np.int64),
            window_sensitivity=np.asarray([r[3] for r in self.records]))
