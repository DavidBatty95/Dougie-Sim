"""Operator-only evidence filtering. This never modifies planning or GP data."""
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import chi2

CLOUD_ALPHA = .018
CLOUD_RADII = np.array([.24, .24, .16])
CLOUD_ALPHA_LEVELS = 8


def fade_near_robot(points, colors, robot_position):
    """Display-only smooth clearance around the rendered robot, not a safe zone."""
    faded = np.asarray(colors).copy()
    if robot_position is None or not len(points):
        return faded
    distance = np.linalg.norm((np.asarray(points)-np.asarray(robot_position))/[.75, .75, .65], axis=1)
    blend = np.clip((distance-1.)/.7, 0., 1.)
    faded[:, 3] *= blend*blend*(3.-2.*blend)
    return faded


def opacity_groups(colors):
    """RViz list markers need shared alpha; quantize downward into stable batches."""
    levels = np.clip(np.floor(np.asarray(colors)[:, 3]/CLOUD_ALPHA*CLOUD_ALPHA_LEVELS+1e-9),
                     0, CLOUD_ALPHA_LEVELS).astype(int)
    return [(CLOUD_ALPHA*level/CLOUD_ALPHA_LEVELS, np.flatnonzero(levels == level))
            for level in range(1, CLOUD_ALPHA_LEVELS+1)]


class RadiationHighlights:
    def __init__(self, background=.2, confidence=.99, resolution=.4, capacity=4096):
        if (not math.isfinite(background) or background < 0 or not 0 < confidence < 1
                or not math.isfinite(resolution) or resolution <= 0 or capacity < 1):
            raise ValueError('invalid radiation display calibration')
        self.background, self.confidence = float(background), float(confidence)
        self.resolution, self.capacity = resolution, capacity
        self.cells = {}
        self.last_stamp = -math.inf
        self._tree = None
        self._confirmed = None

    def observe(self, sample):
        if sample.stamp_s <= self.last_stamp:
            return  # Dashboard redraws must not assimilate the same sample twice.
        self.last_stamp = sample.stamp_s
        key = tuple(np.floor(np.asarray(sample.position)/self.resolution).astype(int))
        if key not in self.cells and len(self.cells) >= self.capacity:
            return
        counts, exposure, weighted, duration = self.cells.get(key, (0, 0., np.zeros(3), 0.))
        counts += sample.counts
        exposure += sample.integration_s*sample.counts_per_usv/3600.
        duration += sample.integration_s
        weighted = weighted+np.asarray(sample.position)*sample.integration_s
        self.cells[key] = counts, exposure, weighted, duration
        self._tree = self._confirmed = None

    def confirmed(self):
        if self._confirmed is None:
            values = list(self.cells.values())
            if not values:
                self._confirmed = np.empty((0, 3)), np.empty(0)
                return self._confirmed
            counts = np.asarray([v[0] for v in values])
            exposure = np.asarray([v[1] for v in values])
            duration = np.asarray([v[3] for v in values])
            positions = np.asarray([v[2] for v in values])/duration[:, None]
            valid = (counts > 0) & (exposure > 0)
            lower = np.zeros(len(values))
            # Vectorized exact one-sided Poisson limits. Repeated looks/multiple
            # cells are not a simultaneous confidence guarantee.
            lower[valid] = .5*chi2.ppf(1.-self.confidence, 2*counts[valid])/exposure[valid]
            selected = lower > self.background
            self._confirmed = positions[selected], counts[selected]/exposure[selected]
        return self._confirmed

    def mask(self, xyz, mean, mode='detected'):
        xyz, mean = np.asarray(xyz), np.asarray(mean)
        if mode == 'all':
            return np.ones(len(xyz), dtype=bool)
        confirmed, _ = self.confirmed()
        if not len(confirmed):
            return np.zeros(len(xyz), dtype=bool)
        # Do not paint unmeasured distant rooms or other floors from the GP prior.
        scale = np.array([.8, .8, .45])
        if self._tree is None:
            self._tree = cKDTree(confirmed/scale)
        distance = self._tree.query(xyz/scale)[0]
        return (mean > self.background) & (distance <= 1.)

    def colors(self, rates, alpha=.85):
        excess = np.maximum(0., np.asarray(rates)-self.background)
        value = np.clip(np.log1p(excess/max(self.background, .05))/np.log(101.), 0., 1.)
        return np.column_stack((np.ones(len(value)), .85*(1.-value)+.08,
                                np.full(len(value), .04), np.full(len(value), alpha)))

    def cloud(self, volume, mode='detected', layer='mean', capacity=1200):
        """Stable, density-independent local interpolation for display only.

        World-aligned cells avoid duplicated overlapping blobs and swimming as
        query positions change. Interpolation is limited in XYZ, not across floors.
        """
        xyz, mean, std, _ = volume
        selected = self.mask(xyz, mean, mode)
        xyz, mean, std = np.asarray(xyz)[selected], np.asarray(mean)[selected], np.asarray(std)[selected]
        if not len(xyz):
            return np.empty((0, 3)), np.empty((0, 4))
        xyz, unique = np.unique(xyz, axis=0, return_index=True)
        mean, std = mean[unique], std[unique]
        spacing = np.array([.18, .18, .12])
        # Fill the diagonal cells too: a seven-point cross leaves visible lobes
        # even after interpolation. A shared lattice still deduplicates overlap.
        offsets = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1)
                            for z in (-1, 0, 1)])*spacing
        cells = np.unique(np.rint((xyz[:, None, :]+offsets)/spacing).reshape(-1, 3).astype(int), axis=0)
        points = cells*spacing
        scale = np.array([.48, .48, .26])
        distances, indices = cKDTree(xyz/scale).query(points/scale, k=min(8, len(xyz)))
        distances, indices = distances.reshape(len(points), -1), indices.reshape(len(points), -1)
        weights = np.exp(-4.*distances**2)*(distances <= 1.)
        total = weights.sum(axis=1)
        supported = total > 0
        edge_alpha = np.clip(1.-distances.min(axis=1)**2, 0., 1.)**2
        points, weights, indices, total = points[supported], weights[supported], indices[supported], total[supported]
        edge_alpha = edge_alpha[supported]
        weights /= total[:, None]
        # Interpolate log rates to reduce bright discontinuities between samples.
        latent = np.log1p(mean[indices])
        blended = np.sum(weights*latent, axis=1)
        rates = np.expm1(blended)
        sigma = np.sqrt(np.sum(weights*(std[indices]**2+(latent-blended[:, None])**2), axis=1))
        keep = self.mask(points, rates, mode)
        points, rates, sigma = points[keep], rates[keep], sigma[keep]
        edge_alpha = edge_alpha[keep]
        if len(points) > capacity:
            indices = np.linspace(0, len(points)-1, capacity).astype(int)
            points, rates, sigma = points[indices], rates[indices], sigma[indices]
            edge_alpha = edge_alpha[indices]
        colors = self.colors(rates, CLOUD_ALPHA)
        colors[:, 3] *= edge_alpha
        if layer == 'uncertainty':
            value = np.clip(sigma/2., 0., 1.)
            colors[:, :3] = np.column_stack((value, 1.-value, np.ones(len(value))))
        return points, colors
