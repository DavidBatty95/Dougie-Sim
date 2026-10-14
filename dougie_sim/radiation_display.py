"""Operator-only evidence filtering. This never modifies planning or GP data."""
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import chi2


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
