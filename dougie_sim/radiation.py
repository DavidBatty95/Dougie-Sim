"""Simulator-only gamma dose-rate proxy and a counting detector.

Rate: micro-sieverts/hour; accumulated dose-equivalent proxy: micro-sieverts.
Inverse-square falloff with finite cores and exponential shielding, not particle
transport. Only Detector samples may be passed to the mapper/planner.
"""
from dataclasses import dataclass
import math

import numpy as np


def segment_box_length(start, end, lower, upper):
    start, end, lower, upper = (np.asarray(v, dtype=float) for v in (start, end, lower, upper))
    delta = end-start
    lo, hi = 0., 1.
    for axis in range(3):
        if abs(delta[axis]) < 1e-12:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return 0.
        else:
            a, b = sorted(((lower[axis]-start[axis])/delta[axis], (upper[axis]-start[axis])/delta[axis]))
            lo, hi = max(lo, a), min(hi, b)
    return max(0., hi-lo)*float(np.linalg.norm(delta))


class RadiationField:
    @classmethod
    def from_mujoco(cls, model, data):
        sources, shields = [], []
        for index in range(model.nsite):
            name = model.site(index).name
            if name.startswith('radiation_source_'):
                strength, radius = model.site_user[index, :2]
                sources.append(dict(position=data.site_xpos[index].tolist(),
                    strength_usv_h_m2=float(strength), core_radius_m=float(radius)))
        if model.nuser_geom:
            for index in range(model.ngeom):
                attenuation = float(model.geom_user[index, 0])
                if attenuation > 0:
                    centre, half = data.geom_xpos[index], model.geom_size[index]
                    shields.append(dict(lower=(centre-half).tolist(), upper=(centre+half).tolist(), attenuation_per_m=attenuation))
        return cls(dict(background_usv_h=.2, sources=sources, shields=shields))

    def __init__(self, description):
        self.background = float(description['background_usv_h'])
        self.sources = description['sources']
        self.shields = description.get('shields', [])
        if not math.isfinite(self.background) or self.background < 0:
            raise ValueError('invalid background')
        for source in self.sources:
            if (not np.isfinite(source['position']).all() or len(source['position']) != 3 or
                    not math.isfinite(source['strength_usv_h_m2']) or not math.isfinite(source['core_radius_m']) or
                    source['strength_usv_h_m2'] < 0 or source['core_radius_m'] <= 0):
                raise ValueError('invalid source')
        for shield in self.shields:
            if (not np.isfinite(shield['lower']).all() or not np.isfinite(shield['upper']).all()
                    or not math.isfinite(shield['attenuation_per_m']) or shield['attenuation_per_m'] < 0
                    or np.any(np.asarray(shield['upper']) <= np.asarray(shield['lower']))):
                raise ValueError('invalid shielding')

    def rate(self, point):
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('invalid detector position')
        rate = self.background
        for source in self.sources:
            origin = np.asarray(source['position'])
            distance2 = float(np.sum((point-origin)**2))
            optical_depth = sum(shield['attenuation_per_m']*segment_box_length(origin, point, shield['lower'], shield['upper'])
                                for shield in self.shields)
            rate += source['strength_usv_h_m2']/(distance2+source['core_radius_m']**2)*math.exp(-optical_depth)
        return rate


@dataclass(frozen=True)
class RadiationSample:
    stamp_s: float
    position: tuple
    counts: int
    integration_s: float
    rate_usv_h: float
    variance_usv_h2: float
    dose_usv: float
    dose_upper_usv: float
    counts_per_usv: float = 50000.


class Detector:
    def __init__(self, seed=1, counts_per_usv=50000., period=.5):
        self.rng = np.random.default_rng(seed)
        self.sensitivity = counts_per_usv
        self.period = period
        self.expected_counts = 0.
        self.elapsed = 0.
        self.position_sum = np.zeros(3)
        self.dose = 0.
        self.counts_total = 0
        self.samples = 0
        if not math.isfinite(counts_per_usv) or counts_per_usv <= 0 or not math.isfinite(period) or period <= 0:
            raise ValueError('invalid detector calibration')

    def integrate(self, point, rate, dt, stamp):
        if not all(math.isfinite(v) for v in (rate, dt, stamp)) or rate < 0 or dt <= 0:
            raise ValueError('invalid exposure interval')
        self.expected_counts += rate*dt/3600.*self.sensitivity
        self.position_sum += np.asarray(point)*dt
        self.elapsed += dt
        if self.elapsed+1e-9 < self.period:
            return None
        counts = int(self.rng.poisson(self.expected_counts))
        self.counts_total += counts
        self.samples += 1
        self.dose = self.counts_total/self.sensitivity
        scale = 3600./(self.elapsed*self.sensitivity)
        # Approximate counting uncertainty, nonzero even for zero counts.
        variance = (counts+1)*scale**2
        upper = (self.counts_total+3.*math.sqrt(self.counts_total+1)+3.)/self.sensitivity
        sample = RadiationSample(stamp, tuple(self.position_sum/self.elapsed), counts,
                                 self.elapsed, counts*scale, variance, self.dose, upper, self.sensitivity)
        self.expected_counts, self.elapsed = 0., 0.
        self.position_sum[:] = 0.
        return sample
