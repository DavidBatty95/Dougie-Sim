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
        background = .2
        for index in range(model.nnumeric):
            if model.numeric(index).name == 'radiation_background_usv_h':
                background = float(model.numeric_data[model.numeric_adr[index]])
        for index in range(model.nsite):
            name = model.site(index).name
            if name.startswith('radiation_source_'):
                strength, radius = model.site_user[index, :2]
                kind = 'leak' if model.nuser_site >= 3 and model.site_user[index, 2] == 1 else 'hotspot'
                sources.append(dict(id=name.removeprefix('radiation_source_'), kind=kind, position=data.site_xpos[index].tolist(),
                    strength_usv_h_m2=float(strength), core_radius_m=float(radius)))
        if model.nuser_geom:
            for index in range(model.ngeom):
                attenuation = float(model.geom_user[index, 0])
                if attenuation > 0:
                    import mujoco
                    if (model.geom_type[index] != mujoco.mjtGeom.mjGEOM_BOX or
                            not np.allclose(np.abs(data.geom_xmat[index].reshape(3, 3)), np.eye(3), atol=1e-8)):
                        raise ValueError('radiation attenuation currently requires axis-aligned boxes')
                    centre, half = data.geom_xpos[index], model.geom_size[index]
                    shields.append(dict(lower=(centre-half).tolist(), upper=(centre+half).tolist(), attenuation_per_m=attenuation))
        return cls(dict(background_usv_h=background, sources=sources, shields=shields))

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
        # The field is static for a mission. Compile its source/shield pairs once;
        # the 200 Hz detector path must not rebuild arrays for every intersection.
        self._origins = np.asarray([s['position'] for s in self.sources], dtype=float).reshape(-1, 3)
        self._strengths = np.asarray([s['strength_usv_h_m2'] for s in self.sources])
        self._cores2 = np.asarray([s['core_radius_m']**2 for s in self.sources])
        lower = np.asarray([s['lower'] for s in self.shields], dtype=float).reshape(-1, 3)
        upper = np.asarray([s['upper'] for s in self.shields], dtype=float).reshape(-1, 3)
        self._lower_delta = lower[None, :, :]-self._origins[:, None, :]
        self._upper_delta = upper[None, :, :]-self._origins[:, None, :]
        self._attenuation = np.asarray([s['attenuation_per_m'] for s in self.shields])

    def rate(self, point):
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('invalid detector position')
        delta = point-self._origins
        distance2 = np.einsum('ij,ij->i', delta, delta)
        contributions = self._strengths/(distance2+self._cores2)
        if len(self._attenuation):
            parallel = np.abs(delta[:, None, :]) < 1e-12
            divisor = np.where(parallel, 1., delta[:, None, :])
            a, b = self._lower_delta/divisor, self._upper_delta/divisor
            entry = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=2)
            exit = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=2)
            outside = np.any(parallel & ((self._lower_delta > 0) | (self._upper_delta < 0)), axis=2)
            fraction = np.maximum(0., np.minimum(1., exit)-np.maximum(0., entry))
            fraction[outside] = 0.
            depth = (fraction @ self._attenuation)*np.sqrt(distance2)
            contributions *= np.exp(-depth)
        return float(self.background+contributions.sum())


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
    window_positions: tuple = ()
    window_durations_s: tuple = ()
    map_revision: int = 0
    position_valid: bool = True


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
        self.window_position_sum = np.zeros((8, 3))
        self.window_duration = np.zeros(8)
        self.window_time_sum = np.zeros(8)
        self.map_revision = 0
        self.window_position_valid = True
        if not math.isfinite(counts_per_usv) or counts_per_usv <= 0 or not math.isfinite(period) or period <= 0:
            raise ValueError('invalid detector calibration')

    def integrate(self, point, rate, dt, stamp, *, position_valid=True):
        if not all(math.isfinite(v) for v in (rate, dt, stamp)) or rate < 0 or dt <= 0:
            raise ValueError('invalid exposure interval')
        self.expected_counts += rate*dt/3600.*self.sensitivity
        self.window_position_valid &= bool(position_valid)
        self.position_sum += np.asarray(point)*dt
        slot = min(7, int(self.elapsed/self.period*8))
        self.window_position_sum[slot] += np.asarray(point)*dt
        self.window_duration[slot] += dt
        self.window_time_sum[slot] += (stamp-dt/2.)*dt
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
        used = self.window_duration > 0
        positions = self.window_position_sum[used]/self.window_duration[used, None]
        sample = RadiationSample(stamp, tuple(self.position_sum/self.elapsed), counts,
                                 self.elapsed, counts*scale, variance, self.dose, upper, self.sensitivity,
                                 tuple(map(tuple, positions)), tuple(self.window_duration[used]), self.map_revision, self.window_position_valid)
        self.expected_counts, self.elapsed = 0., 0.
        self.position_sum[:] = 0.
        self.window_position_sum[:] = 0.
        self.window_duration[:] = 0.
        self.window_time_sum[:] = 0.
        self.window_position_valid = True
        return sample

    def correct_map(self, correction):
        if correction.revision <= self.map_revision:
            return
        if correction.revision != self.map_revision+1:
            raise ValueError('detector correction revisions must be contiguous')
        for i in np.flatnonzero(self.window_duration):
            duration = self.window_duration[i]
            point = self.window_position_sum[i]/duration
            self.window_position_sum[i] = correction.point(point, self.window_time_sum[i]/duration)*duration
        self.position_sum[:] = self.window_position_sum.sum(axis=0)
        self.map_revision = correction.revision
