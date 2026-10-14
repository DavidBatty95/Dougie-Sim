"""Bounded overlapping XYZ GP experts, with conservative planning envelopes.

Observations are assimilated once per relevant expert. Overlapping experts are
correlated: prediction uses mixture moments, NEVER a product of precisions.
Spatial cells are a compute partition, not known rooms, shielding or floor IDs.
"""
import itertools
import math
import time

import numpy as np

from .gp import OnlineGP, PoissonOnlineGP

_HALO_MASKS = np.asarray(list(itertools.product((0, 1), repeat=3)), dtype=np.int64)


class SpatialGP:
    def __init__(self, capacity=32, max_experts=128, cell_size=(4., 4., 1.5), halo=.2, likelihood='poisson'):
        self.capacity, self.max_experts = int(capacity), int(max_experts)
        self.cell_size = np.asarray(cell_size, dtype=float)
        self.halo, self.likelihood = float(halo), likelihood
        if (self.capacity < 1 or self.max_experts < 1 or self.cell_size.shape != (3,)
                or not np.isfinite(self.cell_size).all() or np.any(self.cell_size <= 0)
                or not math.isfinite(self.halo) or not 0 < self.halo < .5
                or likelihood not in ('poisson', 'gaussian')):
            raise ValueError('invalid spatial GP configuration')
        self.experts = {}
        self.prototype = self._new_expert()
        self.prior, self.variance = self.prototype.prior, self.prototype.variance
        self.lengthscales = self.prototype.lengthscales.copy()
        self.updates = 0
        self.update_ms = []
        self.saturated = False
        self.unallocated_updates = 0

    def _new_expert(self):
        expert = (PoissonOnlineGP if self.likelihood == 'poisson' else OnlineGP)(self.capacity)
        if hasattr(self, 'prior'):
            expert.prior, expert.variance = self.prior, self.variance
            expert.lengthscales = self.lengthscales.copy()
        return expert

    def assignments(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if not np.isfinite(points).all():
            raise ValueError('nonfinite spatial GP coordinates')
        scaled = points/self.cell_size
        if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 9e18):
            raise ValueError('spatial GP coordinates exceed index range')
        owner = np.floor(scaled).astype(np.int64)
        fraction = scaled-owner
        primary = np.minimum(1., .5+.5*np.minimum(fraction, 1.-fraction)/self.halo)
        direction = np.where(fraction < .5, -1, 1)
        masks = _HALO_MASKS
        keys = owner[:, None, :]+masks[None, :, :]*direction[:, None, :]
        weights = np.prod(np.where(masks[None, :, :], 1.-primary[:, None, :], primary[:, None, :]), axis=2)
        return keys, weights

    def _update(self, point, method, values):
        started = time.perf_counter()
        point = np.asarray(point, dtype=float)
        if point.shape != (3,):
            raise ValueError('measurement position must be XYZ')
        keys, weights = self.assignments(point[None])
        # Owner first, then the strongest overlapping neighbours. Never evict old
        # return-route beliefs to make room silently; report capacity exhaustion.
        for index in np.argsort(-weights[0], kind='stable'):
            if weights[0, index] <= 0.:
                continue
            key = tuple(map(int, keys[0, index]))
            if key not in self.experts:
                if len(self.experts) >= self.max_experts:
                    self.saturated = True
                    self.unallocated_updates += 1
                    continue
                self.experts[key] = self._new_expert()
            getattr(self.experts[key], method)(point, *values)
        self.updates += 1
        self.update_ms.append((time.perf_counter()-started)*1000.)
        if len(self.update_ms) > 4096:
            self.update_ms = self.update_ms[-2048:]

    def update_counts(self, point, counts, integration_s, counts_per_usv=50000.):
        if (self.likelihood != 'poisson' or not all(math.isfinite(v) for v in (counts, integration_s, counts_per_usv))
                or counts < 0 or counts != int(counts) or integration_s <= 0 or counts_per_usv <= 0):
            raise ValueError('invalid count measurement')
        self._update(point, 'update_counts', (counts, integration_s, counts_per_usv))

    def update(self, point, rate, observation_variance):
        if (self.likelihood != 'gaussian' or not all(math.isfinite(v) for v in (rate, observation_variance))
                or rate < 0 or observation_variance < 0):
            raise ValueError('invalid Gaussian measurement')
        self._update(point, 'update', (rate, observation_variance))

    def _rate(self, latent):
        return (np.exp(np.clip(latent, -30., 30.)) if self.likelihood == 'poisson' else
                np.maximum(0., np.expm1(np.clip(latent, -30., 30.))))

    def _predict(self, points, beta):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        keys, weights = self.assignments(points)
        mu, second, mean, envelope = (np.zeros(len(points)) for _ in range(4))
        rows, slots = np.nonzero(weights > 0.)
        if not len(rows):
            return mu, second, mean, envelope
        assigned = keys[rows, slots]
        order = np.lexsort((assigned[:, 2], assigned[:, 1], assigned[:, 0]))
        ordered = assigned[order]
        boundaries = np.r_[0, np.flatnonzero(np.any(np.diff(ordered, axis=0), axis=1))+1, len(order)]
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            key_array = ordered[start]
            assignment = order[start:end]
            indices, w = rows[assignment], weights[rows[assignment], slots[assignment]]
            expert = self.experts.get(tuple(map(int, key_array)))
            if expert is None:
                m, v = np.full(len(indices), self.prior), np.full(len(indices), self.variance)
            else:
                m, v = expert.predict_log(points[indices])
            rate_mean = self._rate(m+.5*v)
            upper = np.maximum(rate_mean, self._rate(m+beta*np.sqrt(v)))
            np.add.at(mu, indices, w*m)
            np.add.at(second, indices, w*(v+m*m))
            np.add.at(mean, indices, w*rate_mean)
            np.maximum.at(envelope, indices, upper)
        variance = np.maximum(1e-8, second-mu*mu)
        # Disagreement inflates uncertainty. A hotspot seen by one active expert
        # is not averaged away in the planning bound. Missing experts keep priors.
        envelope = np.maximum.reduce((envelope, mean, self._rate(mu+beta*np.sqrt(variance))))
        return mu, variance, mean, envelope

    def predict_log(self, points):
        mu, variance, _, _ = self._predict(points, 2.)
        return mu, variance

    def predict(self, points, beta=2.):
        if not math.isfinite(beta) or beta < 0:
            raise ValueError('invalid prediction confidence multiplier')
        _, variance, mean, upper = self._predict(points, beta)
        return mean, np.sqrt(variance), upper

    def metrics(self):
        return dict(model='overlapping_spatial_experts', likelihood=self.likelihood,
            experts=len(self.experts), max_experts=self.max_experts, capacity_per_expert=self.capacity,
            capacity=self.capacity*self.max_experts,
            inducing_points=sum(len(e.Z) for e in self.experts.values()),
            inducing_capacity_total=self.capacity*self.max_experts, updates=self.updates,
            expert_updates=sum(e.updates for e in self.experts.values()),
            posterior_bytes=sum(e.Z.nbytes+e.L.nbytes+e.mean.nbytes+e.cov.nbytes for e in self.experts.values()),
            prediction_cache_bytes=sum(e.prediction_cache_bytes for e in self.experts.values()),
            cell_size_m=self.cell_size.tolist(), halo_fraction=self.halo, saturated=self.saturated,
            unallocated_updates=self.unallocated_updates,
            update_p50_ms=float(np.median(self.update_ms)) if self.update_ms else None,
            update_p95_ms=float(np.percentile(self.update_ms, 95)) if self.update_ms else None,
            kernel='Matern32-ARD', coordinates='xyz metres',
            uncertainty='mixture variance plus expert disagreement; maximum active upper envelope')

    def save(self, path):
        keys = sorted(self.experts)
        data = dict(model_kind='spatial', schema_version=1, likelihood=self.likelihood,
                    capacity=self.capacity, max_experts=self.max_experts, cell_size=self.cell_size,
                    halo=self.halo, updates=self.updates, saturated=self.saturated,
                    lengthscales=self.lengthscales, variance=self.variance, prior_log_rate=self.prior,
                    unallocated_updates=self.unallocated_updates, expert_keys=np.asarray(keys, dtype=np.int64).reshape(-1, 3))
        for i, key in enumerate(keys):
            expert = self.experts[key]
            for name, value in dict(Z=expert.Z, mean=expert.mean, covariance=expert.cov,
                                    cholesky=expert.L, updates=expert.updates).items():
                data[f'expert_{i}_{name}'] = value
        np.savez_compressed(path, **data)


def load_gp(path):
    """Load global (including legacy) or spatial archives without pickle."""
    with np.load(path, allow_pickle=False) as data:
        likelihood = str(data['likelihood'].item())
        kind = str(data.get('model_kind', 'global'))
        if likelihood not in ('poisson', 'gaussian') or kind not in ('global', 'spatial'):
            raise ValueError('unsupported GP archive')
        if int(data.get('schema_version', 1)) != 1:
            raise ValueError('unsupported GP archive version')
        if kind == 'spatial':
            gp = SpatialGP(int(data['capacity']), int(data['max_experts']), data['cell_size'], float(data['halo']), likelihood)
            gp.lengthscales = np.asarray(data.get('lengthscales', [1.5, 1.5, .7])).copy()
            gp.variance = float(data.get('variance', 2.25))
            gp.prior = float(data.get('prior_log_rate', math.log(2.)))
            gp.prototype = gp._new_expert()
            gp.updates, gp.saturated = int(data['updates']), bool(data['saturated'])
            gp.unallocated_updates = int(data['unallocated_updates'])
            for i, key in enumerate(data['expert_keys']):
                expert = gp._new_expert()
                for attribute, name in (('Z', 'Z'), ('mean', 'mean'), ('cov', 'covariance'), ('L', 'cholesky')):
                    setattr(expert, attribute, data[f'expert_{i}_{name}'].copy())
                expert.updates = int(data[f'expert_{i}_updates'])
                gp.experts[tuple(map(int, key))] = expert
        else:
            gp = (PoissonOnlineGP if likelihood == 'poisson' else OnlineGP)(int(data.get('capacity', max(1, len(data['Z'])))),
                lengthscales=data['lengthscales'], variance=float(data['variance']))
            gp.Z, gp.mean, gp.cov, gp.L = (data[name].copy() for name in ('Z', 'mean', 'covariance', 'cholesky'))
            gp.prior = float(data['prior_log_rate'])
            gp.updates = int(data.get('updates', 0))
    return gp
