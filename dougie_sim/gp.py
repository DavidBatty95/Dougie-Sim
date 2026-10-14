"""Bounded inducing-point, streaming Gaussian-process radiation regression.

Maintains q(u)=N(m,S), assimilating each heteroscedastic log-rate measurement
once. Inducing inputs are admitted by conditional prior variance. Between
admissions this is an O(M^2) Gaussian update; prediction includes the conditional
prior residual so unsupported locations do not inherit false certainty.
Fixed Matérn-3/2 ARD kernel: this is NOT POAM's learned attentive kernel/EM.
"""
import math
import time

import numpy as np
from scipy.linalg import cho_solve, solve_triangular


class OnlineGP:
    likelihood = 'gaussian'
    def __init__(self, capacity=128, lengthscales=(1.5, 1.5, .7), variance=2.25, prior_rate=1.):
        self.capacity = int(capacity)
        self.lengthscales = np.asarray(lengthscales, dtype=float)
        self.variance = float(variance)
        self.prior = math.log1p(prior_rate)
        if (self.capacity < 1 or self.lengthscales.shape != (3,) or not np.isfinite(self.lengthscales).all()
                or np.any(self.lengthscales <= 0) or not math.isfinite(variance) or variance <= 0):
            raise ValueError('invalid GP configuration')
        self.Z = np.empty((0, 3))
        self.L = np.empty((0, 0))
        self.mean = np.empty(0)
        self.cov = np.empty((0, 0))
        self.updates = 0
        self.update_ms = []
        self.use_prepared_predictions = True
        self._prediction_cache = None
        self.prediction_cache_builds = 0
        self.prediction_chunk_size = 2048

    def kernel(self, a, b):
        d = np.linalg.norm((np.asarray(a)[:, None, :]-np.asarray(b)[None, :, :])/self.lengthscales, axis=2)
        distance = math.sqrt(3.)*d
        return self.variance*(1.+distance)*np.exp(-distance)

    def update(self, point, rate, observation_variance):
        start = time.perf_counter()
        point = np.asarray(point, dtype=float).reshape(1, 3)
        if not np.isfinite(point).all() or not all(math.isfinite(v) for v in (rate, observation_variance)) or rate < 0 or observation_variance < 0:
            raise ValueError('invalid GP observation')
        a, residual = self._project(point)
        target = math.log1p(rate)-self.prior
        noise = max(.01, observation_variance/(1.+rate)**2)
        cross = self.cov@a
        denominator = float(a@cross)+residual+noise
        self.mean += cross*((target-float(a@self.mean))/denominator)
        self.cov -= np.outer(cross, cross)/denominator
        self._finish(start)

    def _project(self, point):
        n = len(self.Z)
        if n:
            k = self.kernel(self.Z, point)[:, 0]
            v = solve_triangular(self.L, k, lower=True, check_finite=False)
            a = solve_triangular(self.L.T, v, lower=False, check_finite=False)
            residual = max(1e-8, self.variance-float(v@v))
        else:
            a, v, residual = np.empty(0), np.empty(0), self.variance
        if n < self.capacity and residual > .08*self.variance:
            L = np.zeros((n+1, n+1))
            L[:n, :n], L[n, :n], L[n, n] = self.L, v, math.sqrt(residual+1e-8)
            mean = np.append(self.mean, float(a@self.mean))
            cov = np.zeros((n+1, n+1))
            cov[:n, :n] = self.cov
            cross = self.cov@a
            cov[:n, n], cov[n, :n] = cross, cross
            cov[n, n] = residual+float(a@cross)
            self.Z, self.L, self.mean, self.cov = np.vstack((self.Z, point)), L, mean, cov
            a = np.zeros(n+1)
            a[-1] = 1.
            residual = 0.
        return a, residual

    def _finish(self, start):
        self.invalidate_prediction_cache()
        self.cov = (self.cov+self.cov.T)*.5
        self.updates += 1
        self.update_ms.append((time.perf_counter()-start)*1000.)
        if len(self.update_ms) > 4096:
            self.update_ms = self.update_ms[-2048:]

    def invalidate_prediction_cache(self):
        """Required after manual posterior edits; normal updates do this automatically."""
        self._prediction_cache = None

    @property
    def prediction_cache_bytes(self):
        return sum(a.nbytes for a in self._prediction_cache) if self._prediction_cache is not None else 0

    def _prepare_prediction(self):
        if self._prediction_cache is None:
            inverse = cho_solve((self.L, True), np.eye(len(self.Z)), check_finite=False)
            alpha = inverse@self.mean
            correction = inverse@self.cov@inverse-inverse
            correction = (correction+correction.T)*.5
            self._prediction_cache = alpha, correction
            self.prediction_cache_builds += 1
        return self._prediction_cache

    def predict_log(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if not np.isfinite(points).all():
            raise ValueError('nonfinite query')
        if not len(self.Z):
            return np.full(len(points), self.prior), np.full(len(points), self.variance)
        if self.use_prepared_predictions:
            alpha, correction = self._prepare_prediction()
            mu, var = np.empty(len(points)), np.empty(len(points))
            # Bound temporary M x N arrays independently of facility/query size.
            for start in range(0, len(points), self.prediction_chunk_size):
                end = start+self.prediction_chunk_size
                K = self.kernel(self.Z, points[start:end])
                mu[start:end] = self.prior+K.T@alpha
                var[start:end] = self.variance+np.sum(K*(correction@K), axis=0)
            return mu, np.maximum(var, 1e-8)
        # Kept as an independent reference path for equivalence/performance checks.
        K = self.kernel(self.Z, points)
        A = cho_solve((self.L, True), K, check_finite=False)
        mu = self.prior+A.T@self.mean
        var = self.variance-np.sum(K*A, axis=0)+np.sum(A*(self.cov@A), axis=0)
        return mu, np.maximum(var, 1e-8)

    def predict(self, points, beta=2.):
        mu, var = self.predict_log(points)
        # Lognormal mean and an upper latent-rate credible bound, not a
        # calibrated simultaneous safety guarantee over the whole trajectory.
        mean = np.maximum(0., np.expm1(np.clip(mu+.5*var, -30, 30)))
        upper = np.maximum(mean, np.expm1(np.clip(mu+beta*np.sqrt(var), -30, 30)))
        return mean, np.sqrt(var), upper

    def metrics(self):
        return dict(inducing_points=len(self.Z), capacity=self.capacity, updates=self.updates,
            posterior_bytes=self.Z.nbytes+self.L.nbytes+self.mean.nbytes+self.cov.nbytes,
            prediction_cache_bytes=self.prediction_cache_bytes, prediction_cache_builds=self.prediction_cache_builds,
            update_p50_ms=float(np.median(self.update_ms)) if self.update_ms else None,
            update_p95_ms=float(np.percentile(self.update_ms, 95)) if self.update_ms else None,
            kernel='Matern32-ARD', coordinates='xyz metres', target='log1p(rate_usv_h)')

    def save(self, path):
        np.savez_compressed(path, model_kind='global', schema_version=1,
            Z=self.Z, mean=self.mean, covariance=self.cov, cholesky=self.L,
            lengthscales=self.lengthscales, variance=self.variance, prior_log_rate=self.prior,
            likelihood=self.likelihood, capacity=self.capacity, updates=self.updates)


class PoissonOnlineGP(OnlineGP):
    """Sparse log-Gaussian Cox regression using streaming Laplace updates.

    Count ~ Poisson(exposure * exp(f)); a scalar Laplace approximation of the
    new latent marginal is projected into q(u). Each count batch is used once.
    This keeps the physical likelihood while bounding online inference cost.
    """
    likelihood = 'poisson'
    def __init__(self, capacity=128, lengthscales=(1.5, 1.5, .7), variance=2.25, prior_rate=2.):
        if not math.isfinite(prior_rate) or prior_rate <= 0:
            raise ValueError('Poisson rate prior must be positive')
        super().__init__(capacity, lengthscales, variance, prior_rate)
        self.prior = math.log(prior_rate)

    def update_counts(self, point, counts, integration_s, counts_per_usv=50000.):
        start = time.perf_counter()
        point = np.asarray(point, dtype=float).reshape(1, 3)
        if (not np.isfinite(point).all() or not all(math.isfinite(v) for v in (counts, integration_s, counts_per_usv))
                or counts < 0 or counts != int(counts) or integration_s <= 0 or counts_per_usv <= 0):
            raise ValueError('invalid count measurement')
        a, residual = self._project(point)
        cross = self.cov@a
        variance = max(1e-10, float(a@cross)+residual)
        mu = self.prior+float(a@self.mean)
        exposure = integration_s*counts_per_usv/3600.
        mode = mu
        for _ in range(25):
            expected = exposure*math.exp(np.clip(mode, -30., 30.))
            gradient = (mode-mu)/variance+expected-counts
            curvature = 1./variance+expected
            delta = float(np.clip(gradient/curvature, -2., 2.))
            mode -= delta
            if abs(delta) < 1e-8:
                break
        posterior_variance = 1./(1./variance+exposure*math.exp(np.clip(mode, -30., 30.)))
        self.mean += cross*((mode-mu)/variance)
        self.cov += np.outer(cross, cross)*((posterior_variance-variance)/variance**2)
        self._finish(start)

    def predict(self, points, beta=2.):
        mu, var = self.predict_log(points)
        mean = np.exp(np.clip(mu+.5*var, -30., 30.))
        upper = np.maximum(mean, np.exp(np.clip(mu+beta*np.sqrt(var), -30., 30.)))
        return mean, np.sqrt(var), upper

    def metrics(self):
        result = super().metrics()
        result.update(target='log(rate_usv_h)', likelihood='Poisson counts', inference='streaming sparse Laplace projection')
        return result
