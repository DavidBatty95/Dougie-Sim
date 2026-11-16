"""Evidence-qualified hotspot exclusion using only onboard radiation beliefs."""
import math

import numpy as np


class RadiationExclusion:
    def __init__(self, rate_limit=100., confidence=.95, minimum_observed_s=.25):
        if not math.isfinite(rate_limit) or rate_limit <= 0:
            raise ValueError('frontier radiation threshold must be positive and finite')
        if not math.isfinite(confidence) or not .5 < confidence < 1.:
            raise ValueError('frontier confidence must be between 0.5 and 1')
        self.rate_limit, self.confidence = rate_limit, confidence
        self.minimum_observed_s = minimum_observed_s
        # Cantelli's one-sided bound uses log moments, so it also accommodates
        # overlapping-expert mixtures without treating them as one Gaussian.
        self.multiplier = math.sqrt(confidence/(1.-confidence))
        self.cache = {}

    def refresh(self, gp, utility):
        self.gp, self.utility = gp, utility
        self.cache.clear()

    def evaluate(self, points):
        points = np.asarray(points,dtype=float).reshape(-1,3)
        keys = [tuple(p) for p in points]
        missing = list(dict.fromkeys(k for k in keys if k not in self.cache))
        if missing:
            mu, variance = self.gp.predict_log(np.asarray(missing))
            latent = np.asarray(mu)-self.multiplier*np.sqrt(np.maximum(0.,variance))
            lower = np.exp(np.clip(latent,-30.,30.))
            if getattr(self.gp,'likelihood','poisson') != 'poisson':
                lower = np.maximum(0.,lower-1.)
            for key, rate in zip(missing,lower):
                observed = self.utility.neighbourhood_seconds(key)
                self.cache[key] = (float(rate),observed >= self.minimum_observed_s)
        lower = np.asarray([self.cache[k][0] for k in keys])
        evidence = np.asarray([self.cache[k][1] for k in keys],dtype=bool)
        return lower, evidence&(lower > self.rate_limit)

    def allows_route(self, points):
        from .path_motion import sample_polyline
        queries = sample_polyline(points,.1)+[0.,0.,.44]
        return not self.evaluate(queries)[1].any()

    def metrics(self):
        return dict(rate_limit_usv_h=self.rate_limit,confidence=self.confidence,
            method='one-sided Cantelli lower bound on GP log-rate moments',
            minimum_neighbourhood_observed_s=self.minimum_observed_s,
            scope='Model-conditional pointwise confidence, not an empirically calibrated mission-wide accuracy guarantee')
