"""Bounded, count-only near-field exposure warning for route costing.

This augments planning costs; it never edits the GP map or accesses true sources.
A stationary GP tends to revert to its prior ahead of a newly observed peak.
Recent significant increases provide a short-range directional warning there.
"""
from collections import deque
import math

import numpy as np


class ExposureTrend:
    def __init__(self):
        self.history = deque(maxlen=8)
        self.active = False
        self.gradient = np.zeros(3)
        self.origin = np.zeros(3)
        self.rate = self.anchor_lower = 0.
        self.last_stamp = -math.inf

    def observe(self, sample):
        if isinstance(sample,dict):
            value = sample
        else:
            value = {name:getattr(sample,name) for name in ('position_valid','stamp_s','position','counts',
                'integration_s','counts_per_usv','rate_usv_h','variance_usv_h2')}
        if value['stamp_s'] <= self.last_stamp:
            return
        self.last_stamp = float(value['stamp_s'])
        self.active = False
        self.anchor_lower = 0.
        if not value['position_valid']:
            return
        self.origin = np.asarray(value['position'],dtype=float)
        self.anchor_lower = max(0.,value['rate_usv_h']-3.*math.sqrt(value['variance_usv_h2']))
        self.history = deque([s for s in self.history if value['stamp_s']-s['stamp_s'] <= 5.],maxlen=8)
        self.history.append(value)
        if len(self.history) < 4:
            return
        samples = list(self.history)
        split = len(samples)//2
        def intensity(group):
            counts = sum(s['counts'] for s in group)
            exposure = sum(s['integration_s']*s['counts_per_usv']/3600. for s in group)
            return counts/exposure, (counts+1.)/exposure**2
        early, early_var = intensity(samples[:split])
        late, late_var = intensity(samples[split:])
        if late < 1.5*max(early,.05) or late-early <= 3.*math.sqrt(early_var+late_var):
            return
        points = np.asarray([s['position'] for s in samples],dtype=float)
        displacement = points[split:].mean(axis=0)-points[:split].mean(axis=0)
        if np.linalg.norm(displacement) < .1:
            return
        centred = points-points.mean(axis=0)
        _,singular,axes = np.linalg.svd(centred,full_matrices=False)
        basis = axes[singular > .05].T
        if not basis.size:
            return
        exposure = np.array([s['integration_s']*s['counts_per_usv']/3600. for s in samples])
        counts = np.array([s['counts'] for s in samples])
        log_rate = np.log((counts+.5)/exposure)
        weights = np.sqrt(counts+.5)
        design = np.column_stack((np.ones(len(points)),centred@basis))
        coefficients = np.linalg.lstsq(design*weights[:,None],log_rate*weights,rcond=None)[0]
        gradient = basis@coefficients[1:]
        if gradient@displacement < math.log(1.3):
            return
        self.gradient = gradient*min(1.,6./max(np.linalg.norm(gradient),1e-12))
        self.rate = max(value['rate_usv_h'],late)
        self.active = True

    def adjust(self, points, mean):
        points, mean = np.asarray(points,dtype=float).reshape(-1,3), np.asarray(mean,dtype=float)
        distance = np.linalg.norm(points-self.origin,axis=1)
        # Charge initial turning/dwell at the actual recent measurement location
        # even when the smooth GP has not yet represented a sharp local peak.
        result = np.maximum(mean,self.anchor_lower*np.clip(1.-distance/.25,0.,1.)**2)
        if not self.active:
            return result
        projection = (points-self.origin)@self.gradient
        taper = np.clip((1.25-distance)/.5,0.,1.)
        warning = self.rate*np.exp(np.clip(projection,0.,3.))
        use = (projection > .05)&(distance < 1.25)
        return np.maximum(result,np.where(use,mean+taper*np.maximum(0.,warning-mean),mean))

    def metrics(self):
        return dict(active=self.active,anchor_lower_usv_h=self.anchor_lower,
            gradient_log_rate_per_m=self.gradient.tolist() if self.active else None,
            position=self.origin.tolist(),stamp_s=self.last_stamp if math.isfinite(self.last_stamp) else None,
            scope='count-only near-field planning guard; not a GP posterior or a safety guarantee')


class ExposureCostView:
    def __init__(self,gp,trend):
        self.gp,self.trend = gp,trend

    def predict(self,points):
        mean,std,upper = self.gp.predict(points)
        cost = self.trend.adjust(points,mean)
        return cost,std,np.maximum(upper,cost)
