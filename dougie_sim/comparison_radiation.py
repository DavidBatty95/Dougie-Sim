"""Evaluator-only truth/GP comparison. Nothing here is returned to navigation."""
import json

import numpy as np
from .gp_field import interpolate_field


class RadiationMapComparison:
    sensor_height = .44

    def __init__(self, points, field, heights):
        self.points = np.asarray(points, dtype=float)
        self.query = self.points+[0., 0., self.sensor_height]
        self.truth = np.array([field.rate(point) for point in self.query])
        self.heights = list(heights)
        boundaries = (np.asarray(heights[1:])+heights[:-1])/2
        self.layers = np.searchsorted(boundaries, self.points[:, 2], side='right')
        self.mean = np.full(len(points), np.nan)
        self.std = self.mean.copy()
        self.upper = self.mean.copy()
        self.neighbours = np.zeros(len(points),dtype=int)
        self.source_queries = 0
        self.stamp = 0.
        self.minimum = float(field.background)
        self.maximum = max(100., float(self.truth.max()))

    def update(self, volume, rotation, translation, stamp):
        self.mean[:] = self.std[:] = self.upper[:] = np.nan
        self.neighbours[:] = 0
        self.source_queries = 0
        self.stamp = float(stamp)
        if volume is None or not len(volume[0]):
            return
        xyz, mean, std, upper = volume
        self.source_queries = len(xyz)
        world = np.asarray(xyz)@rotation.T+translation
        self.mean[:],self.std[:],self.upper[:],_,self.neighbours[:] = interpolate_field(
            self.query,world,mean,std,upper)

    def payload(self, seen):
        def values(array):
            return [float(value) if np.isfinite(value) else None for value in array]
        valid = np.isfinite(self.mean)
        log_error = np.log1p(self.mean[valid])-np.log1p(self.truth[valid])
        return dict(
            points=self.points.tolist(), layers=self.layers.tolist(), heights=self.heights,
            truth_usv_h=self.truth.tolist(), gp_mean_usv_h=values(self.mean),
            gp_std_log_rate=values(self.std), gp_upper_usv_h=values(self.upper),
            observed=np.asarray(seen, dtype=bool).tolist(), time_s=self.stamp,
            color_min_usv_h=self.minimum, color_max_usv_h=self.maximum,
            predicted_cells=int(valid.sum()), reference_cells=len(self.points),
            matched_log1p_rmse=float(np.sqrt(np.mean(log_error**2))) if len(log_error) else None,
             sensor_height_m=self.sensor_height,
            interpolation='compact_wendland_shepard',interpolation_radius_m=[.35,.35,.18],
            gp_neighbours=self.neighbours.tolist(),
            exported_gp_queries=self.source_queries,
            scope='Evaluator-only true field versus continuous local interpolation of exported aware GP predictions. Compact support: 35 cm XY and 18 cm Z; uncertainty includes between-prediction disagreement, not precision multiplication. Missing predictions are null. Identical fixed floor samples at +44 cm. No evaluator data is sent to the planner.')

    def save(self, output, seen):
        temporary = output/'radiation_map.tmp'
        temporary.write_text(json.dumps(self.payload(seen), allow_nan=False)+'\n')
        temporary.replace(output/'radiation_map.json')
