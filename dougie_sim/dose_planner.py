"""Dose-budgeted receding-horizon exploration over observed support surfaces.

Inspired by safe-return exploration and streaming GP informative planning;
not an implementation of SafeEstMDP or a formal probabilistic guarantee.
"""
import math
import time

import numpy as np

from .gp import OnlineGP, PoissonOnlineGP
from .terrain import SurfaceMap


class DoseExplorer:
    def __init__(self, budget=2., rate_limit=100., capacity=128, mission_duration=600., likelihood='poisson'):
        if not math.isfinite(budget) or budget <= 0 or not math.isfinite(rate_limit) or rate_limit <= 0:
            raise ValueError('dose budget and rate limit must be positive')
        self.gp = PoissonOnlineGP(capacity) if likelihood == 'poisson' else OnlineGP(capacity)
        self.terrain = SurfaceMap()
        self.budget, self.rate_limit = budget, rate_limit
        self.mission_duration = mission_duration
        self.home = None
        self.home_stamp = 0.
        self.home_tolerance = .4
        self.recovery_reserve_s = 0.
        self.sample = None
        self.path = []
        self.target = None
        self.visited = set()
        self.state = 'initial_survey'
        self.returning = False
        self.return_reserve = 0.
        self.plan_ms = []
        self.worker_ms = []
        self.display_samples = {}
        self.events = []
        self.last_progress = None
        self.progress_stamp = 0.
        self.failed = []
        self.detector_likelihood = 'window'
        self.invalid_position_windows = 0
        from .mission_utility import ExposureUtility, CountCalibration
        self.utility = ExposureUtility()
        self.calibration = CountCalibration()
        self.calibration_enabled = False

    def observe_radiation(self, sample, replay=False):
        self.sample = sample
        self.utility.observe(sample)
        if self.calibration_enabled and not replay:
            self.calibration.observe(self.gp, sample)
        if not sample.position_valid:
            self.invalid_position_windows += 1
        elif self.gp.likelihood == 'poisson':
            if self.detector_likelihood == 'window' and sample.window_positions:
                self.gp.update_counts_window(sample.window_positions, sample.window_durations_s, sample.counts, sample.counts_per_usv)
            else:
                self.gp.update_counts(sample.position, sample.counts, sample.integration_s, sample.counts_per_usv)
        else:
            self.gp.update(sample.position, sample.rate_usv_h, sample.variance_usv_h2)
        if sample.dose_upper_usv >= self.budget*.8 or sample.rate_usv_h > self.rate_limit:
            self.returning = True

    def observe_geometry(self, points, supports, origin=None):
        self.terrain.update(points, supports, origin)

    def replan(self, position, stamp):
        start_time = time.perf_counter()
        nodes, _ = self.terrain.build_graph()
        root = self.terrain.nearest(position)
        if root is None or self.sample is None or stamp-self.sample.stamp_s > 1.5:
            self.path = []
            self.state = 'waiting_for_fresh_sensors_or_support'
            return
        if self.home is None:
            self.home = np.asarray(position).copy()
            self.home_stamp = stamp
        home = self.terrain.nearest(self.home)
        if home is None:
            self.path = []
            self.state = 'no_observed_return_route'
            return
        self.visited.add(root)
        keys = list(nodes)
        locations = np.asarray([nodes[key]+[0., 0., .44] for key in keys])
        _, uncertainty, upper = self.gp.predict(locations)
        rates = dict(zip(keys, upper))
        uncertainty = dict(zip(keys, uncertainty))
        costs, dose, length, parent = self.terrain.routes(root, rates)
        _, home_dose, _, _ = self.terrain.routes(home, rates, reverse=True)
        if home not in parent:
            self.path = []
            self.state = 'no_observed_return_route'
            return
        self.return_reserve = dose[home]+self.budget*.15+rates[root]*3./3600.
        time_reserve = length[home]/.06+15.
        if self.sample.dose_upper_usv+self.return_reserve >= self.budget or stamp+time_reserve >= self.mission_duration:
            self.returning = True
        if self.returning:
            selected = home
            self.state = 'returning'
            if math.dist(position, self.home) < self.home_tolerance:
                self.path = []
                self.state = 'returned'
                return
        else:
            eligible = []
            self.failed = [(p, t) for p, t in self.failed if stamp-t < 60.]
            for key in sorted(parent, key=lambda k: length[k])[::max(1, len(parent)//160)]:
                if length[key] < .75 or length[key] > 4. or key not in home_dose:
                    continue
                if any(math.dist(nodes[key], p) < .65 for p, _ in self.failed):
                    continue
                total = self.sample.dose_upper_usv+dose[key]+home_dose[key]+self.budget*.15+rates[key]*5./3600.
                route = self.terrain.path(parent, key)
                if total > self.budget or max(rates[p] for p in route) > self.rate_limit:
                    continue
                novelty = self.terrain.novelty(key)
                visited_distance = min((math.dist(nodes[key], nodes[p]) for p in self.visited if p in nodes), default=2.)
                information = .5*math.log1p(uncertainty[key]**2/.02)
                score = (1.+novelty*.12+information+min(visited_distance, 2.))/(1.+length[key]+20.*dose[key])
                eligible.append((score, key))
            if not eligible:
                self.returning = True
                self.state = 'returning_no_admissible_frontier'
                selected = home
            else:
                selected = max(eligible)[1]
                self.state = 'exploring'
        self.target = nodes[selected].copy()
        route = self.terrain.path(parent, selected)
        self.path = [nodes[key].copy() for key in route[1:]]
        self.events.append(dict(time_s=stamp, state=self.state, target=self.target.tolist(),
            predicted_outbound_dose_usv=dose[selected], return_reserve_usv=self.return_reserve,
            measured_dose_upper_usv=self.sample.dose_upper_usv))
        self.plan_ms.append((time.perf_counter()-start_time)*1000.)

    def command(self, position, yaw, stamp):
        if self.sample is None or stamp-self.sample.stamp_s > 1.5 or self.state == 'returned':
            return np.zeros(3)
        while self.path and math.dist(position, self.path[0]) < .22:
            self.path.pop(0)
        if not self.path:
            return np.zeros(3)
        if self.last_progress is None or math.dist(position, self.last_progress) > .15:
            self.last_progress, self.progress_stamp = np.asarray(position).copy(), stamp
        elif stamp-self.progress_stamp > 15.:
            self.failed.append((self.target.copy(), stamp))
            self.path = []
            self.state = 'route_stalled'
            self.last_progress = None
            return np.zeros(3)
        target = self.path[min(1, len(self.path)-1)]
        error = math.atan2(target[1]-position[1], target[0]-position[0])-yaw
        error = math.atan2(math.sin(error), math.cos(error))
        speed = .22*max(0., math.cos(error)) if abs(error) < .7 else 0.
        # More conservative speed near a sensed change in support height.
        if abs(target[2]-position[2]) > .025:
            speed = min(speed, .16)
        return np.array([speed, 0., np.clip(1.4*error, -.5, .5)])

    def metrics(self):
        cells = getattr(self.terrain, 'observed_surface_cells', len(self.terrain.surfaces))
        return dict(state=self.state, budget_usv=self.budget, rate_limit_usv_h=self.rate_limit,
            return_reserve_usv=self.return_reserve, mapped_surface_cells=cells,
            mapped_surface_area_m2=cells*self.terrain.resolution**2,
            observed_voxels=len(self.terrain.hits), visited_cells=len(self.visited),
            observed_free_volume_m3=len(self.terrain.free_voxels)*.2**3,
            gp=self.gp.metrics(), planning_p95_ms=float(np.percentile(self.plan_ms, 95)) if self.plan_ms else None,
            worker_p95_ms=float(np.percentile(self.worker_ms, 95)) if self.worker_ms else None,
            events=self.events, map_consistency=getattr(self, 'map_consistency', {}),
            localization_margin_m=self.terrain.localization_margin, invalid_position_windows=self.invalid_position_windows,
            recovery=getattr(self, 'recovery', {}), count_calibration=self.calibration.metrics())
