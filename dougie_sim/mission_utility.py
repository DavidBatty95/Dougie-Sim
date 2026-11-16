"""Observed information proxies per normalized exposure/time; no evaluator truth."""
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import poisson


class ExposureUtility:
    def __init__(self, capacity=8192):
        self.exposure = {}
        self.capacity = capacity
        self._visit_tree = None
        self._visit_seconds = None

    @staticmethod
    def cell(point):
        return tuple(np.floor(np.asarray(point)/[.5, .5, .25]).astype(int))

    def observe(self, sample):
        if not sample.position_valid:
            return
        self._visit_tree = None
        points = sample.window_positions or (sample.position,)
        durations = sample.window_durations_s or (sample.integration_s,)
        for point, duration in zip(points, durations):
            key = self.cell(point)
            if key in self.exposure or len(self.exposure) < self.capacity:
                self.exposure[key] = self.exposure.get(key, 0.)+duration

    def neighbourhood_seconds(self,point,radius=.75):
        if not self.exposure:
            return 0.
        if self._visit_tree is None:
            centres = (np.asarray(list(self.exposure),dtype=float)+.5)*[.5,.5,.25]
            self._visit_tree = cKDTree(centres)
            self._visit_seconds = np.asarray(list(self.exposure.values()),dtype=float)
        indices = self._visit_tree.query_ball_point(point,radius)
        if not indices:
            return 0.
        distance = np.linalg.norm(self._visit_tree.data[indices]-np.asarray(point),axis=1)
        weight = np.maximum(0.,1.-(distance/radius)**2)**2
        return float(weight@self._visit_seconds[indices])

    def score(self, point, novelty, fresh_fraction, mean_rate, sigma, outbound_s, return_s,
              outbound_dose, return_dose, goal_upper, current_upper, wait_s, recovery_s,
              remaining_time, remaining_budget, counts_per_usv=50000., target_bonus=0., connector_bonus=0.):
        # Local expected-count/Fisher proxy, not exact Poisson mutual information.
        expected_counts = max(0., mean_rate)*.5*counts_per_usv/3600.
        information = .5*math.log1p(max(0., sigma)**2*expected_counts)
        observed_seconds = self.exposure.get(self.cell(point), 0.)
        repeat_weight = 1./(1.+observed_seconds/(1.+information))
        useful = fresh_fraction*(1.+.8*novelty+connector_bonus)+information*repeat_weight+target_bonus
        duration = outbound_s+return_s+5.+2.*wait_s+recovery_s
        dose = outbound_dose+return_dose+(current_upper*wait_s+goal_upper*(5.+wait_s+recovery_s))/3600.
        cost = .05+duration/max(remaining_time, 1e-6)+2.*dose/max(remaining_budget, 1e-6)
        result = dict(information_proxy=information, geometric_gain=novelty, revisit_weight=repeat_weight,
            observed_seconds=observed_seconds, upper_round_trip_dose_usv=dose,
            round_trip_duration_s=duration, normalized_cost=cost, score=useful/cost)
        return result

    def frontier_score(self, point, novelty, fresh_fraction, outbound_s, outbound_dose,
                       goal_rate, reference_rate, wait_s, connector_bonus=0.,dose_weight=2.):
        """Useful new views per travel/exposure cost, with no dose-budget scale.

        The same geometric/revisit score applies to blind and aware workers.
        Only the latter receives nonzero radiation inputs. A 60-s reference is
        a unit normalization, never a mission deadline or dose allowance.
        """
        observed_seconds = self.neighbourhood_seconds(point)
        repeat_weight = 1./(1.+observed_seconds/3.)
        useful = repeat_weight*(.1+fresh_fraction*(1.+min(connector_bonus, 8.))+.8*novelty)
        duration = max(0., outbound_s)+wait_s+3.
        exposure = max(0., outbound_dose)+max(0., goal_rate)*(wait_s+3.)/3600.
        dose_reference = max(.05, reference_rate)*60./3600.
        cost = .1+duration/60.+dose_weight*exposure/dose_reference
        return dict(score=useful/cost, geometric_gain=novelty, useful_gain=useful,
            observed_seconds=observed_seconds, revisit_weight=repeat_weight,
            outbound_and_dwell_dose_usv=exposure, duration_s=duration,
            dose_reference_usv=dose_reference, normalized_cost=cost,dose_weight=dose_weight)


class CountCalibration:
    """Pre-assimilation count-bound diagnostics; never lowers planning bounds."""
    def __init__(self):
        self.windows = self.exceedances = 0
        self.deviance_sum = 0.
        self.worst_ratio = 0.

    def observe(self, gp, sample):
        if not sample.position_valid:
            return
        points = sample.window_positions or (sample.position,)
        durations = np.asarray(sample.window_durations_s or (sample.integration_s,))
        mean, _, upper = gp.predict(points)
        exposure = durations*sample.counts_per_usv/3600.
        expected = max(1e-12, float(np.asarray(mean)@exposure))
        upper_intensity = max(expected, float(np.asarray(upper)@exposure))
        count_bound = float(poisson.ppf(.99, upper_intensity))
        self.windows += 1
        self.exceedances += int(sample.counts > count_bound)
        self.worst_ratio = max(self.worst_ratio, (sample.counts+1.)/(count_bound+1.))
        count = sample.counts
        self.deviance_sum += 2.*(count*math.log(max(count, 1.)/expected)-count+expected)

    def metrics(self):
        return dict(windows=self.windows, upper_count_exceedances=self.exceedances,
            upper_count_coverage=1.-self.exceedances/self.windows if self.windows else None,
            mean_poisson_deviance_at_predicted_mean=self.deviance_sum/self.windows if self.windows else None,
            worst_observed_to_bound_ratio=self.worst_ratio,
            scope='pre-assimilation count diagnostics; upper latent-rate envelope plus Poisson 99% quantile, not a calibrated joint predictive distribution')
