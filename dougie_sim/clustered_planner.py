"""Connected, risk-separated region candidates with persistent revalidated goals.

Research links and adaptation boundaries: docs/DOUGIE_RESEARCH.md.
Goal selection and dose admission use the fine observed graph, never a centroid
shortcut. Optional execution smoothing is separately checked against observed
support, clearance, connectivity and predicted upper-rate exposure.
"""
import math
from collections import Counter, deque
import time

import numpy as np

from .dose_planner import DoseExplorer
from .region_clustering import connected_clusters, distance_from_visits, ObservedGain
from .route_forecast import forecast_tree, forecast_polyline
from .motion_cost import FORECAST_SPEED, FORECAST_YAW_RATE
from .region_agenda import RegionAgenda


class ClusteredDoseExplorer(DoseExplorer):
    def __init__(self, *args, commitment=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.commitment = commitment
        self.goal_started = 0.
        self.goal_switches = 0
        self.clusters = []
        self.return_path = []
        self.candidate_count = 0
        self.rejections = {}
        self.last_reason = 'initial_survey'
        self.recent_goals = []
        self.cluster_stamp = 0.
        self.representatives_scored = 0
        self.blocked_edges = {}
        self.sampled_height_bands = set()
        self.landmarks = {}
        self.agenda = {}
        self.agenda_capacity = 2048
        self.agenda_overflow = 0
        self.route_forecast = {}
        self.plan_delays = []
        self.wait_allowance = 3.
        self.objective = 'survey'
        self.smooth_execution = False
        self.region_agenda = RegionAgenda()
        self.region_scheduling = 'hierarchical'
        self.utility_mode = 'legacy'
        self.selected_utility = {}
        from .observed_circuit import ObservedCircuit
        self.circuit = ObservedCircuit()
        self.circuit_tour = {}
        self.execution_admission_refinement = False
        self.vertical_frontier = {}
        self.survey_until_complete = False
        self.survey_status = {}
        self.survey_pending = 0
        self.frontier_count = None
        self.admissible_frontier_count = 0
        self.frontier_complete = False
        self.frontier_confirmation_count = 0
        self.last_frontier_confirmation = None
        self.local_frontier_rescans = 0
        self.frontier_goal_history = deque(maxlen=16)
        self.observed_support_keys = set()
        self.support_progress_bank = 0
        self.last_support_progress_s = None
        self.frontier_decision = {}
        self.committed_goal_rate = None
        self.goal_progress_focus = None
        self.goal_progress_best = math.inf
        self.goal_progress_stamp = 0.
        self.hot_goal_deferrals = 0
        self.goal_progress_yaw = None
        self.connection_probe_count = 0
        self.committed_goal_cost = None
        self.geometry_smoother = None
        self.refined_motions = {}
        self.execution_route = None
        self.execution_route_raw = []
        self.minimum_dose_parent = {}
        self.confident_hot_keys = set()
        self.radiation_frontier_counts = {}
        self.survey_stop_reason = None
        self.radiation_execution_blocked = set()
        self.frontier_views = None
        self.campaign_agenda = None
        self.unproductive_frontier_count = 0
        self.uncertainty_probe_count = 0
        self.uncertainty_probe_target = None
        self.budget_screened = set()
        self.budget_screen_cycles = 0
        self.survival_return_dose=None
        self.survival_root_rate=0.
        self.survival_dose_rejections=set()

    def refine_frontier_route(self,key,position,parent,geometric_parent,novelty,fresh,connector,cost_rates):
        pathwise = self.exposure_policy == 'pathwise' and self.expected_dose_objective
        raw_routes = [(self.terrain.path(parent,key)[1:],'dose_weighted',self.smoother)]
        if pathwise and key in self.minimum_dose_parent:
            dose_route = self.terrain.path(self.minimum_dose_parent,key)[1:]
            if dose_route != raw_routes[0][0]:
                raw_routes.append((dose_route,'dose_only',self.smoother))
        if key in geometric_parent:
            alternative = self.terrain.path(geometric_parent,key)[1:]
            if pathwise:
                raw_routes.append((alternative,'geometric',self.geometry_smoother))
            elif alternative != raw_routes[0][0]:
                raw_routes.append((alternative,'geometric',self.smoother))
        variants = []
        for route,family,smoother in raw_routes:
            raw = [self.terrain.nodes[k].copy() for k in route]
            checked = smoother([position,*raw])
            if not checked.points:
                if checked.metrics.get('fallback_reason') == 'confident_radiation_exclusion':
                    self.radiation_execution_blocked.add(key)
                continue
            queries = np.asarray(checked.points)+[0.,0.,.44]
            mean,std,upper = self.gp.predict(queries)
            policy = self.exposure_cost(queries,mean)
            duration,expected = forecast_polyline(checked.points,policy,start_yaw=getattr(self,'start_yaw',None))
            _,upper_exposure = forecast_polyline(checked.points,np.maximum(upper,policy) if pathwise or self.survival_mode else upper,start_yaw=getattr(self,'start_yaw',None))
            if self.survival_mode and self.terrain.route_experience is not None and self.terrain.route_experience.edges:
                route_parent=geometric_parent if family=='geometric' else self.minimum_dose_parent if family=='dose_only' else parent
                if family not in self.survival_experience_forecasts:
                    self.survival_experience_forecasts[family]=forecast_tree(self.terrain,route_parent,self.survival_rates)[1]
                experienced=self.survival_experience_forecasts[family]
                upper_exposure=max(upper_exposure,experienced[key])
            if self.survival_mode and self.survival_return_dose is not None:
                dwell=(self.survival_root_rate*self.wait_allowance+max(float(upper[-1]),cost_rates[key])*(5.+self.wait_allowance+self.recovery_reserve_s))/3600.
                if self.exceeds_budget(self.sample.dose_upper_usv+upper_exposure+self.survival_return_dose[key]+self.budget_margin+dwell):
                    self.survival_dose_rejections.add(key)
                    continue
            detail = self.utility.frontier_score(self.terrain.nodes[key]+[0.,0.,.44],novelty,fresh,
                duration,expected,cost_rates[key],self.route_rate_scale,self.wait_allowance,connector,dose_weight=self.frontier_dose_weight)
            detail['cost_geometry'] = checked.metrics['mode']
            detail['route_family'] = family
            if pathwise:
                certificate = {k:v for k,v in checked.metrics.items() if k not in
                    ('raw_expected_dose_usv','execution_expected_dose_usv','raw_upper_dose_usv','execution_upper_dose_usv')}
                certificate.update(risk_prediction='policy_evaluated_after_geometry',
                    execution_policy_dose_usv=expected,execution_policy_upper_dose_usv=upper_exposure)
                checked = type(checked)(checked.points,checked.spans,certificate)
            variants.append((detail['score'],raw,detail,duration,expected,upper_exposure,float(np.max(upper)),checked,std))
        if not variants:
            return None
        if pathwise:
            baseline = next((v for v in variants if v[2]['route_family']=='geometric'),None)
            if baseline is None:
                return None
            from scipy.spatial import cKDTree
            baseline_tree = cKDTree(np.asarray(baseline[7].points))
            supported = []
            for variant in variants:
                points = np.asarray(variant[7].points)
                novel = baseline_tree.query(points)[0] > .15
                evidence = np.array([self.utility.neighbourhood_seconds(point+[0.,0.,.44]) for point in points])
                known = (variant[8] <= 1.)&(evidence >= .25)
                permitted = bool(np.all(~novel|known))
                variant[2].update(detour_evidence_supported=permitted,
                    novel_route_points=int(novel.sum()),unobserved_detour_points=int((novel&~known).sum()))
                if permitted:
                    supported.append(variant)
            eligible = [v for v in supported if baseline[4] > 1e-9 and v[4] <= .95*baseline[4]
                        and v[5] <= 1.05*baseline[5]]
            best = min(eligible,key=lambda v:(v[4],v[3])) if eligible else baseline
            progress = self.utility.frontier_score(self.terrain.nodes[key]+[0.,0.,.44],novelty,fresh,
                baseline[3],0.,0.,1.,self.wait_allowance,connector,dose_weight=self.frontier_dose_weight)
            best[2].update(route_score=best[0],geometric_duration_s=baseline[3],
                goal_normalized_cost=progress['normalized_cost'],
                geometric_expected_dose_usv=baseline[4],geometric_upper_dose_usv=baseline[5],
                selected_expected_dose_usv=best[4],selected_upper_dose_usv=best[5],
                expected_dose_saving_fraction=(baseline[4]-best[4])/baseline[4] if baseline[4]>1e-9 else 0.)
            best[2]['score'] = progress['score']
            best = (progress['score'],*best[1:])
        else:
            best = max(variants,key=lambda item:item[0])
        self.refined_motions[key] = best[7]
        return best[:7]

    def defer_hot_stalled_goal(self,position,stamp,previous,cost_rates):
        if (self.returning or not self.survey_until_complete or not self.expected_dose_objective or self.exposure_policy == 'standard' or previous is None
                or self.target is None or self.goal_progress_focus is None or not self.sample.position_valid):
            return False
        distance = float(np.linalg.norm(np.asarray(position)-self.goal_progress_focus))
        if distance < self.goal_progress_best-.05:
            self.goal_progress_best,self.goal_progress_stamp = distance,stamp
        yaw = getattr(self,'start_yaw',None)
        if yaw is not None:
            if self.goal_progress_yaw is not None:
                change = math.atan2(math.sin(yaw-self.goal_progress_yaw),math.cos(yaw-self.goal_progress_yaw))
                if abs(change) > .1:
                    self.goal_progress_stamp = stamp  # A necessary turn is physical progress.
            self.goal_progress_yaw = yaw
        lower = max(0.,self.sample.rate_usv_h-3.*math.sqrt(self.sample.variance_usv_h2))
        reference = max(.05,math.exp(self.gp.prior+.5*self.gp.variance) if hasattr(self.gp,'prior') else self.route_rate_scale)
        # Do not interrupt a turn/detour toward a clearly cooler escape target.
        if lower <= 3.*reference or cost_rates[previous] < .5*lower:
            return False
        patience = max(3.,min(15.,15.*reference/max(lower,.05)))
        if (stamp-self.goal_progress_stamp < patience or self.last_support_progress_s is None
                or stamp-self.last_support_progress_s < patience):
            return False
        self.failed.append((self.target.copy(),stamp))
        self.hot_goal_deferrals += 1
        self.goal_progress_focus = None
        return True

    def observe_plan_delay(self, seconds):
        if math.isfinite(seconds) and seconds >= 0:
            self.plan_delays = (self.plan_delays+[float(seconds)])[-128:]
            self.wait_allowance = max(3., float(np.percentile(self.plan_delays, 95)))

    def _remember(self, key, status, stamp):
        if key not in self.agenda and len(self.agenda) >= self.agenda_capacity:
            completed = next((k for k, value in self.agenda.items() if value['status'] in ('visited', 'support_lost')), None)
            if completed is None:
                self.agenda_overflow += 1
                return
            del self.agenda[completed]
        self.agenda[key] = dict(position=self.terrain.nodes[key].tolist(), status=status, updated_s=stamp)

    def observe_landmarks(self, observations):
        for observation in observations:
            previous = self.landmarks.get(observation.identifier)
            if previous is None or observation.stamp_s >= previous.stamp_s:
                self.landmarks[observation.identifier] = observation

    def landmark_bonus(self, point):
        # Known, uninspected targets only. Safe support and admission remain
        # mandatory; no target catalogue, floor labels or unseen IDs arrive here.
        return 8.*sum(not o.inspected and np.linalg.norm(np.asarray(o.position)-(point+[0., 0., .44])) <= 1.2
                      for o in self.landmarks.values())

    def report_stall(self, position, stamp):
        root = self.terrain.nearest(position)
        next_point = next((p for p in self.path if math.dist(position, p) > .2), None)
        other = None if next_point is None else self.terrain.nearest(next_point)
        if root is not None and other in self.terrain.graph.get(root, []):
            self.blocked_edges[tuple(sorted((root, other)))] = stamp
        if self.target is not None:
            self.failed.append((self.target.copy(), stamp))
        self.path = []

    def observe_radiation(self, sample, replay=False):
        super().observe_radiation(sample, replay=replay)
        if self.rate_limited and sample.rate_usv_h > self.rate_limit:
            self.last_reason = 'measured_rate_limit'
        elif self.budget_enabled and sample.dose_upper_usv >= self.budget*.8:
            self.last_reason = 'measured_dose_reserve'

    def replan(self, position, stamp):
        started = time.perf_counter()
        self.smoother = None
        self.geometry_smoother = None
        self.refined_motions = {}
        self.execution_route = None
        self.execution_route_raw = []
        self.minimum_dose_parent = {}
        self.survival_experience_forecasts={}
        self.survival_return_dose=None
        if not self.returning:
            self.survival_dose_rejections=set()
        self.confident_hot_keys = set()
        if not self.returning:
            self.radiation_execution_blocked = set()
            self.survey_stop_reason = None
        if self.radiation_exclusion is not None:
            self.radiation_exclusion.refresh(self.gp,self.utility)
        if self.returning and self.survey_stop_reason is None:
            self.survey_stop_reason = self.last_reason
        self.return_path = []
        uncertainty_probes=[]
        nodes, graph = self.terrain.build_graph()
        self.blocked_edges = {edge: t for edge, t in self.blocked_edges.items() if stamp-t < 60.}
        for a, b in self.blocked_edges:
            if a in graph and b in graph[a]:
                graph[a].remove(b)
            if b in graph and a in graph[b]:
                graph[b].remove(a)
        self.terrain.refresh_shared_route_edges()
        if self.smooth_execution:
            from .path_motion import RouteSmoother
            self.smoother = RouteSmoother(self.terrain, self.planning_gp, self.smoothing_rate_limit,
                                         prediction='mean' if self.expected_dose_objective else 'upper',
                                         admission=self.radiation_exclusion.allows_route if self.radiation_exclusion else None)
            if self.expected_dose_objective and self.exposure_policy == 'pathwise':
                from .radiation_blind import BlindGP
                self.geometry_smoother = RouteSmoother(self.terrain,BlindGP(1),math.inf,prediction='mean',
                    admission=self.radiation_exclusion.allows_route if self.radiation_exclusion else None)
        root = self.terrain.nearest(position)
        self.rejections = dict(distance=0, failed=0, dose=0, rate=0, return_route=0, time=0)
        self.route_forecast = {}
        self.vertical_frontier = {}
        if self.sample is None or not -1e-9 <= stamp-self.sample.stamp_s <= 1.5:
            self.path = []
            self.state, self.last_reason = 'waiting_for_radiation', 'stale_radiation_sample'
            return
        if root is None:
            self.path = []
            self.state, self.last_reason = 'waiting_for_support', 'missing_local_support'
            return
        if self.home is None:
            self.home = np.asarray(position).copy()
            self.home_stamp = stamp
        home = self.terrain.nearest(self.home)
        if home is None:
            self.path = []
            self.state, self.last_reason = 'no_observed_return_route', 'home_support_unavailable'
            return
        self.visited.add(root)
        if self.campaign_agenda is not None:
            self.campaign_agenda.observe_position(position)
        keys = list(nodes)
        mean, sigma, upper = self.gp.predict([nodes[key]+[0, 0, .44] for key in keys])
        rates, uncertainty = dict(zip(keys, upper)), dict(zip(keys, sigma))
        self.survival_rates=rates
        means = dict(zip(keys, mean))
        cost_mean = self.exposure_cost(np.asarray([nodes[key]+[0.,0.,.44] for key in keys]),mean)
        cost_rates = dict(zip(keys,cost_mean)) if self.expected_dose_objective else rates
        if self.radiation_exclusion is not None:
            _, excluded = self.radiation_exclusion.evaluate([nodes[k]+[0.,0.,.44] for k in keys])
            self.confident_hot_keys = {k for k,blocked in zip(keys,excluded) if blocked}
        if self.survey_until_complete:
            known = {(key[0],key[1],round(float(point[2])/.2)) for key,point in nodes.items()} if keys and isinstance(keys[0],tuple) else set(keys)
            self.support_progress_bank += len(known-self.observed_support_keys)
            self.observed_support_keys.update(known)
            if self.last_support_progress_s is None or self.support_progress_bank >= 3:
                self.last_support_progress_s,self.support_progress_bank = stamp,0
        ceiling = self.rate_limit if self.rate_limited else None
        _, _, length, parent = self.terrain.routes(root, cost_rates, rate_scale=self.effective_route_rate_scale, max_rate=ceiling,
            excluded=self.confident_hot_keys)
        if self.expected_dose_objective and self.exposure_policy == 'pathwise':
            # Exposure-first search, with a tiny travel-time tie breaker. This
            # adds a genuinely minimum-dose alternative, not just time+risk.
            self.minimum_dose_parent = self.terrain.routes(root,cost_rates,rate_scale=1e-6,excluded=self.confident_hot_keys)[3]
        emergency_return = False
        if home not in parent:
            # Do not authorize new exploration without a rate-compliant return.
            # If already trapped by a changed prediction, explicitly retreat on
            # the best observed dose-weighted route rather than hide the violation.
            _, _, length, parent = self.terrain.routes(root, cost_rates, rate_scale=self.effective_route_rate_scale)
            emergency_return = True
            self.returning, self.last_reason = True, ('no_confidence_admissible_return_route'
                if self.confident_hot_keys and not self.rate_limited else 'no_rate_compliant_return_route')
        inspection = getattr(self.terrain, 'active_viewpoints', False)
        def new_height(key):
            return inspection and round(float(nodes[key][2])/.2) not in self.sampled_height_bands
        if inspection:
            band = round(float(position[2])/.2)
            if band not in self.sampled_height_bands:
                self.last_support_progress_s = stamp
            self.sampled_height_bands.add(band)
            # Mark supported body-footprint neighbours as visited, not just one
            # sparse root sample every two seconds. Never cross disconnected walls.
            self.visited.update(key for key, distance in length.items() if distance <= .3)
            self.region_agenda.update(nodes, graph, root, stamp, self.visited)
        _, _, home_length, home_parent = self.terrain.routes(home, cost_rates, rate_scale=self.effective_route_rate_scale, reverse=True, max_rate=ceiling,
            excluded=self.confident_hot_keys)
        if home not in parent:
            self.path = []
            self.state, self.last_reason = 'no_observed_return_route', 'return_graph_disconnected'
            return
        self.return_path = [nodes[key].copy() for key in self.terrain.path(parent, home)]
        outbound_time, dose, outbound_peak = forecast_tree(self.terrain, parent, rates)
        _, objective_dose, _ = forecast_tree(self.terrain, parent, cost_rates)
        home_time, home_dose, home_peak = forecast_tree(self.terrain, home_parent, rates, reverse=True)
        self.survival_return_dose=home_dose
        self.survival_root_rate=max(rates[root],self.sample.rate_usv_h+3.*math.sqrt(self.sample.variance_usv_h2)) if self.survival_mode else rates[root]
        self.route_forecast = dict(return_duration_s=outbound_time[home], return_upper_dose_usv=dose[home],
            return_peak_rate_usv_h=outbound_peak[home], emergency_rate_override=emergency_return,
            wait_allowance_s=self.wait_allowance, speed_m_s=FORECAST_SPEED, yaw_rate_rad_s=FORECAST_YAW_RATE)
        return_alternatives=None
        if (self.survival_mode and self.campaign_agenda is not None and
                getattr(self,'radiation_awareness','aware')=='aware' and self.smooth_execution and not emergency_return
                and len(self.return_path)>1):
            from .return_alternatives import best_return_route
            return_alternatives=best_return_route(self.terrain,root,home,position,parent,rates,
                self.gp,self.smoother,excluded=self.confident_hot_keys,start_yaw=getattr(self,'start_yaw',None),
                local_upper=self.survival_root_rate)
            if return_alternatives is not None:
                best=return_alternatives['best']
                self.return_path=best['raw']
                outbound_time[home],dose[home]=best['duration_s'],best['upper_dose_usv']
                self.route_forecast.update(return_duration_s=best['duration_s'],return_upper_dose_usv=best['upper_dose_usv'],
                    return_peak_rate_usv_h=best['peak_rate_usv_h'],return_forecast_geometry='certified_upper_comparison',
                    return_route_family=best['family'],return_candidates=[{k:v for k,v in item.items() if k not in ('raw','motion')}
                        for item in return_alternatives['variants']])
        if self.smooth_execution and len(self.return_path) > 1 and return_alternatives is None:
            checked = self.smoother([position, *self.return_path], returning=True)
            _, _, return_rates = self.gp.predict(np.asarray(checked.points)+[0., 0., .44])
            seconds, exposure = forecast_polyline(checked.points, return_rates, start_yaw=getattr(self, 'start_yaw', None))
            if self.survival_mode and self.terrain.route_experience is not None and self.terrain.route_experience.edges:
                seconds,exposure=max(seconds,outbound_time[home]),max(exposure,dose[home])
            self.route_forecast.update(return_grid_duration_s=outbound_time[home],
                return_duration_s=seconds, return_upper_dose_usv=exposure,
                return_peak_rate_usv_h=float(np.max(return_rates)), return_forecast_geometry='support_checked_smooth')
            outbound_time[home], dose[home] = seconds, exposure
        self.return_reserve = dose[home]+self.budget_margin+self.survival_root_rate*(self.wait_allowance+self.recovery_reserve_s)/3600.
        self.route_forecast['recovery_reserve_s'] = self.recovery_reserve_s
        if self.exceeds_budget(self.sample.dose_upper_usv+self.return_reserve, inclusive=True):
            self.returning, self.last_reason = True, 'dose_return_reserve'
        if stamp+outbound_time[home]+self.wait_allowance+15. >= self.mission_duration:
            self.returning, self.last_reason = True, 'time_return_reserve'
        self.failed = [(p, t) for p, t in self.failed if stamp-t < 60.]
        if self.survey_until_complete:
            from .survey_comparison import survey_admission
            self.visited.update(key for key, distance in length.items() if distance <= .25)
            safe_parent = self.terrain.routes(root, rates, max_rate=ceiling,excluded=self.confident_hot_keys)[3] if emergency_return else parent
            self.survey_status = survey_admission(self, keys, root, home, rates, safe_parent, home_parent,
                dose, home_dose, outbound_time, home_time, stamp)
        checked_inspections = set()
        visit_distance = None
        refinement_keys = set()
        if self.execution_admission_refinement and self.smooth_execution:
            previous_key = self.terrain.nearest(self.target) if self.target is not None else None
            if previous_key is not None:
                refinement_keys.add(previous_key)
            if inspection and not self.returning:
                visit_distance = distance_from_visits(nodes, graph, self.visited)
                promising = [key for key in parent if key in home_parent and length[key] >= .25
                    and self.terrain.acquisition_bonus(key, self.sampled_height_bands) > 0]
                promising.sort(key=lambda key: (
                    -min(1., visit_distance[key]/.8)*self.terrain.acquisition_bonus(key, self.sampled_height_bands)/(1.+.35*length[key]), key))
                refinement_keys.update(promising[:max(0, 4-len(refinement_keys))])
        for key, task in self.agenda.items():
            if task['status'] == 'active':
                task['status'] = 'pending'
            if key not in nodes:
                task['status'] = 'support_lost'
            elif key not in parent:
                task['status'] = 'disconnected_or_rate_blocked'
            elif key in self.visited and self.landmark_bonus(nodes[key]) == 0:
                task['status'] = 'visited'

        def admitted(key):
            if key not in parent or key not in home_dose:
                self.rejections['return_route'] += 1
                self._remember(key, 'no_rate_compliant_return', stamp)
                return False
            if any(math.dist(nodes[key], p) < .65 for p, _ in self.failed):
                self.rejections['failed'] += 1
                self._remember(key, 'temporarily_blocked', stamp)
                return False
            dwell_dose = (rates[root]*self.wait_allowance+rates[key]*(5.+self.wait_allowance+self.recovery_reserve_s))/3600.
            total = self.sample.dose_upper_usv+dose[key]+home_dose[key]+self.budget_margin+dwell_dose
            deadline = stamp+outbound_time[key]+home_time[key]+2*self.wait_allowance+15.+self.recovery_reserve_s
            if (self.smooth_execution and (self.landmark_bonus(nodes[key]) > 0 or self.objective == 'circuit' or key in refinement_keys) and key not in checked_inspections
                    and len(checked_inspections) < (8 if self.survival_mode else 4) and (self.exceeds_budget(total) or deadline > self.mission_duration)):
                # Grid turns can grossly overestimate a nearby inspection trip.
                # Recheck a bounded number of promising known-target approaches
                # on certified execution geometry before rejecting the objective.
                checked_inspections.add(key)
                outbound = [position, *[nodes[k] for k in self.terrain.path(parent, key)[1:]]]
                inbound = [nodes[k] for k in reversed(self.terrain.path(home_parent, key))]
                for points, durations, doses, peaks, returning in (
                    (outbound, outbound_time, dose, outbound_peak, False),
                    (inbound, home_time, home_dose, home_peak, True)):
                    checked = self.smoother(points, returning=returning)
                    if not returning and not checked.points and checked.metrics.get('fallback_reason')=='confident_radiation_exclusion':
                        self.radiation_execution_blocked.add(key)
                        self.rejections['rate']+=1
                        self._remember(key,'no_confidence_admissible_execution_route',stamp)
                        return False
                    query = checked.points or points
                    _, _, prediction = self.gp.predict(np.asarray(query)+[0., 0., .44])
                    old_duration,old_dose=durations[key],doses[key]
                    durations[key], doses[key] = forecast_polyline(query, prediction,
                        start_yaw=None if returning else getattr(self, 'start_yaw', None))
                    if self.survival_mode and self.terrain.route_experience is not None and self.terrain.route_experience.edges:
                        durations[key],doses[key]=max(old_duration,durations[key]),max(old_dose,doses[key])
                    peaks[key] = float(np.max(prediction))
                total = self.sample.dose_upper_usv+dose[key]+home_dose[key]+self.budget_margin+dwell_dose
                deadline = stamp+outbound_time[key]+home_time[key]+2*self.wait_allowance+15.+self.recovery_reserve_s
            if self.exceeds_budget(total):
                self.rejections['dose'] += 1
                self._remember(key, 'dose_limited', stamp)
                return False
            if self.rate_limited and max(outbound_peak[key], home_peak[key]) > self.rate_limit:
                self.rejections['rate'] += 1
                self._remember(key, 'rate_limited', stamp)
                return False
            if deadline > self.mission_duration:
                self.rejections['time'] += 1
                self._remember(key, 'time_limited', stamp)
                return False
            self._remember(key, 'pending', stamp)
            return True

        utility_details = {}
        frontier_paths = {}
        def information_score(key, novelty, fresh_fraction, target_bonus=0., connector_bonus=0.):
            detail = self.utility.score(nodes[key]+[0., 0., .44], novelty, fresh_fraction,
                means[key], uncertainty[key], outbound_time[key], home_time[key], dose[key], home_dose[key],
                rates[key], rates[root], self.wait_allowance, self.recovery_reserve_s,
                self.mission_duration-stamp, self.remaining_budget,
                self.sample.counts_per_usv, target_bonus, connector_bonus)
            utility_details[key] = detail
            return detail['score']

        if self.objective == 'circuit':
            if not self.returning and not self.circuit.prepare(self.terrain, root, self.home, stamp, getattr(self, 'last_packet_sequence', None)):
                self.path, self.state, self.last_reason = [], 'waiting_for_task_observations', 'circuit_not_observable'
                return
            self.circuit.observe_position(np.asarray(position), stamp)
            if self.circuit.completed:
                self.returning, self.last_reason = True, 'observed_circuit_complete'

        previous = None if self.target is None else self.terrain.nearest(self.target)
        hot_goal_deferred = self.defer_hot_stalled_goal(position,stamp,previous,cost_rates)
        reached_tolerance = .2 if inspection or self.objective == 'circuit' or self.survey_until_complete else .4
        keep_previous = (not self.returning and self.objective != 'circuit' and self.commitment
            and previous is not None and math.dist(nodes[previous], self.target) < .25
            and math.dist(position, self.target) > reached_tolerance
            and stamp-self.goal_started < 45. and admitted(previous))
        risk_preempted = bool(keep_previous and self.exposure_policy != 'pathwise' and self.expected_dose_objective and self.committed_goal_rate is not None
            and stamp-self.goal_started >= 1.
            and cost_rates[previous] > max(2.*self.committed_goal_rate,3.*self.route_rate_scale))
        if risk_preempted:
            keep_previous = False
        unseen_height = [key for key in parent if new_height(key)] if self.objective == 'vertical' else []
        if self.objective == 'vertical':
            self.vertical_frontier = dict(observed_nodes=len(nodes), reachable_nodes=len(parent),
                reachable_new_height_nodes=len(unseen_height),
                maximum_observed_height_m=max(float(p[2]) for p in nodes.values()),
                maximum_reachable_height_m=max(float(nodes[k][2]) for k in parent),
                admitted_height_candidates=0, commitment_preempted=False)
        # Newly observed height must compete before the 45-second flat-goal
        # commitment expires. Keep the old admitted goal if no vertical candidate
        # survives the ordinary return/rate/dose/time checks below.
        reconsider_height = bool(unseen_height) and previous is not None and not new_height(previous)
        selected = None
        if self.returning:
            selected, self.state = home, 'returning'
            if math.dist(position, self.home) < self.home_tolerance:
                self.target = self.home.copy()
                self._remember(home, 'return_home', stamp)
                self.path, self.state = [], 'returned'
                return
        elif self.objective == 'circuit':
            candidates = []
            for key in self.circuit.pending(self.terrain):
                if admitted(key):
                    score = information_score(key, 0., 0., target_bonus=8.) if self.utility_mode == 'information' else (
                        8.+.5*math.log1p(uncertainty[key]**2/.02))/(1.+.35*length[key]+5.*dose[key])
                    candidates.append((score, key))
            self.candidate_count = len(candidates)
            if not candidates:
                self.returning = True
                selected, self.state, self.last_reason = home, 'returning_incomplete_circuit', 'circuit_goals_not_admissible'
            else:
                viable = {key for _, key in candidates}
                if previous in viable and stamp-self.goal_started < 45.:
                    selected = previous
                else:
                    selected = max(candidates)[1]
                    self.circuit_tour = {}
                    if self.utility_mode == 'information':
                        tour = self.circuit.best_tour(self.terrain, root, home, position, self.home, rates, self.gp,
                            self.rate_limit, viable, self.mission_duration-stamp,
                            self.remaining_budget, self.smoother,
                            yaw=getattr(self, 'start_yaw', None), wait_s=self.wait_allowance, recovery_s=self.recovery_reserve_s)
                        if tour is not None:
                            selected = tour['first']
                            self.circuit_tour = dict(planned_s=stamp, duration_s=tour['duration_s'], upper_dose_usv=tour['upper_dose_usv'],
                                length_m=tour['length_m'], normalized_cost=tour['normalized_cost'],
                                remaining_goals=[nodes[k].tolist() for k in tour['order']],
                                scope='support/risk-checked remaining-task lookahead; only first leg dispatched')
                if selected != previous or stamp-self.goal_started >= 45.:
                    self.goal_started = stamp
                self.state = 'exploring'
                self.last_reason = 'observed_circuit_tour' if self.circuit_tour else 'observed_circuit_goal'
        elif self.survey_until_complete:
            from .survey_comparison import AVAILABLE, RADIATION, BUDGET, TIME
            # Frontier standoffs are observed support nodes with an occlusion-
            # aware view of unknown support. Visited nodes remain frontiers until
            # new sensing resolves the unknown; visiting is not completion.
            gain = ObservedGain(self.terrain)
            gains = gain.values(keys) if hasattr(gain,'values') else [gain.value(key) for key in keys]
            pending = [key for key,value in zip(keys,gains) if value > 0]
            marginal = {key:int(value) for key,value in zip(keys,gains)}
            if self.frontier_views is not None and hasattr(gain,'unknown_cells'):
                self.frontier_views.observe(gain.unknown_cells(root),position,
                    getattr(self.terrain,'last_scan_stamp',None) or getattr(self,'last_packet_sequence',stamp))
                marginal = {k:self.frontier_views.marginal_gain(gain.unknown_cells(k),nodes[k]) for k in keys}
            self.unproductive_frontier_count = int(sum(marginal[k]==0 for k in pending))
            from .survey_comparison import DISCONNECTED, TEMPORARY
            self.radiation_frontier_counts = dict(
                radiation_blocked=sum(self.survey_status.get(k)==RADIATION for k in pending),
                confident_hot=sum(k in self.confident_hot_keys for k in pending),
                geometrically_unreachable=sum(self.survey_status.get(k)==DISCONNECTED for k in pending),
                temporary_blocked=sum(self.survey_status.get(k)==TEMPORARY for k in pending))
            self.frontier_count = self.survey_pending = len(pending)
            fresh_scan = getattr(self, 'last_packet_sequence', stamp)
            if not pending:
                if fresh_scan != self.last_frontier_confirmation:
                    self.frontier_confirmation_count += 1
                    self.last_frontier_confirmation = fresh_scan
                self.frontier_complete = self.frontier_confirmation_count >= 3
                self.path, self.target, self.candidate_count = [], None, 0
                self.admissible_frontier_count = 0
                self.connection_probe_count = 0
                self.state = 'finished' if self.frontier_complete else 'confirming_frontier_exhaustion'
                self.last_reason = 'frontiers_exhausted' if self.frontier_complete else 'confirming_no_frontiers'
                if self.frontier_complete and self.survival_mode:
                    self.returning=True
                    self.state,self.last_reason='returning_completed_survey','frontiers_exhausted_returning_for_swap'
                    self.survey_stop_reason=self.last_reason
                    self.target=nodes[home].copy()
                    self.path=self.return_path[1:]
                    if return_alternatives is not None:
                        self.execution_route=return_alternatives['best']['motion']
                        self.execution_route_raw=[p.copy() for p in self.path]
                return
            self.frontier_confirmation_count, self.last_frontier_confirmation = 0, None
            self.frontier_complete = False
            visit_distance = distance_from_visits(nodes, graph, self.visited)
            if self.survival_mode and self.smooth_execution:
                # Grid turns can reject every frontier even when a certified
                # smooth round trip fits the reserve. Recheck a bounded spread
                # before treating budget-limited work as unavailable.
                reconsider=[k for k in pending if marginal[k]>0 and k in parent and k in home_parent
                    and math.dist(nodes[k],position)>=.25 and self.survey_status.get(k) in (BUDGET,TIME)]
                near=sorted(reconsider,key=lambda k:(length[k],k))
                useful=sorted(reconsider,key=lambda k:(-marginal[k]/(1.+length[k]),k))
                screen=(self.campaign_agenda.screening_candidates(reconsider,nodes,length,marginal,self.budget_screened)
                    if self.campaign_agenda is not None else list(dict.fromkeys(near[:4]+useful[:4])))
                for key in screen:
                    refinement_keys.add(key)
                    if admitted(key):
                        self.survey_status[key]=AVAILABLE
                    elif self.campaign_agenda is not None:
                        self.budget_screened.add(key)
            candidates = [key for key in pending if marginal[key] > 0 and math.dist(nodes[key], position) >= .25
                          and self.survey_status.get(key) == AVAILABLE and admitted(key)]
            self.admissible_frontier_count = len(candidates)
            probes = []
            if inspection:
                # A fully observed ramp is not itself a geometric frontier, but
                # a fresh viewpoint on that known connector can reveal/qualify
                # a route to existing frontiers. Never fabricate a new frontier
                # or bypass the fine-graph outward/return admission checks.
                pending_set = set(pending)
                probes = [key for key in keys if key not in pending_set and new_height(key)
                    and math.dist(nodes[key],position) >= .25
                    and self.survey_status.get(key) == AVAILABLE and admitted(key)]
                candidates += probes
            self.uncertainty_probe_count=0
            uncertainty_probes=[]
            if self.campaign_agenda is not None and getattr(self,'radiation_awareness','aware')=='aware' and not candidates:
                from .uncertainty_probe import probe_candidates,target_still_uncertain
                uncertainty_probes=probe_candidates(nodes,set(pending),self.survey_status,uncertainty,length,
                    self.utility,AVAILABLE,{BUDGET,RADIATION})
                # The moving robot can collect a little evidence near its goal
                # before arrival. Do not drop an otherwise valid probe merely
                # because that first evidence crosses the "unmeasured" gate.
                if (keep_previous and previous is not None and self.uncertainty_probe_target is not None
                        and math.dist(nodes[previous],self.uncertainty_probe_target)<.25
                        and self.survey_status.get(previous)==AVAILABLE
                        and target_still_uncertain(nodes[previous],nodes,pending,self.survey_status,uncertainty,{BUDGET,RADIATION})
                        and previous not in uncertainty_probes):
                    uncertainty_probes.append(previous)
                uncertainty_probes=[key for key in uncertainty_probes if admitted(key)]
                self.uncertainty_probe_count=len(uncertainty_probes)
                candidates+=uncertainty_probes
                # Positive information-proxy reward is only for scheduling;
                # it does not change the unresolved frontier count.
                for key in uncertainty_probes:
                    marginal[key]=1
            self.connection_probe_count = len(probes)
            self.candidate_count = len(candidates)
            geometric_parent = {}
            geometric_length,geometric_time = {},{}
            if self.expected_dose_objective and self.smooth_execution:
                _,_,geometric_length,geometric_parent = self.terrain.routes(root,excluded=self.confident_hot_keys)
                if self.exposure_policy == 'pathwise':
                    geometric_time = forecast_tree(self.terrain,geometric_parent,dict.fromkeys(keys,0.))[0]
            route_cost_preempted = False
            if keep_previous and previous in candidates and self.expected_dose_objective and self.smooth_execution:
                connector = self.terrain.acquisition_bonus(previous,self.sampled_height_bands) if inspection else 0.
                refreshed = self.refine_frontier_route(previous,position,parent,geometric_parent,marginal[previous],
                    min(1.,visit_distance[previous]/.8),connector,cost_rates)
                if refreshed is not None:
                    _,frontier_paths[previous],utility_details[previous],outbound_time[previous],objective_dose[previous],dose[previous],outbound_peak[previous] = refreshed
                    refreshed_cost = utility_details[previous].get('goal_normalized_cost',utility_details[previous]['normalized_cost']) if self.exposure_policy=='pathwise' else utility_details[previous]['normalized_cost']
                    route_cost_preempted = bool(self.committed_goal_cost is not None and stamp-self.goal_started >= 1.
                        and refreshed_cost > 2.*self.committed_goal_cost)
                    if route_cost_preempted:
                        keep_previous = False
                elif self.radiation_exclusion is not None:
                    keep_previous = False
            if keep_previous and previous in candidates:
                selected, self.state, self.last_reason = previous, 'exploring', 'continue_survey_sweep'
                self.frontier_decision['route_cost_preempted'] = False
            elif candidates:
                self.local_frontier_rescans = 0
                self.budget_screened.clear()
                self.budget_screen_cycles=0
                candidates.sort(key=lambda key: (geometric_length.get(key,length[key]) if self.exposure_policy=='pathwise' else length[key], key))
                shortlist = candidates[:24]+candidates[::max(1, len(candidates)//24)]
                if inspection:
                    # Preserve observed connector viewpoints when subsampling;
                    # no true floor labels or unseen geometry are supplied.
                    by_height = {}
                    for key in candidates:
                        by_height.setdefault(round(float(nodes[key][2])/.2), key)
                    shortlist += list(by_height.values())[:16]
                for key in set(shortlist):
                    connector = self.terrain.acquisition_bonus(key, self.sampled_height_bands) if inspection else 0.
                    if self.exposure_policy == 'pathwise':
                        utility_details[key] = self.utility.frontier_score(nodes[key]+[0.,0.,.44],marginal[key],
                            min(1.,visit_distance[key]/.8),geometric_time.get(key,outbound_time[key]),0.,0.,1.,self.wait_allowance,connector,dose_weight=self.frontier_dose_weight)
                    else:
                        utility_details[key] = self.utility.frontier_score(nodes[key]+[0., 0., .44], marginal[key],
                            min(1., visit_distance[key]/.8), outbound_time[key], objective_dose[key],
                            cost_rates[key], self.route_rate_scale, self.wait_allowance, connector,dose_weight=self.frontier_dose_weight)
                ranked = sorted(((utility_details[key]['score'], key, marginal[key]) for key in set(shortlist)), reverse=True)
                refined_count = 0
                if self.expected_dose_objective and self.smooth_execution:
                    # Grid turning/dose forecasts can mis-rank an execution that
                    # will be rounded. Compare a bounded number of certified
                    # execution routes, including a geometric alternative.
                    chosen = [key for _,key,_ in ranked[:4]]
                    regions, bands = set(), set()
                    for _,key,_ in ranked:
                        region = self.region_agenda.membership.get(key,key)
                        band = round(float(nodes[key][2])/.2)
                        if (region not in regions or band not in bands) and key not in chosen and len(chosen) < 10:
                            chosen.append(key)
                        regions.add(region)
                        bands.add(band)
                    refined = []
                    for key in chosen:
                        connector = self.terrain.acquisition_bonus(key,self.sampled_height_bands) if inspection else 0.
                        best = self.refine_frontier_route(key,position,parent,geometric_parent,marginal[key],
                            min(1.,visit_distance[key]/.8),connector,cost_rates)
                        if best is not None:
                            _,frontier_paths[key],utility_details[key],outbound_time[key],objective_dose[key],dose[key],outbound_peak[key] = best
                            refined.append((best[0],key,marginal[key]))
                    if refined:
                        ranked = sorted(refined,reverse=True)
                        refined_count = len(refined)
                    elif self.radiation_exclusion is not None or self.survival_mode:
                        # None of the bounded execution candidates is admissible.
                        # Keep unresolved frontiers visible, and return rather
                        # than dispatch an uncertified raw path through a hotspot.
                        self.returning = True
                        self.state,self.last_reason = 'returning_survey_limit',('radiation_frontiers_blocked' if self.survival_dose_rejections else
                            'no_confidence_admissible_execution_route' if self.radiation_execution_blocked else 'unresolved_frontiers')
                        self.survey_stop_reason = self.last_reason
                        self.target,self.path = nodes[home].copy(),self.return_path[1:]
                        if return_alternatives is not None:
                            self.execution_route=return_alternatives['best']['motion']
                            self.execution_route_raw=[p.copy() for p in self.path]
                        self.admissible_frontier_count = 0
                        return
                tour=None
                if self.campaign_agenda is not None:
                    if self.survival_mode and self.budget_enabled and hasattr(gain,'unknown_cells'):
                        tour=self.campaign_agenda.select_tour(ranked,self.terrain,rates,outbound_time,dose,home_dose,
                            self.budget-self.sample.dose_upper_usv,self.budget_margin,self.wait_allowance,
                            gain.unknown_cells,self.confident_hot_keys,home_time)
                    _,selected,_=self.campaign_agenda.choose(ranked,nodes,outbound_time,objective_dose,home_dose)
                    scores={key:score for score,key,_ in ranked}
                    # A coarse advisory tour cannot replace a substantially
                    # better certified immediate goal; avoid long detours for
                    # speculative second-view gain.
                    if tour is not None and scores[tour[1]]>=.9*scores[selected]:
                        selected=tour[1]
                    else:
                        tour=None
                elif inspection and self.region_scheduling == 'hierarchical':
                    goal_time = {key:utility_details[key].get('geometric_duration_s',outbound_time[key]) for _,key,_ in ranked}
                    _, selected, _ = self.region_agenda.choose(ranked, goal_time if self.exposure_policy=='pathwise' else outbound_time, stamp)
                else:
                    selected = ranked[0][1]
                self.frontier_decision = dict(prediction=('mean_plus_near_field' if self.exposure_policy != 'standard' else 'mean') if self.expected_dose_objective else 'upper',
                    rate_reference_usv_h=self.route_rate_scale,
                    execution_refined_candidates=refined_count, goal_preempted_for_risk=risk_preempted,
                    exposure_policy=self.exposure_policy,hot_goal_deferred=hot_goal_deferred,
                    campaign_tour=(dict(first=nodes[tour[1]].tolist(),second=nodes[tour[2]].tolist(),
                        predicted_upper_dose_usv=tour[3],duration_s=tour[4],additional_unknown_cells=tour[5],
                        scope='Advisory bounded two-goal graph lookahead; second goal requires fresh admission') if tour else None),
                    route_cost_preempted=route_cost_preempted,
                    shortlisted=[dict(position=nodes[key].tolist(), mean_rate_usv_h=means[key],
                        policy_rate_usv_h=cost_rates[key],upper_rate_usv_h=rates[key], **utility_details[key]) for _, key, _ in ranked[:8]])
                self.state, self.last_reason = 'exploring', 'systematic_observed_survey'
                if selected in probes:
                    self.last_reason = 'frontier_connection_probe'
                if selected in uncertainty_probes:
                    self.last_reason = 'uncertainty_measurement_probe'
                if hot_goal_deferred:
                    self.last_reason = 'radiation_standoff_reconsidered'
                self.goal_started = stamp
            else:
                if self.campaign_agenda is not None and self.survival_mode and self.smooth_execution:
                    # Recheck at most three further batches. Charge their
                    # simulated waiting time at the current upper rate on top
                    # of the existing return reserve; never weaken admission.
                    unscreened=any(k not in self.budget_screened for k in reconsider)
                    safe=self.campaign_agenda.can_wait_for_screening(self.sample.dose_upper_usv,self.return_reserve,
                        self.survival_root_rate,self.budget,stamp,outbound_time[home],self.wait_allowance,self.mission_duration)
                    if unscreened and safe and self.budget_screen_cycles<3:
                        self.budget_screen_cycles+=1
                        self.path,self.target=[],None
                        self.state,self.last_reason='frontier_rescan','screening_budget_frontiers'
                        return
                local = [key for key in pending if marginal[key] > 0 and self.survey_status.get(key) == AVAILABLE
                         and math.dist(nodes[key], position) < .25]
                if local and self.local_frontier_rescans < 3:
                    self.local_frontier_rescans += 1
                    self.path, self.target = [], None
                    self.state, self.last_reason = 'frontier_rescan', 'resensing_local_frontier'
                    return
                blocked = {self.survey_status[key] for key in pending}
                reason = ('radiation_frontiers_blocked' if blocked and blocked <= {RADIATION, BUDGET}
                          else 'frontier_views_unproductive' if pending and self.unproductive_frontier_count==len(pending)
                          else 'time_return_reserve' if TIME in blocked else 'unresolved_frontiers')
                self.returning = True
                selected, self.state, self.last_reason = home, 'returning_survey_limit', reason
        elif keep_previous and not reconsider_height:
            selected, self.state, self.last_reason = previous, 'exploring', 'continue_revalidated_goal'
        else:
            if self.target is not None and math.dist(position, self.target) < reached_tolerance+.1:
                self.recent_goals.append(self.target.copy())
            gain = ObservedGain(self.terrain)
            reachable = {key: nodes[key] for key in parent}
            groups = connected_clusters(reachable, graph, rates, uncertainty, self.rate_limit)
            visit_distance = visit_distance or distance_from_visits(nodes, graph, self.visited)
            summaries, candidates = [], []
            for group in groups:
                # One representative per homogeneous connected group, plus a
                # second distant representative for elongated regions.
                local = [key for key in group if (.25 if inspection else .75) <= length[key]
                           and (visit_distance[key] >= .65 or uncertainty[key] > .8 or
                                (inspection and visit_distance[key] >= .15 and gain.value(key) > 0) or
                                (self.objective == 'vertical' and new_height(key)) or
                                self.landmark_bonus(nodes[key]) > 0)]
                if not local:
                    continue
                # Bound visibility work but sample the entire group's ordering.
                local = sorted(local)
                target_local = sorted((key for key in local if self.landmark_bonus(nodes[key]) > 0), key=lambda key: length[key])[:8]
                remembered = [key for key in local if key in self.agenda and self.agenda[key]['status'] != 'visited'][:8]
                refined = [key for key in local if key in refinement_keys]
                height_representatives = []
                if self.objective == 'vertical':
                    bands = {}
                    for key in sorted(local, key=lambda key: length[key]):
                        if new_height(key):
                            bands.setdefault(round(float(nodes[key][2])/.2), key)
                    height_representatives = list(bands.values())[:8]
                local = sorted(set(local[::max(1, math.ceil(len(local)/8))]+target_local+remembered+refined+height_representatives))
                scored = []
                for key in local:
                    novelty = gain.value(key)
                    nearest_visit = float(visit_distance[key])
                    target_bonus = self.landmark_bonus(nodes[key])
                    if (nearest_visit < .65 and uncertainty[key] <= .8 and (not inspection or novelty == 0)
                            and target_bonus == 0 and not (self.objective == 'vertical' and new_height(key))):
                        continue
                    information = .5*math.log1p(uncertainty[key]**2/.02)
                    fresh_fraction = min(1., nearest_visit/.8)
                    score = (fresh_fraction*(1.+.8*novelty)+information+min(nearest_visit, 3.)+target_bonus)/(1.+.35*length[key]+5.*dose[key])
                    if inspection and novelty > 0:
                        # Discover new heights through observed connectors instead
                        # of endlessly revisiting a large, already sampled floor.
                        # This affects utility only; dose/return admission is intact.
                        bonus = self.terrain.acquisition_bonus(key, self.sampled_height_bands)
                        # Slopes are useful acquisition viewpoints while novel.
                        # A permanent connector reward otherwise draws the robot
                        # back down a visited ramp instead of onto a new landing.
                        score += fresh_fraction*bonus/(1.+.35*length[key]+5.*dose[key])
                    if admitted(key):
                        if self.utility_mode == 'information':
                            bonus = self.terrain.acquisition_bonus(key, self.sampled_height_bands) if inspection and novelty > 0 else 0.
                            score = information_score(key, novelty, fresh_fraction, target_bonus, bonus)
                        scored.append((score, key, novelty))
                if not scored:
                    continue
                scored.sort(key=lambda item: (self.objective == 'vertical' and new_height(item[1]), item[0], item[1]), reverse=True)
                chosen = scored[0]
                candidates.append(chosen)
                # Preserve a local representative even when this group's best
                # score belongs to a distant, persistent frontier (and vice versa).
                for band in (True, False):
                    alternative = next((item for item in scored if (length[item[1]] <= 6.) == band), None)
                    if alternative is not None and alternative != chosen:
                        candidates.append(alternative)
                for alternative in scored[1:]:
                    if math.dist(nodes[alternative[1]], nodes[chosen[1]]) > 1.5:
                        candidates.append(alternative)
                        break
                summaries.append(dict(anchor=nodes[chosen[1]].tolist(), size=len(group),
                    gain=chosen[2], upper_rate_usv_h=float(rates[chosen[1]]), score=float(chosen[0]),
                    sigma_log_rate=float(uncertainty[chosen[1]])))
            self.clusters = sorted(summaries, key=lambda item: -item['score'])[:128]
            self.cluster_stamp = stamp
            self.representatives_scored = len(gain.cache)
            self.candidate_count = len(candidates)
            heights = [item for item in candidates if new_height(item[1])] if self.objective == 'vertical' else []
            if self.objective == 'vertical':
                self.vertical_frontier.update(admitted_height_candidates=len(heights),
                    commitment_preempted=bool(keep_previous and heights))
            if keep_previous and not heights:
                selected, self.state, self.last_reason = previous, 'exploring', 'continue_revalidated_goal'
            elif not candidates:
                self.returning = True
                selected, self.state, self.last_reason = home, 'returning_no_admissible_frontier', 'no_admissible_region'
            else:
                nearby = [item for item in candidates if length[item[1]] <= 6.]
                inspections = [item for item in candidates if self.landmark_bonus(nodes[item[1]]) > 0]
                pool = heights or (inspections if self.objective == 'inspect' and inspections else nearby or candidates)
                if inspection and self.region_scheduling == 'hierarchical' and not heights and not (self.objective == 'inspect' and inspections):
                    _, selected, _ = self.region_agenda.choose(candidates, outbound_time, stamp,
                        inspection_keys={item[1] for item in inspections})
                else:
                    _, selected, _ = max(pool)
                self.state, self.last_reason = 'exploring', 'best_connected_region'
                if not nearby:
                    self.last_reason = 'reposition_to_distant_frontier'
                if self.objective == 'inspect' and inspections:
                    self.last_reason = 'inspect_observed_landmark'
                if inspection and length[selected] < .75:
                    self.last_reason = 'near_frontier_inspection'
                if heights:
                    self.last_reason = 'survey_observed_height_band'
                self.goal_started = stamp
        goal = nodes[selected].copy()
        if not self.returning:
            self.uncertainty_probe_target = goal.copy() if selected in uncertainty_probes else None
        if self.campaign_agenda is not None and not self.returning:
            self.campaign_agenda.begin(goal,len(self.observed_support_keys))
        if not self.returning and (self.goal_progress_focus is None or math.dist(goal,self.goal_progress_focus) > .75):
            self.goal_progress_focus = goal.copy()
            self.goal_progress_best = float(np.linalg.norm(np.asarray(position)-goal))
            self.goal_progress_stamp = stamp
            self.goal_progress_yaw = getattr(self,'start_yaw',None)
        if not self.returning and not keep_previous:
            self.committed_goal_rate = float(cost_rates[selected])
        if self.survey_until_complete and not self.returning and (self.target is None or math.dist(goal, self.target) > .1):
            region = tuple(np.round(goal/[.5, .5, .2]).astype(int))
            repeated_region = region in self.frontier_goal_history
            self.frontier_goal_history.append(region)
            if (self.last_support_progress_s is not None and stamp-self.last_support_progress_s > 180.
                    and repeated_region and len(self.frontier_goal_history) == 16 and len(set(self.frontier_goal_history)) <= 8):
                self.returning, self.last_reason, self.state = True, 'frontier_progress_stalled', 'returning_survey_limit'
                selected, goal = home, nodes[home].copy()
        self.route_forecast['checked_inspection_candidates'] = len(checked_inspections)
        self.route_forecast['checked_execution_candidates'] = len(checked_inspections)
        if self.target is None or math.dist(goal, self.target) > .35:
            self.goal_switches += 1
        self.target = goal
        self.selected_utility = utility_details.get(selected, {})
        if not self.returning and not keep_previous:
            self.committed_goal_cost = self.selected_utility.get('goal_normalized_cost',self.selected_utility.get('normalized_cost')) if self.exposure_policy=='pathwise' else self.selected_utility.get('normalized_cost')
        self._remember(selected, 'return_home' if self.returning else 'active', stamp)
        if not self.returning:
            self.route_forecast.update(outbound_duration_s=outbound_time[selected], outbound_upper_dose_usv=dose[selected],
                goal_return_duration_s=home_time[selected], goal_return_upper_dose_usv=home_dose[selected],
                goal_return_peak_rate_usv_h=home_peak[selected])
            if self.expected_dose_objective:
                self.route_forecast['outbound_expected_dose_usv'] = objective_dose[selected]
        self.path = (self.return_path[1:] if self.returning and selected==home and return_alternatives is not None else
                     frontier_paths[selected] if not self.returning and selected in frontier_paths else
                     [nodes[key].copy() for key in self.terrain.path(parent, selected)[1:]])
        if self.returning and self.survey_stop_reason is None:
            self.survey_stop_reason = self.last_reason
        if self.returning and selected==home and return_alternatives is not None:
            self.execution_route=return_alternatives['best']['motion']
            self.execution_route_raw=[point.copy() for point in self.path]
        elif self.exposure_policy == 'pathwise' and not self.returning:
            self.execution_route = self.refined_motions.get(selected)
            self.execution_route_raw = [point.copy() for point in self.path]
        if self.returning and not self.path and math.dist(position, self.home) >= self.home_tolerance:
            # A coarse graph cell can be reached before the tighter physical
            # home tolerance. Certify the final continuous approach explicitly.
            from .path_motion import RouteGeometry
            if RouteGeometry(self.terrain).line(np.asarray(position), self.home, margin=0.) is not None:
                self.path = [self.home.copy()]
                seconds, exposure = forecast_polyline([position, self.home], [rates[root], rates[root]],
                    start_yaw=getattr(self, 'start_yaw', None))
                self.route_forecast.update(return_duration_s=seconds, return_upper_dose_usv=exposure,
                                           return_forecast_geometry='checked_final_home_approach')
            else:
                self.state, self.last_reason = 'waiting_for_home_support', 'final_home_approach_unavailable'
        self.events.append(dict(time_s=stamp, state=self.state, reason=self.last_reason, target=goal.tolist(),
            predicted_outbound_dose_usv=dose[selected], return_reserve_usv=self.return_reserve,
            measured_dose_upper_usv=self.sample.dose_upper_usv, candidates=self.candidate_count))
        self.plan_ms.append((time.perf_counter()-started)*1000.)

    def metrics(self):
        result = super().metrics()
        result.update(planner='clustered' if self.commitment else 'clustered-no-commit',
            clusters=self.clusters, candidate_count=self.candidate_count, goal_switches=self.goal_switches,
            rejection_counts=self.rejections, decision_reason=self.last_reason,
            selected_target=None if self.target is None else self.target.tolist(),
            planned_path=[p.tolist() for p in self.path], return_path=[p.tolist() for p in self.return_path])
        result['cluster_stamp_s'] = self.cluster_stamp
        result['representatives_scored'] = self.representatives_scored
        result['supported_graph_nodes'] = len(self.terrain.nodes)
        result['campaign_agenda'] = self.campaign_agenda.metrics() if self.campaign_agenda is not None else None
        result['shared_routes'] = dict(recovered_routes=len(self.terrain.shared_routes),
            usable_graph_edges=len(self.terrain.shared_route_edges),
            scope='Prior recovered robot paths; current support, obstacle, radiation and return admission remain authoritative')
        result['route_experience'] = self.terrain.route_experience.metrics() if self.terrain.route_experience is not None else None
        if self.survey_stop_reason is not None:
            result['decision_reason'] = self.survey_stop_reason
            result['return_navigation_reason'] = self.last_reason
        if self.survey_until_complete:
            result.update(frontier_count=self.frontier_count,
                admissible_frontier_count=self.admissible_frontier_count,
                frontier_complete=self.frontier_complete,
                frontier_confirmation_count=self.frontier_confirmation_count,
                completion_rule='zero observed-map frontiers on three fresh scans')
            result['frontier_decision'] = self.frontier_decision
            result['last_support_progress_s'] = self.last_support_progress_s
            result['hot_goal_deferrals'] = self.hot_goal_deferrals
            result['connection_probe_count'] = self.connection_probe_count
            result['uncertainty_probe_count'] = self.uncertainty_probe_count
            result['budget_frontier_screening'] = dict(screened=len(self.budget_screened),
                additional_batches=self.budget_screen_cycles,maximum_additional_batches=3)
            result['unproductive_frontier_count'] = self.unproductive_frontier_count
            result['frontier_views'] = self.frontier_views.metrics() if self.frontier_views else None
            result['frontier_blocked_counts'] = dict(self.radiation_frontier_counts,
                execution_radiation_blocked=len(self.radiation_execution_blocked),
                execution_dose_blocked=len(self.survival_dose_rejections))
            result['radiation_exclusion'] = (self.radiation_exclusion.metrics() if self.radiation_exclusion else None)
            keys = [key for key in self.survey_status if key in self.terrain.nodes]
            result['survey_admission'] = dict(
                positions=[self.terrain.nodes[key].tolist() for key in keys],
                statuses=[self.survey_status[key] for key in keys],
                pending_viewpoints=self.survey_pending,
                scope='observed support with predicted route/rate/dose admission; unknown space is unclassified')
        result['blocked_edges'] = [[self.terrain.nodes[a].tolist(), self.terrain.nodes[b].tolist()]
                                   for a, b in self.blocked_edges if a in self.terrain.nodes and b in self.terrain.nodes]
        result['sampled_support_height_bands_m'] = [b*.2 for b in sorted(self.sampled_height_bands)]
        result['route_forecast'] = self.route_forecast
        result['exploration_agenda'] = dict(size=len(self.agenda), capacity=self.agenda_capacity,
            unrecorded_candidates=self.agenda_overflow, statuses=dict(Counter(t['status'] for t in self.agenda.values())),
            tasks=list(self.agenda.values())[:64])
        result['observed_support_visit_fraction'] = len(self.visited.intersection(self.terrain.nodes))/max(1, len(self.terrain.nodes))
        result['known_landmarks_pending'] = sum(not o.inspected for o in self.landmarks.values())
        result['mission_objective'] = self.objective
        if self.objective == 'vertical':
            result['vertical_frontier'] = dict(self.vertical_frontier)
        result['region_agenda'] = self.region_agenda.metrics()
        result['utility_mode'] = self.utility_mode
        result['selected_utility'] = self.selected_utility
        if self.objective == 'circuit':
            result['circuit'] = self.circuit.metrics()
            result['circuit_tour'] = self.circuit_tour
        return result
