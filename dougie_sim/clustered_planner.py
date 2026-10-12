"""Connected, risk-separated region candidates with persistent revalidated goals.

Research links and adaptation boundaries: docs/DOUGIE_ORI_IMPROVEMENTS.md.
All execution routes and dose admission use the fine observed graph, never a
centroid shortcut or an average region radiation value.
"""
import math
import time

import numpy as np

from .dose_planner import DoseExplorer
from .region_clustering import connected_clusters, distance_from_visits, ObservedGain


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

    def report_stall(self, position, stamp):
        root = self.terrain.nearest(position)
        next_point = next((p for p in self.path if math.dist(position, p) > .2), None)
        other = None if next_point is None else self.terrain.nearest(next_point)
        if root is not None and other in self.terrain.graph.get(root, []):
            self.blocked_edges[tuple(sorted((root, other)))] = stamp
        if self.target is not None:
            self.failed.append((self.target.copy(), stamp))
        self.path = []

    def observe_radiation(self, sample):
        super().observe_radiation(sample)
        if sample.rate_usv_h > self.rate_limit:
            self.last_reason = 'measured_rate_limit'
        elif sample.dose_upper_usv >= self.budget*.8:
            self.last_reason = 'measured_dose_reserve'

    def replan(self, position, stamp):
        started = time.perf_counter()
        nodes, graph = self.terrain.build_graph()
        self.blocked_edges = {edge: t for edge, t in self.blocked_edges.items() if stamp-t < 60.}
        for a, b in self.blocked_edges:
            if a in graph and b in graph[a]:
                graph[a].remove(b)
            if b in graph and a in graph[b]:
                graph[b].remove(a)
        root = self.terrain.nearest(position)
        self.rejections = dict(distance=0, failed=0, dose=0, rate=0, time=0)
        if root is None or self.sample is None or not 0. <= stamp-self.sample.stamp_s <= 1.5:
            self.path = []
            self.state = 'waiting_for_fresh_sensors_or_support'
            return
        if self.home is None:
            self.home = np.asarray(position).copy()
        home = self.terrain.nearest(self.home)
        if home is None:
            self.path = []
            self.state = 'no_observed_return_route'
            return
        self.visited.add(root)
        keys = list(nodes)
        _, sigma, upper = self.gp.predict([nodes[key]+[0, 0, .44] for key in keys])
        rates, uncertainty = dict(zip(keys, upper)), dict(zip(keys, sigma))
        _, dose, length, parent = self.terrain.routes(root, rates)
        _, home_dose, home_length, _ = self.terrain.routes(home, rates)
        if home not in parent:
            self.path = []
            self.state = 'no_observed_return_route'
            return
        self.return_path = [nodes[key].copy() for key in self.terrain.path(parent, home)]
        self.return_reserve = dose[home]+self.budget*.15+rates[root]*3./3600.
        if self.sample.dose_upper_usv+self.return_reserve >= self.budget:
            self.returning, self.last_reason = True, 'dose_return_reserve'
        if stamp+length[home]/.06+15. >= self.mission_duration:
            self.returning, self.last_reason = True, 'time_return_reserve'
        self.failed = [(p, t) for p, t in self.failed if stamp-t < 60.]

        def admitted(key):
            if key not in parent or key not in home_dose:
                return False
            if any(math.dist(nodes[key], p) < .65 for p, _ in self.failed):
                self.rejections['failed'] += 1
                return False
            total = self.sample.dose_upper_usv+dose[key]+home_dose[key]+self.budget*.15+rates[key]*5./3600.
            if total > self.budget:
                self.rejections['dose'] += 1
                return False
            if any(rates[p] > self.rate_limit for p in self.terrain.path(parent, key)):
                self.rejections['rate'] += 1
                return False
            if stamp+(length[key]+home_length[key])/.06+15. > self.mission_duration:
                self.rejections['time'] += 1
                return False
            return True

        previous = None if self.target is None else self.terrain.nearest(self.target)
        selected = None
        if self.returning:
            selected, self.state = home, 'returning'
            if math.dist(position, self.home) < .4:
                self.path, self.state = [], 'returned'
                return
        elif (self.commitment and previous is not None and math.dist(nodes[previous], self.target) < .25
              and math.dist(position, self.target) > .4 and stamp-self.goal_started < 45. and admitted(previous)):
            selected, self.state, self.last_reason = previous, 'exploring', 'continue_revalidated_goal'
        else:
            if self.target is not None and math.dist(position, self.target) < .5:
                self.recent_goals.append(self.target.copy())
            gain = ObservedGain(self.terrain)
            reachable = {key: nodes[key] for key in parent}
            groups = connected_clusters(reachable, graph, rates, uncertainty, self.rate_limit)
            visit_distance = distance_from_visits(nodes, graph, self.visited)
            summaries, candidates = [], []
            for group in groups:
                # One representative per homogeneous connected group, plus a
                # second distant representative for elongated regions.
                local = [key for key in group if .75 <= length[key] <= 6.
                         and (visit_distance[key] >= .65 or uncertainty[key] > .8)]
                if not local:
                    continue
                # Bound visibility work but sample the entire group's ordering.
                local = sorted(local)
                local = local[::max(1, math.ceil(len(local)/8))]
                scored = []
                for key in local:
                    novelty = gain.value(key)
                    nearest_visit = float(visit_distance[key])
                    information = .5*math.log1p(uncertainty[key]**2/.02)
                    fresh_fraction = min(1., nearest_visit/.8)
                    score = (fresh_fraction*(1.+.8*novelty)+information+min(nearest_visit, 3.))/(1.+.35*length[key]+5.*dose[key])
                    if admitted(key):
                        scored.append((score, key, novelty))
                if not scored:
                    continue
                scored.sort(reverse=True)
                chosen = scored[0]
                candidates.append(chosen)
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
            if not candidates:
                self.returning = True
                selected, self.state, self.last_reason = home, 'returning_no_admissible_frontier', 'no_admissible_region'
            else:
                _, selected, _ = max(candidates)
                self.state, self.last_reason = 'exploring', 'best_connected_region'
                self.goal_started = stamp
        goal = nodes[selected].copy()
        if self.target is None or math.dist(goal, self.target) > .35:
            self.goal_switches += 1
        self.target = goal
        self.path = [nodes[key].copy() for key in self.terrain.path(parent, selected)[1:]]
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
        result['blocked_edges'] = [[self.terrain.nodes[a].tolist(), self.terrain.nodes[b].tolist()]
                                  for a, b in self.blocked_edges if a in self.terrain.nodes and b in self.terrain.nodes]
        return result
