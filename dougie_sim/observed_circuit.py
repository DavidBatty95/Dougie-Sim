"""Four frozen supported goals selected only from the initial observed graph.

Provides comparable completed work for planning experiments without a building
map or hidden landmark catalogue. Goal order remains a planner decision.
"""
import math
import itertools

import numpy as np


class ObservedCircuit:
    def __init__(self, radius=1.5):
        if not math.isfinite(radius) or not 1. <= radius <= 3.:
            raise ValueError('circuit radius must be between 1 and 3 metres')
        self.radius = radius
        self.goals = []
        self.initial_goals = []
        self.observed_s = None
        self.capture_sequence = None

    def prepare(self, terrain, root, home, stamp, capture_sequence):
        if self.goals:
            return True
        _, _, distance, parent = terrain.routes(root)  # geometry only; no field input
        back = terrain.routes(root, reverse=True)[3]
        pool = [k for k in parent if k in back and .6 <= distance[k] <= 2.*self.radius+1.
                and abs(terrain.nodes[k][2]-home[2]) < .08]
        chosen = []
        for direction in ((1., 0., 0.), (0., 1., 0.), (-1., 0., 0.), (0., -1., 0.)):
            desired = home+self.radius*np.asarray(direction)
            candidates = [k for k in pool if np.linalg.norm(terrain.nodes[k]-desired) <= .65
                          and all(np.linalg.norm(terrain.nodes[k]-point) >= .75 for point in chosen)]
            if not candidates:
                return False
            key = min(candidates, key=lambda k: (float(np.linalg.norm(terrain.nodes[k]-desired)), distance[k], k))
            chosen.append(terrain.nodes[key].copy())
        self.goals = [dict(identifier=i, position=point.tolist(), reached_s=None) for i, point in enumerate(chosen)]
        self.initial_goals = [point.tolist() for point in chosen]
        self.observed_s, self.capture_sequence = stamp, capture_sequence
        return True

    def observe_position(self, point, stamp):
        for goal in self.goals:
            if goal['reached_s'] is None and np.linalg.norm(np.asarray(goal['position'])-point) < .22:
                goal['reached_s'] = stamp

    def pending(self, terrain):
        result = []
        for goal in self.goals:
            if goal['reached_s'] is not None:
                continue
            key = terrain.nearest(goal['position'])
            if key is not None and np.linalg.norm(terrain.nodes[key]-goal['position']) < .25:
                result.append(key)
        return result

    def correct(self, event):
        for goal in self.goals:
            goal['position'] = event.point(goal['position'], self.observed_s).tolist()

    def best_tour(self, terrain, root, home, position, home_position, rates, gp, rate_limit,
                  admitted_first, remaining_time, remaining_budget, smoother, yaw=None, wait_s=3., recovery_s=3.):
        """Bounded whole-task lookahead: <=24 orders of <=4 observed goals.

        All remaining goals are required work, so compare complete tour costs
        rather than rewarding the information at one endpoint in isolation.
        Only the first leg is dispatched; every subsequent leg is replanned.
        """
        from .path_motion import RouteGeometry, RouteSmoother, path_length
        from .route_forecast import forecast_polyline
        pending = self.pending(terrain)
        if len(pending) != sum(g['reached_s'] is None for g in self.goals) or not pending:
            return None
        smoother = smoother or RouteSmoother(terrain, gp, rate_limit)
        geometry = RouteGeometry(terrain)
        legs = {}
        for source in set([root, *pending]):
            parent = terrain.routes(source, rates, max_rate=rate_limit)[3]
            for destination in set([home, *pending]):
                if destination == source or destination not in parent:
                    continue
                start = np.asarray(position) if source == root else terrain.nodes[source]
                raw = [start, *[terrain.nodes[k] for k in terrain.path(parent, destination)[1:]]]
                if destination == home and geometry.line(raw[-1], home_position, margin=0.) is not None:
                    raw.append(np.asarray(home_position))
                checked = smoother(raw)
                if len(checked.points) < 2:
                    continue
                points = np.asarray(checked.points)
                _, _, upper = gp.predict(points+[0., 0., .44])
                if np.max(upper) <= rate_limit:
                    legs[source, destination] = points, np.asarray(upper)
        best = None
        for order in itertools.permutations(pending):
            if order[0] not in admitted_first:
                continue
            edges = list(zip((root, *order), (*order, home)))
            if any(edge not in legs for edge in edges):
                continue
            points = np.concatenate([legs[edge][0] if i == 0 else legs[edge][0][1:] for i, edge in enumerate(edges)])
            upper = np.concatenate([legs[edge][1] if i == 0 else legs[edge][1][1:] for i, edge in enumerate(edges)])
            seconds, dose = forecast_polyline(points, upper, start_yaw=yaw)
            seconds += 5.*len(order)+(len(order)+1)*wait_s+recovery_s
            dose += (sum(rates[k]*(5.+wait_s) for k in order)+rates[root]*(wait_s+recovery_s))/3600.
            cost = seconds/max(remaining_time, 1.)+2.*dose/max(remaining_budget, 1e-6)
            candidate = dict(first=order[0], order=order, duration_s=seconds, upper_dose_usv=dose,
                             length_m=path_length(points), normalized_cost=cost)
            if best is None or (cost, order) < (best['normalized_cost'], best['order']):
                best = candidate
        return best

    @property
    def completed(self):
        return len(self.goals) == 4 and all(g['reached_s'] is not None for g in self.goals)

    def metrics(self):
        return dict(goals=self.goals, initial_goals=self.initial_goals, observed_s=self.observed_s,
            capture_sequence=self.capture_sequence, completed=self.completed, radius_m=self.radius,
            scope='four initially observed supported goals; completion still requires independent physical witnesses and home return')
