"""Four frozen supported goals selected only from the initial observed graph.

Provides comparable completed work for planning experiments without a building
map or hidden landmark catalogue. Goal order remains a planner decision.
"""
import math

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

    @property
    def completed(self):
        return len(self.goals) == 4 and all(g['reached_s'] is not None for g in self.goals)

    def metrics(self):
        return dict(goals=self.goals, initial_goals=self.initial_goals, observed_s=self.observed_s,
            capture_sequence=self.capture_sequence, completed=self.completed, radius_m=self.radius,
            scope='four initially observed supported goals; completion still requires independent physical witnesses and home return')
