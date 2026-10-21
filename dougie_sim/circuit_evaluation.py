"""Independent physical witnesses for initially observed circuit goals.

The observation-to-physics frame transform is evaluator-only and is never used
to choose goals or feed localization, mapping, GP inference or route control.
"""
import numpy as np


class CircuitWitness:
    def __init__(self):
        self.goals = None
        self.dwell = np.zeros(4)
        self.reached = [None]*4
        self.frame_missing = False

    def set_goals(self, circuit, frame):
        if self.goals is not None or not circuit.get('initial_goals'):
            return
        if frame is None:
            self.frame_missing = True
            return
        R, t = frame
        goals = np.asarray(circuit['initial_goals'], dtype=float)
        if goals.shape != (4, 3) or not np.isfinite(goals).all():
            raise ValueError('invalid circuit evaluator goals')
        self.goals = goals@R.T+t

    def tick(self, physical_support_position, stamp, dt):
        if self.goals is None:
            return
        close = np.linalg.norm(self.goals-physical_support_position, axis=1) <= .35
        self.dwell = np.where(close, self.dwell+dt, 0.)
        for index in range(4):
            if self.dwell[index] >= .2 and self.reached[index] is None:
                self.reached[index] = stamp

    def metrics(self):
        return dict(physical_goals=None if self.goals is None else self.goals.tolist(),
            reached_s=self.reached, all_goals_reached=bool(self.goals is not None and all(t is not None for t in self.reached)),
            initial_frame_unavailable=self.frame_missing,
            scope='evaluator-only physical support position within .35 m for .2 s at each initially observed goal')
