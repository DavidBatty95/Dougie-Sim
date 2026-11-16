"""Marginal value of repeated sensed views, independent of radiation and truth."""
from collections import OrderedDict
import math

import numpy as np


class FrontierViews:
    """Remember unresolved support seen repeatedly from the same approach.

    An unknown cell remains a frontier. This only reduces acquisition reward for
    a view already tried on three distinct fresh scans. Other approach directions
    and distance bands remain candidates. A return with unknowns is incomplete.
    """
    def __init__(self, resolution, scans=3, capacity=32768):
        self.resolution, self.scans, self.capacity = resolution, scans, capacity
        self.views = OrderedDict()
        self.last_scan = None

    def signature(self,cell,position):
        delta = (np.asarray(cell[:2],dtype=float)+.5)*self.resolution-np.asarray(position)[:2]
        angle = math.atan2(delta[1],delta[0])
        sector = int(math.floor((angle+math.pi)/(math.pi/4.)))%8
        distance = int(math.floor(float(np.linalg.norm(delta))/.5))
        return (*cell,sector,distance)

    def observe(self,cells,position,scan):
        if scan == self.last_scan:
            return
        self.last_scan = scan
        for cell in cells:
            key = self.signature(cell,position)
            count = self.views.pop(key,0)
            self.views[key] = min(self.scans,count+1)
        while len(self.views) > self.capacity:
            self.views.popitem(last=False)

    def marginal_gain(self,cells,position):
        return sum(self.views.get(self.signature(cell,position),0) < self.scans for cell in cells)

    def metrics(self):
        return dict(view_signatures=len(self.views),capacity=self.capacity,
            repeated_unresolved_views=sum(n>=self.scans for n in self.views.values()),
            scans_required=self.scans,scope='onboard unknown-support views; no evaluator coverage or radiation information')

    def export(self):
        return [[*key,value] for key,value in self.views.items()]

    def load(self,rows):
        self.views=OrderedDict((tuple(map(int,row[:-1])),int(row[-1])) for row in rows[-self.capacity:])
        self.last_scan=None
