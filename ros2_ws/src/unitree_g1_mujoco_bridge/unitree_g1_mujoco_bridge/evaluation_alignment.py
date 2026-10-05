"""Evaluation-only alignment of simulator snapshots; never a control input."""
from bisect import bisect_left
from collections import deque


class EvaluationAlignment:
    def __init__(self,max_gap=.2):
        self.max_gap = max_gap
        self.roots = deque(maxlen=64)
        self.people = deque(maxlen=64)
        self.last_root = self.last_people = None
        self.dropped = 0
        self.emitted = 0
        self.received_roots = 0
        self.epoch = 0

    def _reset(self):
        self.dropped += len(self.roots)
        self.epoch += 1
        self.roots.clear()
        self.people.clear()
        self.last_root = self.last_people = None

    def add_root(self,state):
        stamp = state['sim_time']
        if self.last_root is not None and stamp < self.last_root:
            self._reset()
        if self.last_root is None or stamp > self.last_root:
            if len(self.roots) == self.roots.maxlen:
                self.dropped += 1
            self.received_roots += 1
            self.roots.append(dict(state,root_sample_index=self.received_roots,epoch=self.epoch))
            self.last_root = stamp
        return self._drain()

    def add_people(self,stamp,population,people):
        if self.last_people is not None and stamp < self.last_people:
            self._reset()
        if self.last_people is None or stamp > self.last_people:
            self.people.append((stamp,population,{p['id']:dict(p) for p in people}))
            self.last_people = stamp
        return self._drain()

    def _drain(self):
        emitted = []
        while self.roots and self.people:
            root = self.roots[0]
            stamp = root['sim_time']
            if stamp > self.people[-1][0]:
                break  # Require a bracket; never extrapolate scoring positions.
            self.roots.popleft()
            index = bisect_left([p[0] for p in self.people],stamp)
            if index < len(self.people) and self.people[index][0] == stamp:
                _,population,people = self.people[index]
                span = 0.
                people = list(people.values())
            elif index == 0:
                self.dropped += 1
                continue
            else:
                left,right = self.people[index-1],self.people[index]
                span = right[0]-left[0]
                if span > self.max_gap or left[1] != right[1] or left[2].keys() != right[2].keys():
                    self.dropped += 1
                    continue
                fraction = (stamp-left[0])/span
                population = left[1]
                people = []
                for identity,a in left[2].items():
                    b = right[2][identity]
                    person = dict(id=identity,radius_m=max(a['radius_m'],b['radius_m']))
                    person.update({key:a[key]+fraction*(b[key]-a[key]) for key in ('x','y','vx','vy')})
                    people.append(person)
            self.emitted += 1
            emitted.append(dict(root,people=people,population_count=population,active_count=len(people),
                                interpolation_span_s=span,aligned_samples=self.emitted,dropped_unaligned_samples=self.dropped))
        return emitted
