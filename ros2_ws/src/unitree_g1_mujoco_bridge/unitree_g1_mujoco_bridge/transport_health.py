"""Latched transport scoring coverage; receipt freshness is not sensor accuracy."""
import math
import time


class EvaluationHealth:
    def __init__(self,required,max_gap=1.,clock=time.monotonic):
        self.required = set(required)
        self.max_gap,self.clock = max_gap,clock
        self.updated,self.stamps,self.max_gaps = {},{},{}
        self.failures = set()
        self.started = False

    def fail(self,reason):
        if self.started:
            self.failures.add(reason)

    def start(self):
        if not self.started:
            self.started = True
            self.check()

    def observe(self,key,stamp=None):
        now = self.clock()
        if stamp is not None:
            if not math.isfinite(stamp):
                self.fail(f'{key}: nonfinite acquisition stamp')
                return False
            previous = self.stamps.get(key)
            if previous is not None and stamp <= previous:
                if stamp < previous:
                    self.fail(f'{key}: acquisition clock regressed')
                return False  # Repeated samples cannot refresh coverage.
            if previous is not None and self.started and stamp-previous > self.max_gap:
                self.fail(f'{key}: acquisition gap exceeded')
            self.stamps[key] = stamp
        if key in self.updated and self.started:
            gap = now-self.updated[key]
            self.max_gaps[key] = max(gap,self.max_gaps.get(key,0.))
            if gap > self.max_gap:
                self.fail(f'{key}: receipt gap exceeded')
        self.updated[key] = now
        return True

    def check(self):
        if not self.started:
            return
        now = self.clock()
        for key in self.required:
            if key not in self.updated:
                self.fail(f'{key}: missing stream')
            elif now-self.updated[key] > self.max_gap:
                self.fail(f'{key}: receipt gap exceeded')

    def report(self):
        self.check()
        now = self.clock()
        return dict(valid=self.started and not self.failures,started=self.started,
                    failures=sorted(self.failures),max_gap_s=self.max_gap,
                    receipt_age_s={key:now-self.updated[key] if key in self.updated else None
                                   for key in sorted(self.required)},
                    max_receipt_gap_s=dict(self.max_gaps),
                    object_timestamp_scope='ROS IO receipt; native object packets lack acquisition stamps')
