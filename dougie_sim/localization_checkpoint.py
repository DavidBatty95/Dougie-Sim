"""Bounded home relocalization checkpoints using onboard estimator evidence only."""
import math

from .recovery import healthy_local_return


class LocalizationCheckpoints:
    def __init__(self, limit=0, wait_s=40.):
        if not isinstance(limit, int) or not 0 <= limit <= 3 or not math.isfinite(wait_s) or not 0 < wait_s <= 40.:
            raise ValueError('localization checkpoints must be between zero and three')
        self.limit, self.wait_s = limit, wait_s
        self.used = 0
        self.origin = self.started_s = None
        self.outcome = 'idle'

    def request(self, localization):
        if self.used < self.limit and self.origin is None:
            self.origin = (localization.get('map_revision', 0), localization.get('anchor_revalidations', 0))
            self.outcome = 'return_requested'

    def step(self, stamp, localization, at_home, admissible, reason):
        if self.origin is None:
            return False
        if reason != 'localization_return_reserve' or not admissible:
            self.origin = self.started_s = None
            self.outcome = 'not_admissible'
            return False
        if self.started_s is not None and stamp-self.started_s >= self.wait_s:
            self.origin = self.started_s = None
            self.outcome = 'revalidation_timeout'
            return False
        if not at_home:
            return False
        if self.started_s is None:
            self.started_s = stamp
        confirmed = (localization.get('map_revision', 0) > self.origin[0]
                     or localization.get('anchor_revalidations', 0) > self.origin[1])
        std = localization.get('position_std_m', [float('inf')])
        if (confirmed and len(std) == 3 and all(math.isfinite(s) and 0 <= s < .1 for s in std)
                and healthy_local_return(localization, stamp)):
            self.used += 1
            self.origin = self.started_s = None
            self.outcome = 'resumed'
            return True
        self.outcome = 'waiting_for_revalidation'
        return False

    @property
    def waiting(self):
        return self.origin is not None and self.started_s is not None

    def metrics(self):
        return dict(limit=self.limit, used=self.used, pending=self.origin is not None,
                    started_s=self.started_s, wait_limit_s=self.wait_s, outcome=self.outcome)
