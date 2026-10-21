"""Bounded recovery accounting from onboard dose, freshness and route evidence."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RecoveryDecision:
    phase: str
    force_return: bool = False
    terminal: bool = False


class BudgetedRecovery:
    def __init__(self, resense_s=3., max_wait_s=12., max_episode_s=30.):
        self.resense_s, self.max_wait_s, self.max_episode_s = resense_s, max_wait_s, max_episode_s
        self.started = None
        self.started_dose = 0.
        self.last_dose = None
        self.measured_dose = 0.
        self.episodes = self.attempts = 0
        self.last_attempt = -math.inf
        self.reason = None
        self.phase = 'idle'
        self.wait_upper_dose = 0.
        self.terminal = False

    def step(self, reason, stamp, measured_dose, dose_upper, rate_upper, budget, reserve,
             deadline, return_seconds=0., valid_route=False):
        values = (stamp, measured_dose, dose_upper, rate_upper, budget, reserve, deadline, return_seconds)
        if not all(math.isfinite(v) for v in values) or min(measured_dose, dose_upper, rate_upper, reserve, return_seconds) < 0 or budget <= 0:
            raise ValueError('invalid recovery forecast')
        if self.started is not None and self.last_dose is not None:
            self.measured_dose += max(0., measured_dose-self.last_dose)
        self.last_dose = measured_dose
        if self.terminal:
            return RecoveryDecision('stopped_unrecoverable', True, True)
        if reason is None and self.started is None:
            return RecoveryDecision('idle')
        if self.started is None:
            self.started, self.started_dose = stamp, measured_dose
            self.episodes += 1
            self.attempts, self.last_attempt = 0, -math.inf
        elapsed = stamp-self.started
        if elapsed < 0:
            raise ValueError('recovery time reversed')
        if reason is None and valid_route and elapsed >= self.resense_s:
            self.started, self.phase, self.reason = None, 'idle', None
            return RecoveryDecision('recovered')
        self.reason = reason or self.reason
        self.wait_upper_dose = rate_upper*self.resense_s/3600.
        exhausted = (dose_upper+reserve+self.wait_upper_dose >= budget
                     or measured_dose-self.started_dose >= .02*budget
                     or stamp+return_seconds+self.resense_s+15. >= deadline)
        if elapsed >= self.max_episode_s or (elapsed >= self.max_wait_s and not valid_route):
            self.phase, self.terminal = 'stopped_unrecoverable', True
        elif exhausted:
            self.phase = 'return_now' if valid_route else 'stopped_unrecoverable'
            self.terminal = not valid_route
        elif elapsed < self.resense_s:
            self.phase = 'resensing'
        else:
            self.phase = 'checked_retreat'
        return RecoveryDecision(self.phase, self.phase in ('return_now', 'checked_retreat'), self.terminal)

    def allow_attempt(self, stamp):
        if self.attempts >= 2 or stamp-self.last_attempt < 6.:
            return False
        self.attempts += 1
        self.last_attempt = stamp
        return True

    def metrics(self):
        return dict(phase=self.phase, reason=self.reason, terminal=self.terminal, episodes=self.episodes,
            episode_start_s=self.started, retreat_attempts=self.attempts,
            measured_recovery_dose_usv=self.measured_dose, next_wait_upper_dose_usv=self.wait_upper_dose,
            max_wait_s=self.max_wait_s, max_episode_s=self.max_episode_s)


def failure_reason(explorer, packet, stamp, localization, stalled, execution_status):
    if localization:
        if (not localization.get('initialized', False) or localization.get('rejection_streak', 0) >= 5
                or max(localization.get('position_std_m', [0.])) > .3):
            return 'localization_unreliable'
    if explorer.sample is None or not 0. <= stamp-explorer.sample.stamp_s <= 1.5:
        return 'stale_radiation'
    if not 0. <= stamp-packet.stamp_s <= .75:
        return 'stale_range'
    if explorer.state in ('waiting_for_support', 'waiting_for_fresh_sensors_or_support'):
        return 'missing_local_support'
    if explorer.state == 'waiting_for_task_observations':
        return 'task_support_unavailable'
    if explorer.state in ('no_observed_return_route', 'waiting_for_home_support'):
        return 'return_route_disconnected'
    if stalled and explorer.path:
        return 'motion_stalled'
    if execution_status == 'waiting_for_route_reacquisition' and explorer.path:
        return 'route_tracking_lost'
    return None
