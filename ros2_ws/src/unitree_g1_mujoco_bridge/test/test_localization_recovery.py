from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.localization_recovery import LocalizationInterrupted,recover_localization
from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError


def test_cleanup_precedes_recovery_and_retry_keeps_original_goal_and_deadline(monkeypatch):
    clock = [0.]
    monkeypatch.setattr('unitree_g1_mujoco_bridge.localization_recovery.time.monotonic',lambda:clock[0])
    calls = []
    def recover(deadline,attempt):
        assert calls[-2:] == ['terminated','stop']
        assert deadline==10. and attempt==1
        clock[0] = 3.
    mission = SimpleNamespace(stop_intent=lambda:calls.append('stop'),wait_localization=recover)
    @recover_localization
    def motion(self,goal,timeout=10.):
        calls.append((goal,timeout))
        if len(calls)==1:
            try:
                raise LocalizationInterrupted('stale')
            finally:
                calls.append('terminated')
        return 'arrived'
    assert motion(mission,(1.,2.,3.)) == 'arrived'
    assert calls[-1] == ((1.,2.,3.),7.)


def test_unknown_termination_is_not_retried():
    @recover_localization
    def motion(self,timeout=10.):
        raise ActionTerminationError('unknown')
    with pytest.raises(ActionTerminationError):
        motion(SimpleNamespace())


def test_localization_retries_are_bounded():
    attempts = []
    @recover_localization
    def motion(self,timeout=10.):
        attempts.append(1)
        raise LocalizationInterrupted('stale')
    mission = SimpleNamespace(stop_intent=lambda:None,wait_localization=lambda *args:None)
    with pytest.raises(LocalizationInterrupted):
        motion(mission)
    assert len(attempts)==3
