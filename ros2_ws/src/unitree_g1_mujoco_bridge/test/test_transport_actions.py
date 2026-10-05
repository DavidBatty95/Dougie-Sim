from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.transport_actions import ManagedAction,ActionTerminationError


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class Fixture:
    def __init__(self):
        self.now = 0.
        self.result = Future()
        self.cancel = completed(SimpleNamespace(return_code=0))
        self.cancels = 0
        self.events = []
        self.handle = SimpleNamespace(accepted=True,get_result_async=lambda:self.result,
                                      cancel_goal_async=self.cancel_goal)
        self.sent = completed(self.handle)
        self.on_pump = lambda:None

    def cancel_goal(self):
        self.cancels += 1
        return self.cancel

    def pump(self,duration):
        self.now += duration
        self.on_pump()

    def action(self,wait=None):
        return ManagedAction('arm',self.sent,wait or (lambda f:f.result()),self.pump,
                             lambda:True,lambda **kw:self.events.append(kw),
                             timeout=.2,clock=lambda:self.now)


def test_deadline_failure_waits_for_terminal_not_just_cancel_ack():
    f = Fixture()
    def terminal_later():
        if f.now >= .15 and not f.result.done():
            f.result.set_result(SimpleNamespace(status=5))
    f.on_pump = terminal_later
    with pytest.raises(RuntimeError,match='mission deadline'):
        with f.action():
            raise RuntimeError('mission deadline exceeded')
    assert f.cancels == 1 and f.now >= .15
    assert f.events[-1]['terminal_status'] == 5


def test_cancel_ack_without_terminal_result_is_not_safe_completion():
    f = Fixture()
    with pytest.raises(ActionTerminationError,match='terminal state unknown'):
        with f.action():
            pass
    assert f.now == pytest.approx(.2)


def test_late_goal_acceptance_after_mission_timeout_is_canceled():
    f = Fixture()
    f.sent = Future()
    def progress():
        if not f.sent.done():
            f.sent.set_result(f.handle)
        elif f.cancels and not f.result.done():
            f.result.set_result(SimpleNamespace(status=5))
    f.on_pump = progress
    def expired(future):
        raise RuntimeError('mission deadline exceeded')
    with pytest.raises(RuntimeError,match='mission deadline'):
        with f.action(expired):
            pytest.fail('must not execute action body')
    assert f.cancels == 1


def test_lost_goal_acknowledgement_is_reported_as_unknown():
    f = Fixture()
    f.sent = Future()
    def expired(future):
        raise RuntimeError('mission deadline exceeded')
    with pytest.raises(ActionTerminationError):
        with f.action(expired):
            pass
    assert f.cancels == 0


@pytest.mark.parametrize('cancel_code',[0,1,2,3])
def test_terminal_race_is_decisive_even_if_cancel_is_rejected(cancel_code):
    f = Fixture()
    f.cancel = completed(SimpleNamespace(return_code=cancel_code))
    f.on_pump = lambda:f.result.set_result(SimpleNamespace(status=4))
    with f.action():
        pass
    assert f.events[-1]['terminal_status'] == 4


def test_lost_cancel_response_does_not_block_observed_terminal_result():
    f = Fixture()
    f.cancel = Future()
    f.on_pump = lambda:f.result.set_result(SimpleNamespace(status=5))
    with f.action():
        pass
    assert f.events[-1]['terminal_status'] == 5


@pytest.mark.parametrize('status',[4,5,6])
def test_already_terminal_action_does_not_request_cancellation(status):
    f = Fixture()
    f.result.set_result(SimpleNamespace(status=status))
    with f.action():
        pass
    assert f.cancels == 0


def test_nonterminal_result_is_rejected():
    f = Fixture()
    f.result.set_result(SimpleNamespace(status=2))
    with pytest.raises(ActionTerminationError,match='nonterminal'):
        with f.action():
            pass


def test_rejected_goal_does_not_cancel_unowned_motion():
    f = Fixture()
    f.handle.accepted = False
    with pytest.raises(RuntimeError,match='goal rejected'):
        with f.action():
            pass
    assert f.cancels == 0
