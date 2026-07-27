import threading
import time
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import (
    PolicyInferenceCandidate,
    PolicyInferenceWorker,
)


def _candidate(value: float, obs_dim: int = 2, action_dim: int = 2):
    return PolicyInferenceCandidate(
        obs_dim=obs_dim,
        action_dim=action_dim,
        observation=(value,) * obs_dim,
    )


def _wait_for_result(worker: PolicyInferenceWorker, timeout: float = 1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        result = worker.take_result()
        if result is not None:
            return result
        time.sleep(0.005)
    pytest.fail("Timed out waiting for policy inference result")


def _wait_until(predicate, timeout: float = 1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail("Timed out waiting for worker state")


def test_worker_is_daemon_and_requests_results_are_immutable():
    worker = PolicyInferenceWorker(lambda obs: np.zeros((1, 2)))
    try:
        mutable_observation = [1.0, 1.0]
        candidate = PolicyInferenceCandidate(2, 2, mutable_observation)
        request = worker.submit([candidate], time.monotonic() + 1.0)
        mutable_observation[0] = 9.0
        result = _wait_for_result(worker)

        assert worker.thread.daemon
        assert isinstance(request.candidates, tuple)
        assert request.candidates[0].observation == (1.0, 1.0)
        assert isinstance(result.actions, tuple)
        with pytest.raises(FrozenInstanceError):
            request.sequence = 4
        with pytest.raises(FrozenInstanceError):
            result.actions = (1.0,)
    finally:
        assert worker.shutdown(1.0)


def test_candidate_fallback_and_output_normalization_run_in_worker():
    policy_thread = []

    def policy(obs):
        policy_thread.append(threading.current_thread().name)
        if obs.shape == (1, 2):
            raise ValueError("wrong candidate")
        return np.array([[1.0, 2.0, 3.0, 99.0]], dtype=np.float64)

    worker = PolicyInferenceWorker(policy)
    try:
        worker.submit(
            (_candidate(1.0), _candidate(2.0, obs_dim=3, action_dim=3)),
            time.monotonic() + 1.0,
        )
        result = _wait_for_result(worker)

        assert result.error is None
        assert result.obs_dim == 3
        assert result.action_dim == 3
        assert result.actions == (1.0, 2.0, 3.0)
        assert policy_thread == ["g1-policy-inference", "g1-policy-inference"]
    finally:
        worker.shutdown(1.0)


def test_single_flight_rejects_requests_until_result_is_consumed():
    entered = threading.Event()
    release = threading.Event()
    calls = []

    def policy(obs):
        calls.append(float(obs[0, 0]))
        if len(calls) == 1:
            entered.set()
            assert release.wait(1.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    try:
        deadline = time.monotonic() + 2.0
        worker.submit((_candidate(1.0),), deadline)
        assert entered.wait(1.0)
        assert worker.submit((_candidate(2.0),), deadline) is None
        assert worker.submit((_candidate(3.0),), deadline) is None
        release.set()

        result = _wait_for_result(worker)
        assert calls == [1.0]
        assert result.sequence == 1

        latest = worker.submit((_candidate(3.0),), deadline)
        assert latest is not None
        next_result = _wait_for_result(worker)
        assert calls == [1.0, 3.0]
        assert next_result.sequence == latest.sequence
    finally:
        release.set()
        worker.shutdown(1.0)


@pytest.mark.parametrize(
    "policy, expected_error",
    [
        (lambda obs: np.array([[np.nan, 0.0]]), "non-finite"),
        (lambda obs: np.array([[1.0]]), "expected 2"),
    ],
)
def test_invalid_outputs_fail_and_invalidate_generation(policy, expected_error):
    worker = PolicyInferenceWorker(policy)
    try:
        first = worker.submit((_candidate(1.0),), time.monotonic() + 1.0)
        result = _wait_for_result(worker)

        assert expected_error in result.error
        assert worker.diagnostics().failed == 1
        assert worker.diagnostics().invalidated == 1
        second = worker.submit((_candidate(2.0),), time.monotonic() + 1.0)
        assert second.generation == first.generation + 1
    finally:
        worker.shutdown(1.0)


def test_deadline_uses_monotonic_time_and_returns_no_action():
    release = threading.Event()

    def policy(obs):
        release.wait(1.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    try:
        request = worker.submit(
            (_candidate(1.0),), deadline=time.monotonic() + 0.02
        )
        _wait_until(lambda: worker.deadline_missed(), timeout=0.5)
        release.set()
        result = _wait_for_result(worker)

        assert result.sequence == request.sequence
        assert result.deadline_missed
        assert result.actions == ()
        diagnostics = worker.diagnostics()
        assert diagnostics.expired == 1
        assert diagnostics.invalidated == 1
    finally:
        release.set()
        worker.shutdown(1.0)


def test_invalidation_discards_active_result_and_shutdown_join_is_bounded():
    entered = threading.Event()
    release = threading.Event()

    def policy(obs):
        entered.set()
        release.wait(2.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    worker.submit((_candidate(1.0),), time.monotonic() + 2.0)
    assert entered.wait(1.0)
    worker.invalidate()

    started = time.monotonic()
    assert not worker.shutdown(0.02)
    assert time.monotonic() - started < 0.2
    assert worker.thread.daemon

    release.set()
    assert worker.shutdown(1.0)
    assert worker.take_result() is None
    assert worker.diagnostics().discarded == 1
