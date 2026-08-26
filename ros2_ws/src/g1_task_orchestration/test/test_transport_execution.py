import asyncio
import threading

from g1_msgs.action import TransportObject
from rclpy.action import GoalResponse

from g1_task_orchestration.mission_core import (
    PHASE_ACQUIRE,
    PHASE_APPROACH_DELIVERY,
    PHASE_APPROACH_PICKUP,
    PHASE_CARRY,
    PHASE_NAVIGATE_DELIVERY,
    PHASE_NAVIGATE_PICKUP,
    PHASE_PIN_BASE,
    PHASE_PICK,
    PHASE_PLACE,
    PHASE_RELEASE,
    PHASE_RETREAT_DELIVERY,
    PHASE_RETREAT_PICKUP,
    PHASE_TUCK,
    PHASE_UNPIN_BASE,
)
from g1_task_orchestration.transport_server import (
    ChildTerminationFailure,
    MissionFailure,
    TransportServer,
)


class _Logger:
    def warning(self, _):
        pass

    def error(self, _):
        pass


class _Handle:
    def __init__(self):
        self.request = TransportObject.Goal()
        self.request.object_id = "red_cube"
        self.request.surface_object_id = "drop_pad"
        self.request.arm = "right"
        self.request.pickup_staging_pose.header.frame_id = "map"
        self.request.delivery_staging_pose.header.frame_id = "map"
        self.is_cancel_requested = False
        self.status = None

    def publish_feedback(self, _):
        pass

    def succeed(self):
        self.status = "succeeded"

    def abort(self):
        self.status = "aborted"

    def canceled(self):
        self.status = "canceled"


def _server(fail_phase=None):
    server = object.__new__(TransportServer)
    server._navigation_timeout = 300.0
    server._approach_timeout = 60.0
    server._retreat_distance = 0.8
    server._retreat_timeout = 30.0
    server._approach_attempts = 2
    server._pick_attempts = 2
    server._busy = True
    server._recovery_required = False
    server._busy_lock = threading.Lock()
    server._posture = "posture"
    server._navigate = "navigate"
    server._approach = "approach"
    server._pick = "pick"
    server._retreat = "retreat"
    server._place = "place"
    server.events = []
    server.get_logger = lambda: _Logger()

    async def set_authority(enabled, phase):
        server.events.append((phase, enabled))

    async def set_base_pin(enabled, phase):
        server.events.append((phase, enabled))

    async def call_action(handle, client, goal, phase, timeout, check_success=True):
        del handle, client, goal, timeout, check_success
        server.events.append((phase, None))
        if phase == fail_phase:
            raise MissionFailure(phase, "injected failure")

    async def retry_action(attempts, *args, **kwargs):
        del attempts
        return await call_action(*args, **kwargs)

    async def clear():
        server.events.append(("clear", None))

    server._set_authority = set_authority
    server._set_base_pin = set_base_pin
    server._call_action = call_action
    server._retry_action = retry_action
    server._best_effort_clear = clear
    return server


def test_successful_mission_has_exclusive_sequential_order():
    server = _server()
    handle = _Handle()

    result = asyncio.run(server._execute(handle))

    assert result.success
    assert handle.status == "succeeded"
    assert server.events == [
        (PHASE_NAVIGATE_PICKUP, None),
        (PHASE_APPROACH_PICKUP, None),
        (PHASE_PIN_BASE, True),
        (PHASE_ACQUIRE, True),
        (PHASE_PICK, None),
        (PHASE_CARRY, None),
        (PHASE_UNPIN_BASE, False),
        (PHASE_RETREAT_PICKUP, None),
        ("clear", None),
        (PHASE_NAVIGATE_DELIVERY, None),
        (PHASE_APPROACH_DELIVERY, None),
        (PHASE_PIN_BASE, True),
        (PHASE_PLACE, None),
        (PHASE_TUCK, None),
        (PHASE_RELEASE, False),
        (PHASE_UNPIN_BASE, False),
        (PHASE_RETREAT_DELIVERY, None),
        ("clear", None),
    ]


def test_failure_after_pick_retains_authority_and_reports_possible_hold():
    server = _server(fail_phase=PHASE_CARRY)
    handle = _Handle()

    result = asyncio.run(server._execute(handle))

    assert not result.success
    assert result.failed_phase == PHASE_CARRY
    assert result.arm_authority_active
    assert result.object_may_be_held
    assert server._recovery_required
    assert (PHASE_RELEASE, False) not in server.events
    assert handle.status == "aborted"


def test_child_cancellation_waits_for_terminal_result():
    class ResultFuture:
        terminal = False

        def done(self):
            return self.terminal

    class Remote:
        async def cancel_goal_async(self):
            return type("Response", (), {"goals_canceling": [object()]})()

    server = object.__new__(TransportServer)
    server._cancel_timeout = 1.0
    result = ResultFuture()
    sleeps = 0

    async def sleep(_):
        nonlocal sleeps
        sleeps += 1
        result.terminal = True

    server._sleep = sleep
    asyncio.run(server._cancel_and_wait(Remote(), result, PHASE_PICK))
    assert sleeps == 1


def test_rejected_child_cancellation_is_an_unsettled_failure():
    class Remote:
        async def cancel_goal_async(self):
            return type("Response", (), {"goals_canceling": []})()

    server = object.__new__(TransportServer)
    server._cancel_timeout = 1.0
    result = type("ResultFuture", (), {"done": lambda self: False})()

    try:
        asyncio.run(server._cancel_and_wait(Remote(), result, PHASE_PICK))
    except ChildTerminationFailure as error:
        assert error.phase == PHASE_PICK
    else:
        raise AssertionError("rejected cancellation must fail the mission")


def test_recovery_latch_rejects_the_next_mission():
    server = object.__new__(TransportServer)
    server._simulation_enabled = True
    server._busy = False
    server._recovery_required = True
    server._busy_lock = threading.Lock()
    server.get_logger = lambda: _Logger()

    assert server._goal(_Handle().request) == GoalResponse.REJECT
    assert not server._busy
