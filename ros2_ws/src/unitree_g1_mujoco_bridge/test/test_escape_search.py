from dataclasses import FrozenInstanceError,replace
import json
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Bool,String

from unitree_g1_mujoco_bridge.escape_search import (
    EscapeSnapshot,EscapeSearchWorker,SearchResult,checked_escape,search_escape,stopped_tail,
)
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.prediction_risk import prepare_predictions
from unitree_g1_mujoco_bridge.human_collision_avoidance import HumanCollisionAvoidance


def snapshot(seconds=10.):
    predictions = prepare_predictions([[(i*.1,1.8-i*.1,0.,0.,0.,1.) for i in range(31)]],.1)
    return EscapeSnapshot((0.,0.,0.),seconds,1.,3.,
        MotionEnvelope(max_x=.65,max_y=.45,accel_x=1.2,accel_y=1.2),
        (0.,0.,0.),(0.,0.,0.),predictions,seconds,(),.48,2.4,.1,0.,.4,.8,.8,(0.,0.,0.))


def wait_done(worker):
    end = time.monotonic()+2.
    while not worker.future.done():
        assert time.monotonic()<end
        time.sleep(.001)


def test_search_proposes_a_checked_escape_and_static_intrusion_rejects_it():
    context = snapshot()
    proposed = search_escape(context)
    assert proposed is not None
    assert checked_escape(context,proposed,stopped_tail(context)) is not None
    blocked = replace(context,obstacles=((0.,0.),))
    assert search_escape(blocked) is None
    assert checked_escape(blocked,proposed,stopped_tail(blocked)) is None
    with pytest.raises(FrozenInstanceError):
        context.pose = (1.,0.,0.)


def test_slow_search_does_not_block_poll_queue_work_or_survive_an_epoch_change():
    entered,release = threading.Event(),threading.Event()
    def delayed(context):
        entered.set()
        assert release.wait(2.)
        return (.3,.3,0.)
    worker = EscapeSearchWorker(delayed)
    try:
        assert worker.submit(snapshot())
        assert entered.wait(1.)
        start = time.monotonic()
        assert worker.poll(.2) is None
        assert time.monotonic()-start < .1
        assert not worker.submit(snapshot())
        assert worker.counters['submitted'] == 1
        worker.invalidate()
        release.set()
        wait_done(worker)
        assert worker.poll(.5) is None
        assert worker.counters['discarded'] == 1
        assert worker.submit(snapshot())
        wait_done(worker)
        assert worker.poll(.5).generation == worker.generation
    finally:
        release.set()
        worker.shutdown()
    assert not worker.submit(snapshot())


@pytest.mark.parametrize('now',[10.3,9.9])
def test_old_or_clock_regressed_results_are_discarded(now):
    clock = [10.]
    worker = EscapeSearchWorker(lambda context:(.3,.3,0.),clock=lambda:clock[0])
    try:
        worker.submit(snapshot())
        wait_done(worker)
        clock[0] = now
        assert worker.poll(.2) is None
        assert worker.counters['expired'] == 1
    finally:
        worker.shutdown()


def test_worker_failure_is_reported_without_escaping_into_control_thread():
    def fail(context):
        raise ValueError('search fixture failed')
    worker = EscapeSearchWorker(fail)
    try:
        worker.submit(snapshot())
        wait_done(worker)
        assert worker.poll(.2) is None
        assert worker.counters['failed'] == 1
        assert worker.last_error == 'search fixture failed'
    finally:
        worker.shutdown()


def test_no_feasible_proposal_is_distinct_from_pending_or_failed_work():
    worker = EscapeSearchWorker(lambda context:None)
    try:
        worker.submit(snapshot())
        wait_done(worker)
        result = worker.poll(.2)
        assert result is not None and result.command is None
        assert worker.counters['no_candidate'] == 1
        assert worker.counters['failed'] == worker.counters['proposed'] == 0
    finally:
        worker.shutdown()


@pytest.fixture
def supervisor():
    rclpy.init()
    node = HumanCollisionAvoidance()
    node.motion_envelope = snapshot().envelope
    node.async_predictive_escape_search = True
    node.require_command_envelope = True
    node.prediction_uncertainty_scale = 0.
    node.scan_frame = node.robot_frame
    now = node.get_clock().now().nanoseconds*1e-9
    node.scan_stamp = node.prediction_epoch = node.odom_stamp = now
    node.latest_predictions = snapshot().predictions
    node.scan_obstacles = []
    yield node
    node.destroy_node()
    rclpy.shutdown()


def completed_worker(node):
    seconds = node.get_clock().now().nanoseconds*1e-9
    context = node._escape_snapshot((0.,0.,0.),seconds,1.,(0.,0.,0.),.8,(0.,0.,0.))
    command = search_escape(context)
    assert command is not None
    result = SearchResult(0,time.monotonic(),time.monotonic(),context,command)
    results = [result]
    worker = SimpleNamespace(generation=0,poll=lambda age:results.pop() if results else None,
                            submit=lambda context:False,shutdown=lambda:None)
    worker.invalidate = lambda:setattr(worker,'generation',worker.generation+1)
    node.escape_worker = worker
    return context,command


def test_completed_proposal_is_revalidated_against_new_static_geometry(supervisor):
    context,command = completed_worker(supervisor)
    supervisor.scan_obstacles = [(0.,0.)]
    assert supervisor._async_predictive_escape((0.,0.,0.),context.seconds,1.,(0.,0.,0.),.8,(0.,0.,0.)) is None
    assert supervisor.async_escape_state == 'revalidation_rejected'


def test_completed_proposal_is_revalidated_against_new_people(supervisor):
    context,command = completed_worker(supervisor)
    supervisor.latest_predictions = prepare_predictions([[(0.,0.,0.,0.,0.,1.),(3.,0.,0.,0.,0.,1.)]],.1)
    assert supervisor._async_predictive_escape((0.,0.,0.),context.seconds,1.,(0.,0.,0.),0.,(0.,0.,0.)) is None
    assert supervisor.async_escape_state == 'revalidation_rejected'


def test_goal_and_payload_changes_invalidate_cached_and_inflight_generations(supervisor):
    context,command = completed_worker(supervisor)
    supervisor.cached_escape = SearchResult(0,time.monotonic(),time.monotonic(),context,command)
    goal = PoseStamped()
    goal.pose.position.x = 2.
    supervisor._goal_callback(goal)
    assert supervisor.cached_escape is None
    assert supervisor.escape_worker.generation == 1
    supervisor._payload_callback(Bool(data=True))
    assert supervisor.escape_worker.generation == 2


def test_fresh_rechecked_proposal_can_be_used_but_stale_scan_cannot(supervisor):
    context,command = completed_worker(supervisor)
    assert supervisor._async_predictive_escape((0.,0.,0.),context.seconds,1.,(0.,0.,0.),.8,(0.,0.,0.)) == command
    supervisor.scan_stamp -= 2.
    assert supervisor._async_predictive_escape((0.,0.,0.),context.seconds,1.,(0.,0.,0.),.8,(0.,0.,0.)) is None
    assert supervisor.async_escape_state == 'stale_scan'


def test_supervisor_keeps_validating_while_search_is_blocked(supervisor):
    entered,release = threading.Event(),threading.Event()
    def delayed(context):
        entered.set()
        assert release.wait(2.)
        return search_escape(context)
    supervisor.escape_worker = EscapeSearchWorker(delayed)
    supervisor.predictive_escape_search = True
    try:
        start = time.monotonic()
        command,reason = supervisor._validate_final_command(Twist(),(0.,0.,0.),supervisor.get_clock().now())
        assert time.monotonic()-start < .1
        assert entered.wait(1.)
        assert reason == 'temporal_override'
        assert command.linear.x == command.linear.y == command.angular.z == 0.
        # A second validation must not queue another search or wait for this one.
        supervisor._validate_final_command(Twist(),(0.,0.,0.),supervisor.get_clock().now())
        assert supervisor.escape_worker.counters['submitted'] == 1
    finally:
        release.set()


def test_late_completed_result_cannot_cross_a_goal_fence(supervisor):
    context,command = completed_worker(supervisor)
    goal = PoseStamped()
    goal.pose.position.y = 2.
    supervisor._goal_callback(goal)
    assert supervisor._async_predictive_escape((0.,0.,0.),context.seconds,1.,(0.,0.,0.),.8,(0.,0.,0.)) is None
    assert supervisor.async_escape_state == 'waiting_for_current_result'


def test_idle_control_tick_drains_completed_work(supervisor):
    supervisor.escape_worker = EscapeSearchWorker(lambda context:None)
    supervisor.escape_worker.submit(snapshot(supervisor.get_clock().now().nanoseconds*1e-9))
    wait_done(supervisor.escape_worker)
    supervisor._robot_pose = lambda:None
    supervisor._timer_callback()
    assert supervisor.escape_worker.future is None
    assert supervisor.cached_escape is not None and supervisor.cached_escape.command is None
    assert supervisor.escape_worker.counters['no_candidate'] == 1


def test_mission_and_publisher_changes_fence_search_results(supervisor):
    context,command = completed_worker(supervisor)
    supervisor.cached_escape = SearchResult(0,time.monotonic(),time.monotonic(),context,command)
    supervisor._mission_callback(Bool(data=True))
    assert supervisor.cached_escape is None and supervisor.escape_worker.generation == 1
    supervisor.require_command_envelope = True
    packet = dict(version=1,publisher_id='first',sequence_id=1,
        stamp=supervisor.get_clock().now().nanoseconds*1e-9,frame=supervisor.robot_frame,
        active=True,goal=supervisor.goal_signature,first=[0.,0.,0.],then=None,duration=0.,state='EMERGENCY_STOP')
    supervisor._sequence_callback(String(data=json.dumps(packet)))
    assert supervisor.escape_worker.generation == 2
    supervisor.command_gate.stamp -= 1.  # Old producer lease expired.
    packet.update(publisher_id='replacement',stamp=supervisor.get_clock().now().nanoseconds*1e-9)
    supervisor._sequence_callback(String(data=json.dumps(packet)))
    assert supervisor.escape_worker.generation == 3


def test_shutdown_requests_stop_before_waiting_for_worker():
    rclpy.init()
    node = HumanCollisionAvoidance()
    events = []
    node.pub = SimpleNamespace(publish=lambda command:events.append(
        ('stop',command.linear.x,command.linear.y,command.angular.z)))
    node.escape_worker = SimpleNamespace(shutdown=lambda:events.append(('worker_shutdown',)))
    try:
        node.destroy_node()
        assert events == [('stop',0.,0.,0.),('worker_shutdown',)]
    finally:
        rclpy.shutdown()


def test_async_configuration_without_epoch_bearing_input_is_rejected():
    rclpy.init(args=['--ros-args','-p','async_predictive_escape_search:=true'])
    try:
        with pytest.raises(ValueError,match='requires require_command_envelope'):
            HumanCollisionAvoidance()
    finally:
        rclpy.shutdown()
