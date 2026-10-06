import json

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from rclpy.duration import Duration
from std_msgs.msg import Bool,String

from unitree_g1_mujoco_bridge.command_envelope import CommandEnvelopeGate
from unitree_g1_mujoco_bridge.human_collision_avoidance import HumanCollisionAvoidance


def packet(sequence=1,stamp=10.,publisher='planner',**changes):
    return dict(version=1,publisher_id=publisher,sequence_id=sequence,stamp=stamp,
                frame='base_footprint',active=True,goal=['odom',1,0,2.,0.],
                first=[.3,0.,0.],then=None,duration=0.,state='NORMAL_NAVIGATION',**changes)


def accept(gate,data,now=10.1,active=True):
    return gate.accept(data,now,'base_footprint',['odom',1,0,2.,0.],active,.25)


def test_atomic_normal_command_does_not_need_a_continuation():
    gate = CommandEnvelopeGate()
    assert accept(gate,packet()) is None
    assert gate.sequence == 1


def test_delayed_duplicate_and_reordered_packets_cannot_refresh_command():
    gate = CommandEnvelopeGate()
    assert accept(gate,packet(2)) is None
    assert accept(gate,packet(1)) == 'out_of_order'
    assert accept(gate,packet(2)) == 'out_of_order'
    assert accept(gate,packet(3,9.9),now=10.2) == 'expired_or_prefence'
    assert gate.sequence == 2 and gate.stamp == 10.


def test_cancel_resume_fence_rejects_pre_cancel_packet_even_with_same_goal():
    gate = CommandEnvelopeGate()
    accept(gate,packet())
    gate.fence(10.1)
    assert accept(gate,packet(2,10.05),now=10.15) == 'expired_or_prefence'
    assert accept(gate,packet(3,10.12),now=10.15) is None


def test_restart_waits_for_old_lease_and_old_publisher_cannot_return():
    gate = CommandEnvelopeGate()
    accept(gate,packet())
    assert accept(gate,packet(1,10.1,'new'),now=10.1) == 'publisher_busy'
    assert accept(gate,packet(1,10.3,'new'),now=10.3) is None
    assert accept(gate,packet(3,10.6,'planner'),now=10.6) == 'retired_publisher'


@pytest.mark.parametrize('change',[
    {'first':[float('nan'),0.,0.]},{'stamp':10.2},{'active':False},
    {'goal':['odom',2,0,2.,0.]},{'frame':'world'},{'sequence_id':True},
    {'version':True},
])
def test_invalid_packet_does_not_replace_current_lease(change):
    gate = CommandEnvelopeGate()
    accept(gate,packet())
    invalid = packet(2)
    invalid.update(change)
    assert accept(gate,invalid) is not None
    assert gate.sequence == 1


@pytest.fixture
def supervisor():
    rclpy.init()
    node = HumanCollisionAvoidance()
    node.require_command_envelope = True
    node.use_planned_sequence = True
    node._mission_callback(Bool(data=True))
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 2.
    node._goal_callback(goal)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def send(node,sequence=1):
    data = packet(sequence,node.get_clock().now().nanoseconds*1e-9)
    data['frame'],data['goal'] = node.robot_frame,node.goal_signature
    node._sequence_callback(String(data=json.dumps(data)))
    return data


def test_supervisor_uses_atomic_velocity_and_state_not_legacy_topics(supervisor):
    send(supervisor)
    legacy = Twist()
    legacy.linear.x = .6
    supervisor._cmd_callback(legacy)
    supervisor._planner_state_callback(String(data='EMERGENCY_STOP'))
    assert supervisor.latest_cmd.linear.x == .3
    assert supervisor.planner_state == 'NORMAL_NAVIGATION'
    assert not supervisor._input_stale()
    supervisor.latest_cmd_time -= Duration(seconds=.3)
    assert supervisor._input_stale()


def test_goal_change_immediately_revokes_old_velocity(supervisor):
    old = send(supervisor)
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.y = 2.
    supervisor._goal_callback(goal)
    assert supervisor.latest_cmd.linear.x == 0.
    assert supervisor._input_stale()
    old['sequence_id'] += 1
    supervisor._sequence_callback(String(data=json.dumps(old)))
    assert supervisor._input_stale()
    send(supervisor,3)
    assert not supervisor._input_stale()


def test_heading_only_change_with_reused_stamp_revokes_old_velocity(supervisor):
    old = send(supervisor)
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 2.
    goal.pose.orientation.z = 1.
    supervisor._goal_callback(goal)
    assert supervisor._input_stale()
    supervisor._sequence_callback(String(data=json.dumps(old)))
    assert supervisor._input_stale()


def test_inactive_intent_requires_new_packet_and_keeps_emergency_avoidance_available(supervisor):
    old = send(supervisor)
    supervisor._mission_callback(Bool(data=False))
    supervisor._sequence_callback(String(data=json.dumps(old)))
    assert supervisor._input_stale()
    data = packet(2,supervisor.get_clock().now().nanoseconds*1e-9)
    data.update(active=False,goal=supervisor.goal_signature,frame=supervisor.robot_frame,state='CLEARANCE_RECOVERY')
    supervisor._sequence_callback(String(data=json.dumps(data)))
    assert not supervisor._input_stale()
    assert supervisor.latest_cmd.linear.x == .3
