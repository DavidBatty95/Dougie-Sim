from types import SimpleNamespace

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError
from unitree_g1_mujoco_bridge.transport_demo import TransportMission,WorkspaceInterrupted,ReachUnreachable


def mission_fixture():
    events = []
    mission = SimpleNamespace(pose=(0.,0.,0.),phase='reach',events=events,
                              wait_workspace=lambda:None)
    def event(phase,**details):
        mission.phase = phase
        events.append((phase,details))
    mission.event = event
    mission.arm_with_recovery = lambda *args,**kwargs:TransportMission.arm_with_recovery(mission,*args,**kwargs)
    return mission


def test_arm_resumes_from_measured_state_after_terminated_workspace_interruption():
    mission = mission_fixture()
    calls = []
    target = np.array((.36,-.2,.37))
    def move(point,orientation,**kwargs):
        calls.append(point.copy())
        if len(calls) == 1:
            raise WorkspaceInterrupted('person crossing')
        return point
    mission._move_arm = move
    TransportMission.move_arm(mission,target)
    assert len(calls) == 2
    assert any(details.get('resumed') for _,details in mission.events)


def test_recovery_does_not_retry_when_action_termination_is_unknown():
    mission = mission_fixture()
    def move(*args,**kwargs):
        raise ActionTerminationError('server disappeared')
    mission._move_arm = move
    with pytest.raises(ActionTerminationError):
        TransportMission.move_arm(mission,np.zeros(3))
    assert not mission.events


def test_repeated_workspace_intrusions_have_a_bounded_retry_count():
    mission = mission_fixture()
    calls = []
    def move(*args,**kwargs):
        calls.append(1)
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    with pytest.raises(RuntimeError,match='recovery limit'):
        TransportMission.move_arm(mission,np.zeros(3))
    assert len(calls) == 3


@pytest.mark.parametrize('pose',[(.05,0.,0.),(0.,0.,.11)])
def test_base_displacement_requires_redocking_instead_of_replaying_old_target(pose):
    mission = mission_fixture()
    def move(*args,**kwargs):
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    mission.wait_workspace = lambda:setattr(mission,'pose',pose)
    with pytest.raises(RuntimeError,match='re-docking required'):
        TransportMission.move_arm(mission,np.zeros(3))


def test_placement_reobserves_from_retreat_viewpoint_after_release_ack():
    mission = mission_fixture()
    calls = []
    mission.release,mission.grasp_R = object(),np.eye(3)
    mission.seconds = lambda:10.
    mission.service = lambda *args,**kw:calls.append('release_ack')
    mission.move_to_carry = lambda *args:calls.append('retract')
    mission.retreat = lambda *args:calls.append('retreat')
    mission.spin = lambda *args:None
    def observed(identifier,pad,stamp):
        assert calls == ['release_ack','retract','retreat']
        assert stamp == 10.
        calls.append('post_release_observation')
        return True
    mission.placement_observed = observed
    mission.wait = lambda predicate,*args:predicate()
    TransportMission.release_and_verify(mission,'box','pad',np.zeros(3),(1.,0.),0.)
    assert calls[-1] == 'post_release_observation'


def test_uncertain_release_does_not_initiate_retreat_or_verification():
    mission = mission_fixture()
    mission.release = object()
    def service(*args,**kwargs):
        raise RuntimeError('release acknowledgement lost')
    mission.service = service
    with pytest.raises(RuntimeError,match='acknowledgement lost'):
        TransportMission.release_and_verify(mission,'box','pad',np.zeros(3),(1.,0.),0.)
    assert [phase for phase,_ in mission.events] == ['release']


def test_unreachable_pickup_redocks_and_reprojects_before_grasp():
    mission = mission_fixture()
    box = np.array((1.5,0.,1.))
    mission.joints = {}
    mission.observe = lambda *args:dict(position=box)
    mission.wait = lambda predicate,*args:predicate()
    mission.to_pelvis = lambda point:point-np.array((mission.pose[0],mission.pose[1],.8))
    hand = [None]
    calls = []
    def move(target,orientation):
        calls.append(target.copy())
        if target[0] > .5:
            raise ReachUnreachable('base drifted from table')
        hand[0] = target
    mission.move_arm = move
    mission.dock = lambda pose:setattr(mission,'pose',pose)
    mission.kin = SimpleNamespace(forward=lambda joints:(hand[0],None,None,None))
    result = TransportMission.reach_box(mission,'box',np.array((1.,0.)),np.array((0.,1.)),0.)
    np.testing.assert_array_equal(result,box)
    assert len(calls) == 2 and calls[1][0] < .5
    assert any(phase == 'redock_pickup' for phase,_ in mission.events)


def test_docking_recloses_after_coasting_outside_tolerance():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose,mission.active = pose,True
    mission.set_goal = goal
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    def settle(duration):
        if len(goals) == 1:
            mission.pose = (.2,0.,0.)
    mission.spin = settle
    TransportMission.dock(mission,(0.,0.,0.))
    assert len(goals) == 2
    assert not mission.active
    assert mission.events[-1][1]['dock_settled']


def test_docking_retries_share_original_deadline(monkeypatch):
    import unitree_g1_mujoco_bridge.transport_demo as module
    now = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    mission.set_goal = lambda pose,mode:setattr(mission,'pose',pose)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    def drift(duration):
        now[0] += duration
        mission.pose = (.2,0.,0.)
    mission.spin = drift
    with pytest.raises(RuntimeError,match='settle before deadline'):
        TransportMission.dock(mission,(0.,0.,0.),timeout=2.)
    assert now[0] == 2.
    assert not mission.active


def test_stale_pose_cannot_confirm_docking_and_failure_clears_intent():
    mission = mission_fixture()
    mission.pose_stamp = 10.
    mission.seconds = lambda:12.
    mission.set_goal = lambda *args:setattr(mission,'active',True)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    def wait(predicate,*args):
        assert not predicate()
        raise RuntimeError('stale pose')
    mission.wait = wait
    with pytest.raises(RuntimeError,match='stale pose'):
        TransportMission.dock(mission,(0.,0.,0.))
    assert not mission.active


def test_retreat_accepts_safe_coast_past_clearance_without_forward_reversal():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose,mission.active = pose,True
    mission.set_goal = goal
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    mission.spin = lambda duration:setattr(mission,'pose',(-.7,.05,0.))
    TransportMission.retreat(mission,.5,0.)
    assert len(goals) == 1
    assert mission.events[-1][1]['distance_m'] == pytest.approx(.7)
    assert not mission.active
