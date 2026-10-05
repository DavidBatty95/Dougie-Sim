from types import SimpleNamespace

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError
from unitree_g1_mujoco_bridge.transport_demo import TransportMission,WorkspaceInterrupted


def mission_fixture():
    events = []
    mission = SimpleNamespace(pose=(0.,0.,0.),phase='reach',events=events,
                              wait_workspace=lambda:None)
    def event(phase,**details):
        mission.phase = phase
        events.append((phase,details))
    mission.event = event
    return mission


def test_arm_resumes_from_measured_state_after_terminated_workspace_interruption():
    mission = mission_fixture()
    calls = []
    target = np.array((.36,-.2,.37))
    def move(point,orientation):
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
    def move(*args):
        raise ActionTerminationError('server disappeared')
    mission._move_arm = move
    with pytest.raises(ActionTerminationError):
        TransportMission.move_arm(mission,np.zeros(3))
    assert not mission.events


def test_repeated_workspace_intrusions_have_a_bounded_retry_count():
    mission = mission_fixture()
    calls = []
    def move(*args):
        calls.append(1)
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    with pytest.raises(RuntimeError,match='recovery limit'):
        TransportMission.move_arm(mission,np.zeros(3))
    assert len(calls) == 3


@pytest.mark.parametrize('pose',[(.05,0.,0.),(0.,0.,.11)])
def test_base_displacement_requires_redocking_instead_of_replaying_old_target(pose):
    mission = mission_fixture()
    def move(*args):
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    mission.wait_workspace = lambda:setattr(mission,'pose',pose)
    with pytest.raises(RuntimeError,match='re-docking required'):
        TransportMission.move_arm(mission,np.zeros(3))
