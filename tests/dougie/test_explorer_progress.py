from types import SimpleNamespace as S
from unittest.mock import Mock

from dougie_sim.explore import Explorer


def active_explorer(stamp, position=(0., 0.)):
    return S(get_clock=lambda: S(now=lambda: S(nanoseconds=int(stamp*1e9))),
             pending=None, result=S(done=lambda: False), started=0., cancel=None,
             progress_position=(0., 0.), progress_stamp=0.,
             tf=S(lookup_transform=lambda *args: S(transform=S(translation=S(x=position[0], y=position[1])))),
             goal_handle=S(cancel_goal_async=Mock(return_value=object())), report=Mock())


def test_stalled_action_cancelled_in_simulation_time():
    node = active_explorer(16.)
    Explorer.tick(node)
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert node.cancel is not None
    Explorer.tick(node)
    node.goal_handle.cancel_goal_async.assert_called_once()


def test_progress_refreshes_stall_timer_and_paused_clock_does_not_timeout():
    node = active_explorer(16., (.2, 0.))
    Explorer.tick(node)
    assert node.progress_stamp == 16.
    node.goal_handle.cancel_goal_async.assert_not_called()
    for _ in range(3):
        Explorer.tick(node)
    node.goal_handle.cancel_goal_async.assert_not_called()
