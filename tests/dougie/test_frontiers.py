from types import SimpleNamespace as S

import numpy as np
import pytest

from dougie_sim.frontiers import frontier_goals
from dougie_sim.ros_bridge import CommandWatchdog


def grid(data):
    return S(data=data.flatten().tolist(), info=S(width=data.shape[1], height=data.shape[0],
        resolution=.1, origin=S(position=S(x=0., y=0.), orientation=S(x=0., y=0., z=0., w=1.))))


def test_frontiers_are_known_clear_and_not_across_a_wall():
    data = np.full((80, 80), -1)
    data[5:75, 5:50] = 0
    data[:, 30] = 100
    goals = frontier_goals(grid(data), (1.5, 4.))
    assert goals
    assert all(.5 < x < 2.7 and .5 < y < 7.5 for _, x, y, _ in goals)


def test_unknown_or_blocked_start_never_produces_a_goal():
    data = np.full((40, 40), -1)
    assert frontier_goals(grid(data), (2., 2.)) == []
    data[:] = 0
    data[20, 20] = 100
    assert frontier_goals(grid(data), (2., 2.)) == []


def test_invalid_map_is_rejected():
    message = grid(np.zeros((10, 10)))
    message.info.resolution = float('nan')
    with pytest.raises(ValueError):
        frontier_goals(message, (0., 0.))


def test_frontier_cache_matches_uncached_after_motion_and_map_change():
    data = np.full((50, 50), -1)
    data[5:45, 5:40] = 0
    cache = {}
    for position in ((1.5, 2.), (2., 2.)):
        message = grid(data)
        assert frontier_goals(message, position, cache=cache) == frontier_goals(message, position)
    old_visibility = cache['visibility']
    data[:, 30] = 100
    message = grid(data)
    assert frontier_goals(message, (2., 2.), cache=cache) == frontier_goals(message, (2., 2.))
    assert cache['visibility'] is not old_visibility


def test_watchdog_stops_stale_nonfinite_and_time_reversed_commands():
    watchdog = CommandWatchdog()
    assert not watchdog.value(0.).any()
    watchdog.update((.3, 0., .2), 1.)
    np.testing.assert_allclose(watchdog.value(1.2), (.3, 0., .2))
    assert not watchdog.value(1.6).any()
    assert not watchdog.value(.9).any()
    watchdog.update((float('nan'), 0., 0.), 2.)
    assert not watchdog.value(2.).any()
