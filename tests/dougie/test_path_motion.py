import math

import numpy as np
import pytest

from dougie_sim.path_motion import RouteFollower, RouteGeometry, path_length, smooth_route
from dougie_sim.terrain import SurfaceMap


class ConstantGP:
    def predict(self, points):
        return np.ones(len(points)), np.ones(len(points))*.1, np.ones(len(points))*2.


def grid(allowed=lambda x, y: True):
    terrain = SurfaceMap(resolution=.2, radius=.4, max_step=.02)
    terrain.nodes = {(x, y, 0): np.array([(x+.5)*.2, (y+.5)*.2, 0.])
                     for x in range(-4, 17) for y in range(-4, 17) if allowed(x, y)}
    terrain.graph = {key: [other for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                          if (other := (key[0]+dx, key[1]+dy, 0)) in terrain.nodes] for key in terrain.nodes}
    return terrain


def test_open_floor_grid_staircase_becomes_a_shorter_direct_route():
    terrain = grid()
    path = [terrain.nodes[(0, 0, 0)]]
    for i in range(10):
        path += [terrain.nodes[(i+1, i, 0)], terrain.nodes[(i+1, i+1, 0)]]
    result = smooth_route(path, terrain, ConstantGP())
    assert path_length(result.points) < .8*path_length(path)
    assert result.metrics['execution_upper_dose_usv'] <= result.metrics['raw_upper_dose_usv']
    assert result.metrics['execution_turn_rad'] < result.metrics['raw_turn_rad']/3


def test_cycle_cache_reuses_routes_but_new_cycle_rechecks_obstacles():
    from dougie_sim.path_motion import RouteSmoother
    terrain = grid()
    path = [terrain.nodes[(0, 0, 0)], terrain.nodes[(10, 0, 0)], terrain.nodes[(10, 10, 0)]]
    smoother = RouteSmoother(terrain, ConstantGP(), 100.)
    checked = smoother(path)
    assert smoother(path) is checked and smoother.hits == 1
    terrain.hits[0] = np.array([1.1, 1.1, .4])
    refreshed = RouteSmoother(terrain, ConstantGP(), 100.)(path)
    assert path_length(refreshed.points) > path_length(checked.points)


def test_unknown_inside_corner_cannot_be_shortcut_or_rounded():
    terrain = grid(lambda x, y: (y == 0 and 0 <= x <= 10) or (x == 10 and 0 <= y <= 10))
    path = [terrain.nodes[(x, 0, 0)] for x in range(11)]+[terrain.nodes[(10, y, 0)] for y in range(1, 11)]
    result = smooth_route(path, terrain, ConstantGP())
    assert result.metrics['rounded_corners'] == 0
    assert all(abs(p[1]-.1) < 1e-8 or abs(p[0]-2.1) < 1e-8 for p in result.points)


def test_body_clearance_blocks_a_shortcut_even_with_admitted_centre_nodes():
    terrain = grid()
    terrain.hits[0] = np.array([1.1, 1.1, .4])
    geometry = RouteGeometry(terrain)
    assert geometry.line(np.array([.1, .1, 0.]), np.array([2.1, 2.1, 0.])) is None


def test_stacked_floor_jump_and_blocked_edges_are_rejected():
    terrain = grid()
    geometry = RouteGeometry(terrain)
    assert geometry.line([.1, .1, 0.], [.1, .1, 2.4]) is None
    a, b = (0, 0, 0), (1, 0, 0)
    terrain.graph[a].remove(b)
    assert RouteGeometry(terrain).line(terrain.nodes[a], terrain.nodes[b]) is None


def test_radiation_hotspot_prevents_a_geometrically_valid_diagonal():
    class Hotspot:
        def predict(self, points):
            points = np.asarray(points)
            upper = 2.+1000.*np.exp(-np.sum((points[:, :2]-[1.1, .3])**2, axis=1)/.005)
            return upper, np.ones(len(points)), upper
    terrain = grid()
    path = [terrain.nodes[(x, 0, 0)] for x in range(11)]+[terrain.nodes[(10, y, 0)] for y in range(1, 11)]
    result = smooth_route(path, terrain, Hotspot())
    assert result.metrics['fallback_reason'] == 'predicted_exposure_or_rate_increase'
    np.testing.assert_allclose(result.points, path)


def test_follower_does_not_look_through_an_uncertified_right_angle():
    follower = RouteFollower()
    follower.set_route([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]], [[0, 1], [1, 2]], [.7, 0, 0])
    command = follower.tick([.7, 0, 0], 0., .02)
    assert command[0] > 0 and command[2] == 0.
    command = follower.tick([1., 0., 0.], 0., .02)
    assert command[2] > 0


def test_command_slew_limits_and_emergency_stop():
    follower = RouteFollower()
    follower.set_route([[0., 0., 0.], [3., 0., 0.]], [[0, 1]], [0, 0, 0])
    previous = np.zeros(3)
    for _ in range(30):
        command = follower.tick([0., 0., 0.], -.2, .02)
        assert abs(command[0]-previous[0]) <= .35*.02+1e-9
        assert abs(command[2]-previous[2]) <= .02+1e-9
        previous = command
    np.testing.assert_array_equal(follower.stop(), np.zeros(3))
    np.testing.assert_array_equal(follower.tick([2., 2., 0.], 0, .02), np.zeros(3))


def test_delayed_replan_joins_later_span_without_backtracking():
    follower = RouteFollower()
    points = [[0., 0., 0.], [.2, 0., 0.], [.4, 0., 0.], [2., 0., 0.]]
    assert follower.set_route(points, None, [.6, 0., 0.])
    assert follower.progress == pytest.approx(.6)
    command = follower.tick([.6, 0., 0.], 0., .02)
    assert command[0] > 0 and command[2] == 0


@pytest.mark.parametrize('points', [
    [[0., 0., 0.], [10., 0., 0.]],
    [[0., 0., 0.], [.2, 0., 0.], [3., 0., 0.], [10., 0., 0.]],
])
def test_join_window_is_bounded_even_for_long_segments(points):
    follower = RouteFollower()
    assert not follower.set_route(points, None, [3., 0., 0.])
    np.testing.assert_array_equal(follower.tick([3., 0., 0.], 0., .02), np.zeros(3))
    assert follower.status == 'waiting_for_route_reacquisition'
    assert follower.set_route([[3., 0., 0.], [10., 0., 0.]], None, [3., 0., 0.])
    assert follower.tick([3., 0., 0.], 0., .02)[0] > 0


def test_replan_does_not_join_a_different_floor():
    follower = RouteFollower()
    assert not follower.set_route([[0., 0., 0.], [1., 0., 0.]], None, [.5, 0., 2.4])
    np.testing.assert_array_equal(follower.tick([.5, 0., 2.4], 0., .02), np.zeros(3))


def test_corner_progress_cannot_advance_before_reaching_span_endpoint():
    follower = RouteFollower()
    follower.set_route([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]], None, [.7, 0., 0.])
    follower.tick([1.01, -.09, 0.], 0., .02)
    assert follower.progress < 1.


@pytest.mark.parametrize('dt', [0., -.1, float('nan'), float('inf')])
def test_invalid_controller_timestep(dt):
    with pytest.raises(ValueError):
        RouteFollower().tick([0., 0., 0.], 0., dt)
