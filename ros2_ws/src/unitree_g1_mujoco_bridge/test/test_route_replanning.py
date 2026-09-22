from unitree_g1_mujoco_bridge.route_replanning import (
    ReplanPolicy, corridor_people, distance_to_route, forward_route,
)


def test_crossing_people_trigger_before_reaching_the_route_and_far_people_do_not():
    route = [(0., 0.), (4., 0.)]
    people = [(1, 2., -2., 0., 1., .4), (2, 20., 20., 1., 0., .4)]
    assert set(corridor_people(people, route)) == {1}
    assert distance_to_route((2., 1.), route) == 1.


def test_events_preempt_periodic_replanning_but_are_rate_limited():
    policy = ReplanPolicy()
    assert policy.reason(0., {}, False) == "missing_route"
    policy.accept_plan(.01)
    people = {1: (1., 0., .5, 0.)}
    assert policy.reason(.1, people, True) is None
    assert policy.reason(.21, people, True) == "corridor_occupancy_changed"
    policy.accept_plan(.22)
    assert policy.reason(.42, {1: (1.3, 0., .5, 0.)}, True) == "corridor_motion_changed"
    policy.accept_plan(.43)
    assert policy.reason(.64, {}, True) == "corridor_occupancy_changed"  # Cleared routes shorten again.
    policy.accept_plan(.65)
    assert policy.reason(.85, {}, True) is None
    assert policy.reason(1.16, {}, True) == "periodic_refresh"


def test_map_blockage_and_robot_drift_request_replans_without_people():
    policy = ReplanPolicy()
    policy.accept_plan(0.)
    assert policy.reason(.05, {}, True, map_blocked=True) == "observed_map_blocked"
    assert policy.reason(.3, {}, True) == "observed_map_cleared"
    assert policy.reason(.55, {}, True, off_route=True) == "off_route"


def test_obstacles_behind_robot_are_not_in_the_forward_route():
    route = [(float(x), 0.) for x in range(20)]
    ahead = forward_route(route, (10., .2), 3.)
    assert ahead == [(10., 0.), (11., 0.), (12., 0.), (13., 0.)]
    assert corridor_people([(1, 2., 0., 0., 0., .4)], ahead) == {}
