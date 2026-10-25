from types import SimpleNamespace
from dataclasses import replace

import numpy as np
import pytest

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.radiation import RadiationSample
from dougie_sim.route_forecast import forecast_tree, forecast_polyline
from dougie_sim.terrain import SurfaceMap


def planner_fixture(monkeypatch, coordinates, edges, rates=None):
    planner = ClusteredDoseExplorer(budget=5., mission_duration=1200.)
    terrain = planner.terrain
    terrain.nodes = {key: np.asarray(point, dtype=float) for key, point in coordinates.items()}
    terrain.graph = edges
    terrain.build_graph = lambda: (terrain.nodes, terrain.graph)
    rates = rates or {key: 1. for key in coordinates}
    def predict(points):
        keys = [min(terrain.nodes, key=lambda k: np.linalg.norm(terrain.nodes[k]-(p-[0., 0., .44]))) for p in points]
        return np.ones(len(keys)), np.full(len(keys), .1), np.array([rates[key] for key in keys])
    planner.gp = SimpleNamespace(predict=predict)
    planner.sample = RadiationSample(1., (0., 0., .44), 1, .5, .144, .01, .001, .002)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain', lambda terrain: SimpleNamespace(value=lambda key: 0, cache={}))
    return planner


def test_exhausted_local_area_repositions_to_distant_observed_frontier(monkeypatch):
    keys = [(x, 0, 0) for x in range(10)]
    edges = {key: [keys[j] for j in (i-1, i+1) if 0 <= j < len(keys)] for i, key in enumerate(keys)}
    planner = planner_fixture(monkeypatch, {key: key for key in keys}, edges)
    planner.visited.update(keys[:-1])
    planner.replan(np.zeros(3), 1.)
    np.testing.assert_allclose(planner.target, [9., 0., 0.])
    assert planner.last_reason == 'reposition_to_distant_frontier'
    assert planner.agenda[keys[-1]]['status'] == 'active'
    planner.sample = replace(planner.sample, stamp_s=2.)
    planner.replan(np.array([1., 0., 0.]), 2.)
    assert planner.last_reason == 'continue_revalidated_goal'
    assert keys[-1] in planner.agenda


def test_outbound_reachable_goal_with_only_hot_return_is_rejected(monkeypatch):
    root, goal, hot = (0, 0, 0), (1, 0, 0), (2, 0, 0)
    planner = planner_fixture(monkeypatch, {k: k for k in (root, goal, hot)},
        {root: [goal], goal: [hot], hot: [root]}, {root: 1., goal: 1., hot: 200.})
    planner.replan(np.zeros(3), 1.)
    assert planner.returning
    assert planner.rejections['return_route'] > 0
    assert planner.agenda[goal]['status'] == 'no_rate_compliant_return'
    np.testing.assert_allclose(planner.target, np.zeros(3))


def test_rate_constrained_search_finds_long_safe_alternative():
    terrain = SurfaceMap()
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([1., 0., 0.]),
                     2: np.array([2., 0., 0.]), 3: np.array([1., 10., 0.])}
    terrain.graph = {0: [1, 3], 1: [2], 3: [2], 2: []}
    rates = {0: 1., 1: 101., 2: 1., 3: 1.}
    unconstrained = terrain.routes(0, rates)[3]
    constrained = terrain.routes(0, rates, max_rate=100.)[3]
    assert terrain.path(unconstrained, 2) == [0, 1, 2]
    assert terrain.path(constrained, 2) == [0, 3, 2]
    reverse = terrain.routes(2, rates, reverse=True, max_rate=100.)[3]
    assert terrain.path(reverse, 0) == [2, 3, 0]


def test_forecast_accounts_for_turning_and_reverse_directed_edge_time():
    terrain = SurfaceMap()
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([1., 0., 0.]), 2: np.array([1., 1., 0.])}
    terrain.edge_time = lambda a, b, speed: 2. if a < b else 3.
    parents, rates = {0: None, 1: 0, 2: 1}, {0: 2., 1: 10., 2: 2.}
    time, dose, peak = forecast_tree(terrain, parents, rates)
    turn = (np.pi/2)/.35
    assert time[2] == pytest.approx(4.+turn)
    assert dose[2] == pytest.approx(10.*(4.+turn)/3600.)
    assert peak[2] == 10.
    reverse_time, _, _ = forecast_tree(terrain, parents, rates, reverse=True)
    assert reverse_time[2] == pytest.approx(6.+turn)


def test_agenda_is_bounded_and_plan_delay_reserve_is_finite():
    planner = ClusteredDoseExplorer()
    planner.agenda_capacity = 2
    planner.terrain.nodes = {k: np.array([float(k), 0., 0.]) for k in range(3)}
    for key in range(3):
        planner._remember(key, 'pending', 1.)
    assert len(planner.agenda) == 2 and planner.agenda_overflow == 1
    planner.observe_plan_delay(float('nan'))
    planner.observe_plan_delay(10.)
    assert planner.wait_allowance == 10.


def test_checked_polyline_forecast_does_not_charge_for_removed_grid_corners():
    grid = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [2., 1., 0.]]
    checked = [[0., 0., 0.], [2., 1., 0.]]
    grid_time, grid_dose = forecast_polyline(grid, [10.]*4)
    smooth_time, smooth_dose = forecast_polyline(checked, [10.]*2)
    assert smooth_time == pytest.approx(np.sqrt(5.)/.06)
    assert smooth_time < grid_time and smooth_dose < grid_dose


def test_committed_supported_goal_is_refined_before_grid_turn_deadline_rejection():
    planner = ClusteredDoseExplorer(budget=10., mission_duration=80.)
    terrain = planner.terrain
    terrain.resolution = .2
    terrain.nodes = {(x, y, 0): np.array([(x+.5)*.2, (y+.5)*.2, 0.])
                     for x in range(-4, 12) for y in range(-4, 12)}
    terrain.graph = {key: [other for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                          if (other := (key[0]+dx, key[1]+dy, 0)) in terrain.nodes] for key in terrain.nodes}
    terrain.build_graph = lambda: (terrain.nodes, terrain.graph)
    planner.gp = SimpleNamespace(predict=lambda p: (np.ones(len(p)), np.ones(len(p))*.1, np.ones(len(p))))
    planner.sample = RadiationSample(1., (.1, .1, .44), 5, .5, .72, .1, .001, .002)
    planner.home = np.array([.1, .1, 0.])
    planner.target = np.array([1.1, 1.1, 0.])
    planner.smooth_execution = planner.execution_admission_refinement = True
    planner.replan(planner.home.copy(), 1.)
    np.testing.assert_allclose(planner.target, [1.1, 1.1, 0.])
    assert planner.last_reason == 'continue_revalidated_goal'
    assert planner.route_forecast['checked_execution_candidates'] == 1


def test_start_heading_exposure_and_shared_search_speed():
    points = np.array([[0., 0., 0.], [1., 0., 0.]])
    direct, dose = forecast_polyline(points, [10., 10.])
    turned, turn_dose = forecast_polyline(points, [10., 10.], start_yaw=np.pi)
    assert turned-direct == pytest.approx(np.pi/.35)
    assert turn_dose-dose == pytest.approx(10.*np.pi/.35/3600.)
    terrain = SurfaceMap()
    terrain.nodes = dict(enumerate(points))
    terrain.graph = {0: [1], 1: []}
    _, search_dose, _, _ = terrain.routes(0, {0: 10., 1: 10.})
    assert search_dose[1] == pytest.approx(dose)


def test_newly_hot_only_return_route_is_reported_as_emergency_retreat(monkeypatch):
    root, hot, home = (2, 0, 0), (1, 0, 0), (0, 0, 0)
    planner = planner_fixture(monkeypatch, {k: k for k in (root, hot, home)},
        {root: [hot], hot: [home], home: []}, {root: 1., hot: 200., home: 1.})
    planner.home = np.zeros(3)
    planner.replan(np.array([2., 0., 0.]), 1.)
    assert planner.returning and planner.state == 'returning'
    assert planner.route_forecast['emergency_rate_override']
    assert planner.route_forecast['return_peak_rate_usv_h'] == 200.
    np.testing.assert_allclose(planner.path[-1], planner.home)


@pytest.mark.parametrize('blocked', [None, 'rate', 'return', 'time', 'dose'])
@pytest.mark.parametrize('committed', [False, True])
def test_vertical_objective_prioritizes_only_admitted_observed_height(monkeypatch, blocked, committed):
    root, flat, high = (0, 0, 0), (1, 0, 0), (0, 2, 1)
    nodes = {root: [0., 0., 0.], flat: [1., 0., 0.], high: [0., 2., .2]}
    edges = {root: [flat, high], flat: [root], high: [] if blocked == 'return' else [root]}
    rates = {root: 1., flat: 1., high: 200. if blocked == 'rate' else 50. if blocked == 'dose' else 1.}
    planner = planner_fixture(monkeypatch, nodes, edges, rates)
    planner.objective, planner.region_scheduling = 'vertical', 'local'
    planner.terrain.active_viewpoints = True
    planner.terrain.acquisition_bonus = lambda key, bands: 0.
    if committed:
        planner.target = np.array(nodes[flat])
        planner.goal_started = .5
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain', lambda terrain:
        SimpleNamespace(value=lambda key: 10. if key == flat else 0., cache={}))
    if blocked == 'time':
        planner.mission_duration = 80.
    if blocked == 'dose':
        planner.budget = 1.
    planner.replan(np.zeros(3), 1.)
    np.testing.assert_allclose(planner.target, nodes[high if blocked is None else flat])
    if blocked is None:
        assert planner.last_reason == 'survey_observed_height_band'
    elif committed:
        assert planner.last_reason == 'continue_revalidated_goal'
        assert planner.goal_started == .5
