import math

import mujoco
import numpy as np

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.demo_overlay import DemoControls, draw_overlay, hud_lines
from dougie_sim.radiation import RadiationSample
from dougie_sim.region_clustering import connected_clusters, distance_from_visits, ObservedGain
from dougie_sim.simulation import Simulation
from dougie_sim.terrain import SurfaceMap
from dougie_sim.reference_terrain import ReferenceSurfaceMap


def test_clustering_cannot_merge_across_disconnected_walls_or_floors():
    nodes = {(0, 0, 0): np.array([0., 0., 0.]), (1, 0, 0): np.array([.25, 0., 0.]),
             (2, 0, 0): np.array([.5, 0., 0.]), (0, 0, 100): np.array([0., 0., 2.])}
    graph = {key: [] for key in nodes}
    graph[(0, 0, 0)] = [(1, 0, 0)]
    graph[(1, 0, 0)] = [(0, 0, 0)]
    rates = dict.fromkeys(nodes, 1.)
    sigmas = dict.fromkeys(nodes, .2)
    clusters = connected_clusters(nodes, graph, rates, sigmas, 100., max_clusters=1)
    assert sorted(map(len, clusters)) == [1, 1, 2]
    assert set(key for cluster in clusters for key in cluster) == set(nodes)


def test_risk_threshold_and_uncertainty_remain_distinguishable():
    keys = [(i, 0, 0) for i in range(3)]
    nodes = {key: np.array([i*.25, 0, 0.]) for i, key in enumerate(keys)}
    graph = {keys[0]: [keys[1]], keys[1]: [keys[0], keys[2]], keys[2]: [keys[1]]}
    rates = dict(zip(keys, [99., 101., 101.]))
    sigmas = dict(zip(keys, [.2, .2, 1.2]))
    assert len(connected_clusters(nodes, graph, rates, sigmas, 100., max_clusters=1)) == 3


def test_visit_distance_cannot_cross_a_wall():
    nodes = {0: np.array([0., 0., 0.]), 1: np.array([.25, 0., 0.]), 2: np.array([.5, 0., 0.])}
    graph = {0: [1], 1: [0], 2: []}
    distance = distance_from_visits(nodes, graph, {0})
    assert distance[1] == .25 and math.isinf(distance[2])


def test_observed_gain_does_not_count_unknown_behind_a_closed_wall():
    terrain = SurfaceMap()
    key = (0, 0, 0)
    terrain.nodes[key] = np.array([.125, .125, 0.])
    terrain.surfaces = {(x, y, 0): 0. for x in range(-2, 3) for y in range(-2, 3)}
    walls = [(x, y) for x in range(-3, 4) for y in range(-3, 4) if abs(x) == 3 or abs(y) == 3]
    terrain.hits = {(x, y, 3): np.array([(x+.5)*.25, (y+.5)*.25, .4]) for x, y in walls}
    assert ObservedGain(terrain).value(key) == 0
    terrain.hits.clear()
    assert ObservedGain(terrain).value(key) > 0


def test_cached_novelty_is_invalidated_by_new_observations():
    terrain = SurfaceMap()
    assert terrain.novelty((0, 0, 0)) == 121
    terrain.update(np.empty((0, 3)), np.array([[.1, .1, 0.]]))
    assert terrain.novelty((0, 0, 0)) == 120


def test_conservative_profile_rejects_floor_hypothesis_beneath_a_low_ledge():
    terrain = SurfaceMap(radius=.4, max_step=.02, max_local_relief=.02)
    floor = np.array([[x, y, 0.] for x in np.arange(-2., 2., .1) for y in np.arange(-2., 2., .1)])
    ledge = np.array([[x, y, .06] for x in np.arange(.5, 1.5, .1) for y in np.arange(-.5, .5, .1)])
    terrain.update(np.vstack((floor, ledge)), np.vstack((floor, ledge)))
    terrain.build_graph()
    assert terrain.nearest([-.5, 0, 0]) is not None
    assert not any(.6 < p[0] < 1.4 and abs(p[1]) < .4 for p in terrain.nodes.values())


def test_clustered_planner_preserves_budget_and_does_not_create_centroid_paths():
    planner = ClusteredDoseExplorer(budget=2.)
    supports = np.array([[x, y, 0.] for x in np.arange(-2., 2., .1) for y in np.arange(-2., 2., .1)])
    planner.observe_geometry(supports, supports)
    sample = RadiationSample(1., (0, 0, .44), 5, .5, .72, .1, .001, .002)
    planner.observe_radiation(sample)
    planner.replan([0, 0, 0], 1.)
    assert planner.path
    points = {tuple(point): key for key, point in planner.terrain.nodes.items()}
    for a, b in zip(planner.path, planner.path[1:]):
        assert points[tuple(b)] in planner.terrain.graph[points[tuple(a)]]
    target = planner.target.copy()
    planner.replan([.05, 0, 0], 1.5)
    np.testing.assert_allclose(planner.target, target)
    assert planner.last_reason == 'continue_revalidated_goal'
    planner.observe_radiation(RadiationSample(2., (0, 0, .44), 5, .5, .72, .1, 1.9, 1.95))
    planner.replan([0, 0, 0], 2.)
    assert planner.returning and planner.state == 'returned'


def test_overlay_is_bounded_and_hud_uses_measured_dose():
    sim = Simulation()
    scene = mujoco.MjvScene(sim.model, maxgeom=8)
    controls = DemoControls()
    sample = RadiationSample(1., (0, 0, 0), 3, .5, 4., .2, .125, .14)
    metrics = dict(state='exploring', planner='clustered', selected_target=[1., 0., 0.],
                   planned_path=[[0., 0., 0.], [1., 0., 0.]], clusters=[dict(anchor=[1, 0, 0])])
    volume = (np.zeros((100, 3)), np.ones(100), np.ones(100), np.ones(100)*5)
    draw_overlay(scene, volume, metrics, [], controls, 100.)
    assert scene.ngeom <= 8
    left, right = hud_lines(metrics, sample, 1., 2., 'mean')
    assert '.1250 / 2' in right and 'Measured dose' in left
    controls.key(ord('C'))
    controls.key(ord('M'))
    assert controls.camera == 'overview' and controls.layer == 'uncertainty'


def test_optimized_geometry_and_costs_match_frozen_reference():
    ground = np.array([[x, y, 0.] for x in np.arange(-2, 2, .1) for y in np.arange(-2, 2, .1)])
    upper = np.array([[x, y, 2.] for x in np.arange(-2, 2, .2) for y in np.arange(-2, 2, .2)])
    wall = np.array([[.8, y, z] for y in np.arange(-1., 1., .1) for z in (.2, .4, .6)])
    maps = [ReferenceSurfaceMap(), SurfaceMap()]
    for grid in maps:
        grid.update(np.vstack((ground, upper, wall)), np.vstack((ground, upper)))
        grid.build_graph()
    old, new = maps
    assert old.graph == new.graph
    rates = {key: 1.+abs(p[0]) for key, p in old.nodes.items()}
    root = old.nearest([0, 0, 0])
    for a, b in zip(old.routes(root, rates)[:3], new.routes(root, rates)[:3]):
        assert a.keys() == b.keys()
        np.testing.assert_allclose([a[k] for k in a], [b[k] for k in a], rtol=1e-10, atol=1e-10)
    assert [old.novelty(k) for k in old.nodes] == [new.novelty(k) for k in old.nodes]


def test_stalled_edge_is_temporarily_removed_in_both_directions():
    planner = ClusteredDoseExplorer()
    a, b = (0, 0, 0), (1, 0, 0)
    planner.terrain.nodes = {a: np.array([0., 0., 0.]), b: np.array([.25, 0., 0.])}
    planner.terrain.graph = {a: [b], b: [a]}
    planner.path = [np.array([.25, 0., 0.])]
    planner.target = np.array([1., 0., 0.])
    planner.report_stall([0., 0., 0.], 10.)
    assert planner.blocked_edges[tuple(sorted((a, b)))] == 10.
    assert not planner.path
    planner.terrain.build_graph = lambda: (planner.terrain.nodes, planner.terrain.graph)
    planner.replan([0., 0., 0.], 11.)
    assert b not in planner.terrain.graph[a] and a not in planner.terrain.graph[b]
