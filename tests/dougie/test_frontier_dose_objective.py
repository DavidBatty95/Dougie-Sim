from collections import deque
from types import SimpleNamespace

import numpy as np

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.mission_utility import ExposureUtility
from dougie_sim.radiation import RadiationSample


def planner(monkeypatch, soft):
    explorer = ClusteredDoseExplorer(mission_duration=1e12)
    explorer.budget_enabled = explorer.rate_limited = not soft
    explorer.survey_until_complete = True
    explorer.observed_rates.append(1.)
    terrain = explorer.terrain
    terrain.nodes = {0: np.array([0.,0.,0.]), 1: np.array([1.,0.,0.]), 2: np.array([0.,1.,0.])}
    terrain.graph = {0:[1,2], 1:[0], 2:[0]}
    terrain.build_graph = lambda: (terrain.nodes, terrain.graph)
    def predict(points):
        means = np.array([5. if p[1] > .1 else 1. for p in points])
        upper = np.array([100. if p[0] > .1 else 6. if p[1] > .1 else 1. for p in points])
        return means, np.ones(len(points)), upper
    explorer.gp = SimpleNamespace(predict=predict)
    explorer.sample = RadiationSample(1., (0.,0.,.44), 5, .5, 1., .1, .001, .002)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain', lambda terrain: SimpleNamespace(value=lambda key:int(key!=0), cache={}))
    return explorer


def test_uncapped_expected_dose_objective_is_not_uncertainty_avoidance(monkeypatch):
    expected = planner(monkeypatch, True)
    expected.replan([0.,0.,0.], 1.)
    np.testing.assert_array_equal(expected.target, [1.,0.,0.])
    assert expected.frontier_decision['prediction'] == 'mean'
    conservative = planner(monkeypatch, False)
    conservative.replan([0.,0.,0.], 1.)
    np.testing.assert_array_equal(conservative.target, [0.,1.,0.])
    assert conservative.frontier_decision['prediction'] == 'upper'


def test_repeated_count_windows_reduce_stale_viewpoint_reward(monkeypatch):
    explorer = planner(monkeypatch, True)
    explorer.utility.exposure[explorer.utility.cell([1.,0.,.44])] = 90.
    explorer.replan([0.,0.,0.], 1.)
    np.testing.assert_array_equal(explorer.target, [0.,1.,0.])
    assert explorer.frontier_count == 2 and not explorer.frontier_complete


def test_low_rate_and_high_rate_worlds_use_consistent_dose_units():
    utility = ExposureUtility()
    low = utility.frontier_score([0,0,.44], 4., 1., 30., .01, 1., 1., 3.)
    high = utility.frontier_score([0,0,.44], 4., 1., 30., 1., 100., 100., 3.)
    assert abs(low['score']-high['score']) < 1e-10


def test_nonproductive_cycles_stop_as_incomplete_not_as_finished(monkeypatch):
    explorer = planner(monkeypatch, True)
    explorer.observed_support_keys = {0,1,2}
    explorer.last_support_progress_s = 0.
    explorer.frontier_goal_history = deque([(2,0,0),(0,2,0)]*8, maxlen=16)
    explorer.sample = RadiationSample(201., (0.,0.,.44), 5, .5, 1., .1, .001, .002)
    explorer.replan([0.,0.,0.], 201.)
    assert explorer.returning and explorer.last_reason == 'frontier_progress_stalled'
    assert explorer.frontier_count == 2 and not explorer.frontier_complete


def test_new_radiation_evidence_can_preempt_an_old_goal_commitment(monkeypatch):
    explorer = planner(monkeypatch,True)
    explorer.target = np.array([0.,1.,0.])
    explorer.committed_goal_rate = .1
    explorer.goal_started = 0.
    explorer.sample = RadiationSample(3.,(0.,0.,.44),5,.5,1.,.1,.001,.002)
    explorer.replan([0.,0.,0.],3.)
    np.testing.assert_array_equal(explorer.target,[1.,0.,0.])
    assert explorer.frontier_decision['goal_preempted_for_risk']


def test_execution_lookahead_is_bounded_and_uses_certified_geometry(monkeypatch):
    explorer = ClusteredDoseExplorer(mission_duration=1e12)
    explorer.budget_enabled = explorer.rate_limited = False
    explorer.survey_until_complete = explorer.smooth_execution = True
    explorer.observed_rates.append(1.)
    support = np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    explorer.observe_geometry(support,support)
    explorer.gp = SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),.1),np.full(len(p),1.3)))
    explorer.sample = RadiationSample(1.,(0.,0.,.44),5,.5,1.,.1,.001,.002)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:SimpleNamespace(value=lambda key:1,cache={}))
    explorer.replan(np.zeros(3),1.)
    assert 0 < explorer.frontier_decision['execution_refined_candidates'] <= 10
    assert explorer.selected_utility['cost_geometry'] in ('support_checked','grid_fallback')
    assert explorer.path and explorer.route_forecast['outbound_expected_dose_usv'] >= 0.
    # Retaining the goal must retain/revalidate its selected execution route,
    # not silently fall back to the unscored grid-parent alternative.
    explorer.sample = RadiationSample(2.,(0.,0.,.44),5,.5,1.,.1,.001,.002)
    explorer.replan(np.zeros(3),2.)
    assert explorer.selected_utility.get('cost_geometry') in ('support_checked','grid_fallback')
    assert not explorer.frontier_decision['route_cost_preempted']
    target = explorer.target.copy()
    def changed_path_cost(points):
        mean = np.array([1. if np.linalg.norm(p[:2]-target[:2]) < .03 else 20. for p in points])
        return mean,np.full(len(points),.1),mean*1.3
    explorer.gp.predict = changed_path_cost
    explorer.sample = RadiationSample(3.,(0.,0.,.44),5,.5,1.,.1,.001,.002)
    explorer.replan(np.zeros(3),3.)
    assert explorer.frontier_decision['route_cost_preempted']
