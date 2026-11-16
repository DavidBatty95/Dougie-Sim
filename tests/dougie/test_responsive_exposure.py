from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.exposure_trend import ExposureTrend, ExposureCostView
from dougie_sim.mission_utility import ExposureUtility
from dougie_sim.radiation import RadiationSample


def sample(index,count,position=None):
    exposure = .5*50000./3600.
    return RadiationSample(index*.5,position or (index*.1,0.,.44),count,.5,
        count/exposure,(count+1.)/exposure**2,.001,.002)


def rising():
    trend = ExposureTrend()
    last = None
    for index,count in enumerate((2,3,5,10,20,40,80,160),1):
        last = sample(index,count)
        trend.observe(last)
    return trend,last


def test_rising_counts_warn_ahead_without_inventing_distant_or_backward_hazards():
    trend,last = rising()
    assert trend.active and trend.gradient[0] > 0
    points = np.array([last.position,np.asarray(last.position)+[.4,0,0],
                       np.asarray(last.position)+[-.5,0,0],np.asarray(last.position)+[2.,0,0]])
    cost = trend.adjust(points,np.ones(4))
    assert cost[0] > 1 and cost[1] > cost[0]
    np.testing.assert_array_equal(cost[2:],[1.,1.])
    trend.observe(last)  # Replaying the same packet cannot erase/re-assimilate the warning.
    assert trend.active


def test_flat_or_stationary_data_do_not_invent_a_spatial_gradient():
    flat,stationary = ExposureTrend(),ExposureTrend()
    for index,count in enumerate((2,3,5,10,20,40,80,160),1):
        flat.observe(sample(index,20))
        stationary.observe(sample(index,count,(0.,0.,.44)))
    assert not flat.active and not stationary.active
    assert stationary.anchor_lower > 0  # Real high counts still charge local turning/dwell.


def test_cost_view_does_not_edit_the_gp_map_and_invalid_pose_clears_guard():
    trend,last = rising()
    gp = SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),.2),np.full(len(p),2.)))
    points = [np.asarray(last.position)+[.4,0,0]]
    assert ExposureCostView(gp,trend).predict(points)[0][0] > 1.
    assert gp.predict(points)[0][0] == 1.
    trend.observe(replace(last,stamp_s=last.stamp_s+.5,position_valid=False))
    np.testing.assert_array_equal(trend.adjust(points,[1.]),[1.])


def test_neighbourhood_revisits_cross_voxel_boundaries_but_not_other_floors():
    utility = ExposureUtility()
    point = [.501,0.,.44]
    before = utility.frontier_score(point,4.,1.,10.,.01,1.,1.,3.)
    for index in range(20):
        utility.observe(sample(index+1,10,(.499,0.,.44)))
    after = utility.frontier_score(point,4.,1.,10.,.01,1.,1.,3.)
    assert after['observed_seconds'] > 0 and after['score'] < before['score']
    assert utility.neighbourhood_seconds([.501,0.,1.64]) == 0.


def hot_planner():
    explorer = ClusteredDoseExplorer(mission_duration=1e12)
    explorer.budget_enabled = explorer.rate_limited = False
    explorer.survey_until_complete = True
    explorer.exposure_policy = 'responsive'
    explorer.observed_rates.append(1.)
    explorer.gp = SimpleNamespace()
    explorer.sample = replace(sample(20,1400),position=(0.,0.,.44))
    explorer.target = explorer.goal_progress_focus = np.array([1.,0.,0.])
    explorer.goal_progress_best = 1.
    explorer.goal_progress_stamp = explorer.last_support_progress_s = 0.
    explorer.frontier_count = 2
    return explorer


def test_hot_unproductive_goal_is_temporarily_deferred_not_called_finished():
    explorer = hot_planner()
    assert explorer.defer_hot_stalled_goal(np.zeros(3),10.,1,{1:200.})
    assert explorer.failed and explorer.hot_goal_deferrals == 1
    assert explorer.frontier_count == 2 and not explorer.frontier_complete


def test_cool_escape_turn_and_fresh_mapping_progress_are_not_aborted():
    escape = hot_planner()
    assert not escape.defer_hot_stalled_goal(np.zeros(3),10.,1,{1:1.})
    turning = hot_planner()
    turning.goal_progress_yaw,turning.start_yaw = 0.,.5
    assert not turning.defer_hot_stalled_goal(np.zeros(3),10.,1,{1:200.})
    mapping = hot_planner()
    mapping.last_support_progress_s = 9.
    assert not mapping.defer_hot_stalled_goal(np.zeros(3),10.,1,{1:200.})


def test_blind_ablation_does_not_receive_exposure_guard_information():
    from dougie_sim.planning_worker import create_explorer
    from dougie_sim.radiation_blind import blind_sample
    blind = create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',
        dict(no_dose_budget=True,radiation_policy='minimize-dose',radiation_awareness='blind',exposure_policy='responsive'))
    for index,count in enumerate((2,3,5,10,20,40,80,160),1):
        blind.observe_radiation(blind_sample(sample(index,count)))
    assert not blind.exposure_trend.active and blind.exposure_trend.anchor_lower == 0.
    np.testing.assert_array_equal(blind.planning_gp.predict([[.8,0,.44],[1.,0,.44]])[0],[0.,0.])
