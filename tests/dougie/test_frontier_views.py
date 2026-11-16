import numpy as np

from dougie_sim.frontier_views import FrontierViews


def test_three_fresh_views_reduce_reward_without_erasing_unknown_cells():
    views = FrontierViews(.2)
    unknown = frozenset({(5,0,0),(5,1,0)})
    point = [0.,0.,0.]
    assert views.marginal_gain(unknown,point)==2
    for scan in (1,2):
        views.observe(unknown,point,scan)
    assert views.marginal_gain(unknown,point)==2
    views.observe(unknown,point,2)  # Duplicate planner updates are not fresh scans.
    assert views.marginal_gain(unknown,point)==2
    views.observe(unknown,point,3)
    assert views.marginal_gain(unknown,point)==0
    assert len(unknown)==2
    assert views.marginal_gain(unknown,[2.,0.,0.])==2
    assert views.marginal_gain(unknown,[.7,0.,0.])==2
    assert views.marginal_gain({(5,0,60)},point)==1


def test_view_memory_is_bounded_and_eviction_reopens_views():
    views = FrontierViews(.2,capacity=2)
    for scan in (1,2,3):
        views.observe({(5,0,0),(6,0,0)},[0.,0.,0.],scan)
    views.observe({(7,0,0)},[0.,0.,0.],4)
    assert len(views.views)==2
    assert views.marginal_gain({(5,0,0)},[0.,0.,0.])==1


def test_unproductive_views_return_incomplete_with_frontiers_still_recorded(monkeypatch):
    from types import SimpleNamespace
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.radiation import RadiationSample
    value = ClusteredDoseExplorer(mission_duration=1e12)
    value.budget_enabled = value.rate_limited = False
    value.survey_until_complete = True
    value.terrain.nodes = {0:np.array([0.,0.,0.]),1:np.array([1.,0.,0.])}
    value.terrain.graph = {0:[1],1:[0]}
    value.terrain.build_graph = lambda:(value.terrain.nodes,value.terrain.graph)
    value.gp = SimpleNamespace(predict=lambda p:(np.zeros(len(p)),np.zeros(len(p)),np.zeros(len(p))))
    value.sample = RadiationSample(1.,(0.,0.,.44),0,.5,0.,0.,0.,0.)
    value.frontier_views = SimpleNamespace(observe=lambda *args:None,marginal_gain=lambda *args:0)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(value=lambda k:1,values=lambda keys:np.ones(len(keys)),unknown_cells=lambda k:{(5,0,0)}))
    value.replan(np.zeros(3),1.)
    assert value.returning and value.last_reason=='frontier_views_unproductive'
    assert value.frontier_count==2 and value.unproductive_frontier_count==2
    assert not value.frontier_complete


def test_native_frontier_gains_remain_json_serializable_in_raw_profile():
    import json
    from dougie_sim.planning_worker import create_explorer
    from dougie_sim.radiation import RadiationSample
    value = create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',
        dict(no_dose_budget=True,radiation_policy='minimize-dose',survey_until_complete=True,radiation_awareness='blind'))
    points = np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.observe_radiation(RadiationSample(1.,(0.,0.,.44),0,.5,0.,0.,0.,0.))
    value.replan(np.zeros(3),1.)
    payload = json.loads(json.dumps(value.metrics()))
    assert payload['frontier_count']>0
    assert isinstance(payload['unproductive_frontier_count'],int)
