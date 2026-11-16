from types import SimpleNamespace

import numpy as np
import pytest

from dougie_sim.radiation_exclusion import RadiationExclusion
from dougie_sim.terrain import SurfaceMap


def model(rate=200., sigma=.1):
    return SimpleNamespace(likelihood='poisson',
        predict_log=lambda p:(np.full(len(p),np.log(rate)),np.full(len(p),sigma**2)),
        predict=lambda p:(np.full(len(p),rate*np.exp(.5*sigma**2)),
            np.full(len(p),sigma),np.full(len(p),rate*np.exp(2*sigma))),metrics=lambda:{})


def test_hotspot_requires_both_lower_bound_and_local_evidence():
    gate = RadiationExclusion(100.,.95)
    gate.refresh(model(),SimpleNamespace(neighbourhood_seconds=lambda p:0.))
    lower,blocked = gate.evaluate([[0.,0.,.44]])
    assert lower[0] > 100. and not blocked[0]
    gate.refresh(model(),SimpleNamespace(neighbourhood_seconds=lambda p:1.))
    assert gate.evaluate([[0.,0.,.44]])[1][0]
    gate.refresh(model(sigma=1.),SimpleNamespace(neighbourhood_seconds=lambda p:1.))
    # A high mean/upper bound alone cannot certify a hot region.
    assert not gate.evaluate([[0.,0.,.44]])[1][0]


def test_higher_confidence_is_stricter_and_new_observations_can_reopen_region():
    evidence = SimpleNamespace(neighbourhood_seconds=lambda p:1.)
    ordinary,strict = RadiationExclusion(100.,.95),RadiationExclusion(100.,.99)
    for gate in (ordinary,strict):
        gate.refresh(model(sigma=.15),evidence)
    assert ordinary.evaluate([[0.,0.,.44]])[1][0]
    assert not strict.evaluate([[0.,0.,.44]])[1][0]
    ordinary.refresh(model(rate=2.),evidence)
    assert not ordinary.evaluate([[0.,0.,.44]])[1][0]


def test_graph_exclusion_finds_cool_alternative_and_keeps_physical_connectivity():
    terrain = SurfaceMap()
    terrain.nodes = {0:np.array([0.,0.,0.]),1:np.array([1.,0.,0.]),
                     2:np.array([0.,1.,0.]),3:np.array([1.,1.,0.])}
    terrain.graph = {0:[1,2],1:[0,3],2:[0,3],3:[1,2]}
    parent = terrain.routes(0,excluded={1})[3]
    assert terrain.path(parent,3) == [0,2,3]
    assert 1 not in parent
    assert terrain.path(terrain.routes(0)[3],1) == [0,1]
    # Starting in a newly identified hotspot must still permit escape.
    assert 2 in terrain.routes(0,excluded={0,1})[3]


def test_hot_corridor_is_radiation_blocked_not_geometrically_disconnected():
    from dougie_sim.survey_comparison import survey_admission,RADIATION,AVAILABLE
    terrain = SurfaceMap()
    terrain.nodes = {k:np.array([float(k),0.,0.]) for k in range(3)}
    terrain.graph = {0:[1],1:[0,2],2:[1]}
    _,dose,_,parent = terrain.routes(0,excluded={1})
    explorer = SimpleNamespace(terrain=terrain,confident_hot_keys={1},rate_limited=False,
        failed=[],wait_allowance=3.,recovery_reserve_s=3.,budget_enabled=False,
        sample=SimpleNamespace(dose_upper_usv=0.),mission_duration=1e12)
    status = survey_admission(explorer,list(terrain.nodes),0,0,dict.fromkeys(range(3),0.),
        parent,parent,dose,dose,dict.fromkeys(parent,0.),dict.fromkeys(parent,0.),1.)
    assert status == {0:AVAILABLE,1:RADIATION,2:RADIATION}


def test_entire_hot_frontier_set_returns_incomplete_and_keeps_stop_reason(monkeypatch):
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.radiation import RadiationSample
    value = ClusteredDoseExplorer(mission_duration=1e12)
    value.budget_enabled = value.rate_limited = False
    value.survey_until_complete = True
    points = np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.gp = model()
    value.sample = RadiationSample(1.,(0.,0.,.44),100,.5,200.,1.,0.,0.)
    value.radiation_exclusion = RadiationExclusion(100.,.95)
    monkeypatch.setattr(value.utility,'neighbourhood_seconds',lambda point:1.)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(value=lambda k:1,values=lambda keys:np.ones(len(keys)),cache={}))
    value.replan(np.zeros(3),1.)
    assert value.returning and value.last_reason == 'radiation_frontiers_blocked'
    assert value.frontier_count > 0 and not value.frontier_complete
    assert value.radiation_frontier_counts['radiation_blocked'] == value.frontier_count
    value.last_reason = 'route_tracking_lost'
    assert value.metrics()['decision_reason'] == 'radiation_frontiers_blocked'
    assert value.metrics()['return_navigation_reason'] == 'route_tracking_lost'


def test_blind_worker_has_no_hotspot_exclusion_even_with_identical_settings():
    from dougie_sim.planning_worker import create_explorer
    options = dict(no_dose_budget=True,radiation_policy='minimize-dose',
        frontier_rate_limit=100.,frontier_confidence=.95)
    blind = create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',
        dict(options,radiation_awareness='blind'))
    aware = create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    assert blind.radiation_exclusion is None
    assert aware.radiation_exclusion.rate_limit == 100.


def test_execution_shortcut_cannot_reenter_hotspot_but_return_can_escape(monkeypatch):
    from dougie_sim.path_motion import MotionRoute,RouteSmoother
    terrain = SimpleNamespace(nodes={},hits={},resolution=.2)
    shortcut = [np.array([0.,0.,0.]),np.array([1.,0.,0.])]
    monkeypatch.setattr('dougie_sim.path_motion.smooth_route',lambda *a,**kw:
        MotionRoute(shortcut,[[0,1]],dict(mode='support_checked')))
    admission = lambda p:all(v[0] < .4 or v[1] > .5 for v in p)
    smoother = RouteSmoother(terrain,None,float('inf'),admission=admission)
    raw = [[0.,0.,0.],[0.,1.,0.],[1.,1.,0.]]
    checked = smoother(raw)
    np.testing.assert_allclose(checked.points,raw)
    assert checked.metrics['fallback_reason'] == 'confident_radiation_exclusion'
    assert smoother(shortcut).points == []
    np.testing.assert_allclose(smoother(shortcut,returning=True).points,shortcut)


def test_geometry_rejection_is_not_mislabelled_as_radiation(monkeypatch):
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.path_motion import MotionRoute
    from dougie_sim.radiation import RadiationSample
    value = ClusteredDoseExplorer(mission_duration=1e12)
    value.budget_enabled = value.rate_limited = False
    value.survey_until_complete = value.smooth_execution = True
    points = np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.gp = model(rate=1.)
    value.sample = RadiationSample(1.,(0.,0.,.44),5,.5,1.,1.,0.,0.)
    value.radiation_exclusion = RadiationExclusion()
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(value=lambda k:1,values=lambda keys:np.ones(len(keys)),cache={}))
    monkeypatch.setattr('dougie_sim.path_motion.RouteSmoother',lambda *a,**kw:
        lambda points,returning=False:MotionRoute(points if returning else [],[],
            dict(mode='empty',fallback_reason='geometry_revalidation_failed')))
    value.replan(np.zeros(3),1.)
    assert value.returning and value.last_reason == 'unresolved_frontiers'
    assert not value.frontier_complete


@pytest.mark.parametrize('rate,confidence',[(0.,.95),(float('inf'),.95),(100.,.5),(100.,1.)])
def test_invalid_exclusion_settings_are_rejected(rate,confidence):
    with pytest.raises(ValueError):
        RadiationExclusion(rate,confidence)
