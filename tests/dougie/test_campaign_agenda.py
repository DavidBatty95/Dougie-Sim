import json
from types import SimpleNamespace

import numpy as np

from dougie_sim.campaign_agenda import CampaignAgenda
from dougie_sim.campaign_memory import save_memory,restore_memory
from dougie_sim.frontier_views import FrontierViews
from dougie_sim.planning_worker import create_explorer


def test_three_dimensional_regions_and_connectors_are_distinct():
    agenda=CampaignAgenda()
    for position in ([1.,1.,0.],[1.,1.,.4],[1.,1.,.8],[1.,1.,2.4]):
        agenda.observe_position(position)
    assert len(agenda.connectors)==3
    assert agenda.region([1.,1.,0.])!=agenda.region([1.,1.,2.4])


def test_bounded_screening_spans_regions_and_does_not_repeat_rejected_keys():
    agenda=CampaignAgenda()
    nodes={i:np.array([float(i%4),float(i//4),0. if i<8 else 2.4]) for i in range(12)}
    length={i:float(i+1) for i in nodes}
    gain={i:20 if i>=8 else 1 for i in nodes}
    first=agenda.screening_candidates(list(nodes),nodes,length,gain,set(),limit=8)
    second=agenda.screening_candidates(list(nodes),nodes,length,gain,set(first),limit=8)
    assert len(first)==8 and len(second)==4 and not set(first)&set(second)
    assert {agenda.region(nodes[k])[2] for k in first}=={0,6}


def test_extra_screening_wait_never_consumes_return_reserve_or_deadline():
    check=CampaignAgenda.can_wait_for_screening
    assert check(.1,.5,5.,2.,100.,100.,3.,1000.)
    assert not check(.1,1.89,20.,2.,100.,100.,3.,1000.)
    assert not check(.1,.5,5.,2.,880.,100.,3.,1000.)


def test_budget_screening_wait_is_bounded_and_never_declares_completion(monkeypatch):
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.radiation import RadiationSample
    from dougie_sim.survey_comparison import AVAILABLE,BUDGET
    value=ClusteredDoseExplorer(budget=2.,mission_duration=1e12)
    value.rate_limited=False
    value.survival_mode=value.survey_until_complete=value.smooth_execution=True
    value.campaign_agenda=CampaignAgenda()
    value.radiation_awareness='blind'
    points=np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.gp=SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),.1),np.full(len(p),1.2)))
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(values=lambda keys:np.ones(len(keys)),value=lambda key:1,cache={}))
    monkeypatch.setattr('dougie_sim.survey_comparison.survey_admission',
        lambda explorer,keys,root,*args:{k:AVAILABLE if k==root else BUDGET for k in keys})
    monkeypatch.setattr(value.campaign_agenda,'screening_candidates',lambda *args:[])
    for index,stamp in enumerate((1.,3.,5.)):
        value.sample=RadiationSample(stamp,(0.,0.,.44),5,.5,1.,.1,0.,0.)
        value.replan(np.zeros(3),stamp)
        assert value.last_reason=='screening_budget_frontiers' and not value.returning
        assert value.budget_screen_cycles==index+1 and not value.frontier_complete
    for stamp in (7.,9.,11.,13.):
        value.sample=RadiationSample(stamp,(0.,0.,.44),5,.5,1.,.1,0.,0.)
        value.replan(np.zeros(3),stamp)
        assert value.last_reason!='screening_budget_frontiers'
        if value.returning:
            break
    assert value.returning and value.frontier_count>0 and not value.frontier_complete


def test_unproductive_region_can_be_deprioritized_without_removing_frontiers():
    agenda=CampaignAgenda()
    for _ in range(3):
        agenda.begin([1.,1.,0.],100)
        agenda.finish(100)
    nodes={0:np.array([1.,1.,0.]),1:np.array([4.,1.,2.4])}
    ranked=[(5.,0,3),(4.,1,3)]
    _,selected,_=agenda.choose(ranked,nodes,{0:20.,1:20.},{0:.1,1:.1},{0:.1,1:.1})
    assert selected==1 and len(ranked)==2


def test_attempts_and_view_signatures_survive_replacement(tmp_path):
    options=dict(no_dose_budget=True,radiation_policy='minimize-dose',radiation_awareness='blind',campaign_strategy='agenda')
    original=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    points=np.array([[x,y,0.] for x in np.arange(-1.5,1.5,.1) for y in np.arange(-1.5,1.5,.1)])
    original.observe_geometry(points,points)
    original.terrain.build_graph()
    original.campaign_agenda.observe_position([0.,0.,0.])
    original.campaign_agenda.observe_position([0.,0.,.5])
    original.campaign_agenda.begin([1.,1.,2.4],0)
    for scan in (1,2,3):
        original.frontier_views.observe({(5,0,60)},[0.,0.,2.4],scan)
    save_memory(original,tmp_path/'campaign_memory.npz')
    restored=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    restore_memory(restored,tmp_path,[0.,0.,0.])
    assert restored.campaign_agenda.metrics()['attempts']==1
    assert restored.campaign_agenda.metrics()['connector_transitions']==1
    assert restored.frontier_views.marginal_gain({(5,0,60)},[0.,0.,2.4])==0
    assert restored.frontier_views.last_scan is None
    json.dumps(restored.campaign_agenda.metrics())


def test_tour_lookahead_respects_budget_exclusions_and_distinct_information():
    from dougie_sim.terrain import SurfaceMap
    terrain=SurfaceMap()
    terrain.nodes={0:np.array([0.,0.,0.]),1:np.array([1.,0.,0.]),2:np.array([2.,0.,0.])}
    terrain.graph={0:[1],1:[0,2],2:[1]}
    agenda=CampaignAgenda()
    ranked=[(3.,1,2),(2.,2,2)]
    rates={0:1.,1:1.,2:1.}
    cells={1:{(1,0,0),(1,1,0)},2:{(2,0,0),(2,1,0)}}
    args=(ranked,terrain,rates,{1:10.,2:20.},{1:.01,2:.02},{1:.01,2:.02})
    tour=agenda.select_tour(*args,1.,.1,3.,lambda k:cells[k])
    assert tour and tour[1:3]==(1,2) and tour[3]<1.
    assert agenda.select_tour(*args,.001,.1,3.,lambda k:cells[k]) is None
    assert agenda.select_tour(*args,1.,.1,3.,lambda k:cells[k],excluded={2}) is None
    assert agenda.select_tour(*args,1.,.1,3.,lambda k:cells[1]) is None


def test_uncertainty_probes_require_blocked_target_and_observed_unmeasured_same_floor():
    from dougie_sim.uncertainty_probe import probe_candidates
    nodes={0:np.array([0.,0.,0.]),1:np.array([.5,0.,0.]),2:np.array([1.,0.,0.]),3:np.array([.5,0.,2.4])}
    status={0:1,1:1,2:3,3:1}
    uncertainty={2:1.5}
    lengths={0:0.,1:1.,2:2.,3:1.}
    utility=SimpleNamespace(neighbourhood_seconds=lambda p:0.)
    assert probe_candidates(nodes,{2},status,uncertainty,lengths,utility,1,{3})==[1]
    assert probe_candidates(nodes,{2},status,{2:.2},lengths,utility,1,{3})==[]
    assert probe_candidates(nodes,{2},status,uncertainty,lengths,SimpleNamespace(neighbourhood_seconds=lambda p:1.),1,{3})==[]
    assert probe_candidates(nodes,{2},status,uncertainty,lengths,utility,1,{4})==[]


def test_probe_considers_all_nearby_targets_not_only_nearest_wrong_height_or_distance():
    from dougie_sim.uncertainty_probe import probe_candidates
    # The nearest blocked target is too close (<35 cm); a farther target on
    # the same level is informative. Nearest-only selection hid the valid one.
    nodes={0:np.array([0.,0.,0.]),1:np.array([.5,0.,0.]),
           2:np.array([.55,0.,.2]),3:np.array([1.,0.,0.])}
    result=probe_candidates(nodes,{2,3},{0:1,1:1,2:3,3:3},{2:1.5,3:1.5},
        {0:0.,1:1.,2:2.,3:2.},SimpleNamespace(neighbourhood_seconds=lambda p:0.),1,{3})
    assert result==[1]


def test_probe_is_dispatched_only_for_aware_and_does_not_complete_frontier(monkeypatch):
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.radiation import RadiationSample
    from dougie_sim.survey_comparison import AVAILABLE,BUDGET
    def build(awareness):
        value=ClusteredDoseExplorer(mission_duration=1e12)
        value.budget_enabled=value.rate_limited=False
        value.survey_until_complete=True
        value.campaign_agenda=CampaignAgenda()
        value.radiation_awareness=awareness
        value.terrain.nodes={0:np.array([0.,0.,0.]),1:np.array([.5,0.,0.]),2:np.array([1.,0.,0.])}
        value.terrain.graph={0:[1],1:[0,2],2:[1]}
        value.terrain.build_graph=lambda:(value.terrain.nodes,value.terrain.graph)
        value.gp=SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),1.5),np.full(len(p),2.)))
        value.sample=RadiationSample(1.,(0.,0.,.44),5,.5,1.,.1,0.,0.)
        return value
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(values=lambda keys:np.array([int(k==2) for k in keys]),value=lambda k:int(k==2),cache={}))
    monkeypatch.setattr('dougie_sim.survey_comparison.survey_admission',lambda *args:{0:AVAILABLE,1:AVAILABLE,2:BUDGET})
    aware=build('aware')
    aware.replan(np.zeros(3),1.)
    assert aware.last_reason=='uncertainty_measurement_probe'
    np.testing.assert_allclose(aware.target,[.5,0.,0.])
    assert aware.frontier_count==1 and not aware.frontier_complete
    # New evidence near the stand-off should not abandon an in-flight probe if
    # the blocked target remains uncertain and the route is still admissible.
    aware.utility.neighbourhood_seconds=lambda p:1.
    aware.sample=RadiationSample(3.,(.1,0.,.44),5,.5,1.,.1,0.,0.)
    aware.replan(np.array([.1,0.,0.]),3.)
    np.testing.assert_allclose(aware.target,[.5,0.,0.])
    assert aware.last_reason=='continue_survey_sweep' and not aware.returning
    # Revalidation must not keep a probe whose blocked target is now certain.
    aware.gp.predict=lambda p:(np.ones(len(p)),np.full(len(p),.2),np.full(len(p),2.))
    aware.sample=RadiationSample(5.,(.1,0.,.44),5,.5,1.,.1,0.,0.)
    aware.replan(np.array([.1,0.,0.]),5.)
    assert aware.returning and not aware.frontier_complete
    blind=build('blind')
    blind.replan(np.zeros(3),1.)
    assert blind.uncertainty_probe_count==0 and blind.returning
    assert blind.frontier_count==1 and not blind.frontier_complete


def test_uncertain_target_revalidation_respects_status_height_and_current_uncertainty():
    from dougie_sim.uncertainty_probe import target_still_uncertain
    nodes={1:np.array([1.,0.,0.])}
    assert target_still_uncertain([.5,0.,0.],nodes,{1},{1:3},{1:1.5},{3})
    assert not target_still_uncertain([.5,0.,0.],nodes,{1},{1:3},{1:.5},{3})
    assert not target_still_uncertain([.5,0.,0.],nodes,{1},{1:1},{1:1.5},{3})
    assert not target_still_uncertain([.5,0.,2.4],nodes,{1},{1:3},{1:1.5},{3})
