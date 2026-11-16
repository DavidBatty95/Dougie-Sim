import json
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from dougie_sim.campaign import RadiationLifetime,RadiationCampaign
from dougie_sim.simulation import Simulation
from dougie_sim.wrecks import freeze_robot,origin_blocked


def test_lifetime_failure_is_latched_and_dose_cannot_reset_in_place():
    robot=RadiationLifetime(2.)
    assert not robot.update(1.,10.,[1.,0.,.4])
    assert robot.payload()['remaining_fraction']==.5
    assert robot.update(2.,20.,[2.,0.,.4])
    assert not robot.payload()['alive'] and robot.payload()['death']['position']==[2.,0.,.4]
    with pytest.raises(ValueError):
        robot.update(0.,21.,[0.,0.,.4])
    assert RadiationLifetime(2.).payload()['alive']  # A replacement is a NEW robot.
    assert robot.payload()['absorbed_usv']==2.


@pytest.mark.parametrize('limit',[0.,-1.,float('nan'),float('inf'),True,'2'])
def test_invalid_lifetime_is_rejected(limit):
    with pytest.raises(ValueError):
        RadiationLifetime(limit)


def test_disabled_robot_stays_in_situ_and_frozen_geometry_matches_articulated_pose():
    old=Simulation('flat',controller='stand')
    old.data.qpos[:3]=[1.5,0.,.445]
    old.data.qpos[7:]+=np.linspace(-.1,.1,12)
    mujoco.mj_forward(old.model,old.data)
    old.disable_in_place()
    pose,time=old.data.qpos.copy(),old.data.time
    old.step([.5,0.,0.])
    np.testing.assert_array_equal(old.data.qpos,pose)
    assert old.data.time==time and old.powered_off
    wreck=freeze_robot(old,'test')
    new=Simulation('flat',controller='stand',wrecks=[wreck])
    assert new.model.nq==old.model.nq and new.model.nv==old.model.nv
    for i,index in enumerate(sorted(old.robot_geoms)):
        frozen=new.model.geom(f'wreck_test_geom_{i}').id
        np.testing.assert_allclose(new.data.geom_xpos[frozen],old.data.geom_xpos[index],atol=1e-7)
        np.testing.assert_allclose(new.data.geom_xmat[frozen],old.data.geom_xmat[index],atol=1e-7)
        assert frozen not in new.robot_geoms


def test_wreck_is_seen_by_ranges_and_collides_with_the_replacement():
    old=Simulation('flat',controller='stand')
    old.data.qpos[:3]=[1.5,0.,.445]
    mujoco.mj_forward(old.model,old.data)
    wreck=freeze_robot(old,'collision')
    sim=Simulation('flat',controller='stand',wrecks=[wreck])
    index=np.zeros(1,dtype=np.int32)
    distance=mujoco.mj_ray(sim.model,sim.data,np.array([0.,0.,.445]),np.array([1.,0.,0.]),
        np.array([1,1,0,1,0,1],dtype=np.uint8),True,sim.base,index)
    assert 1.<distance<1.6 and sim.model.geom(index[0]).name.startswith('wreck_')
    sim.data.qpos[:3]=[1.5,0.,.445]
    mujoco.mj_forward(sim.model,sim.data)
    assert any((int(c.geom1) in sim.robot_geoms)!=(int(c.geom2) in sim.robot_geoms) for c in sim.data.contact)


def artifacts(root,dead_blind=True,home=.1):
    summaries={}
    for mode in ('blind','aware'):
        directory=root/mode
        directory.mkdir(exist_ok=True)
        for name in ('campaign_memory.npz','gp_posterior.npz','coverage_reference.npz','minimap_final.npz'):
            np.savez_compressed(directory/name,schema_version=1)
        dead=mode=='blind' and dead_blind
        summaries[mode]=dict(robot_lifetime=dict(alive=not dead),true_dose_usv=2. if dead else .6,
            distance_3d_m=30.,simulation=dict(simulation_time_s=100.),comparison_survey=dict(explored_percent=50.),
            end_state='dead_radiation' if dead else 'returned',error=None,frontier_complete=False,
            localization_evaluation=dict(true_home_distance_m=3. if dead else home))
        if dead:
            (directory/'dead_robot.json').write_text(json.dumps(dict(id='c1_r1',position=[2.,0.,.4],
                bounds=[[1.5,-.3,0.],[2.5,.3,.6]],xml='<body name="wreck_stub"/>')))
    return summaries


def test_campaign_resets_only_new_robot_allowance_and_retains_losses_maps_and_totals(tmp_path):
    campaign=RadiationCampaign()
    first=campaign.prepare(tmp_path,'same-protocol')
    assert json.loads(first['aware'].read_text())['robot_id']==1
    record=dict(id=1)
    campaign.finish(record,tmp_path,artifacts(tmp_path))
    assert campaign.status=='active'
    assert campaign.modes['blind']['robots_lost']==1 and len(campaign.modes['blind']['wrecks'])==1
    assert campaign.modes['aware']['robots_recovered']==1
    assert campaign.modes['blind']['total_dose_usv']==2.
    assert campaign.modes['aware']['total_dose_usv']==.6
    second=tmp_path/'second'
    second.mkdir()
    paths=campaign.prepare(second,'same-protocol')
    for mode in ('blind','aware'):
        data=json.loads(paths[mode].read_text())
        assert data['robot_id']==2 and data['lifetime_dose_usv']==2.
        assert data['previous_directory']==str((tmp_path/mode).resolve())
    assert json.loads(paths['blind'].read_text())['prior_dose_usv']==2.
    assert len(json.loads(paths['blind'].read_text())['wrecks'])==1
    assert json.loads(paths['aware'].read_text())['wrecks']==[]
    restored=RadiationCampaign.load(json.loads(json.dumps(campaign.save())))
    assert restored.payload()==campaign.payload()
    campaign.finish(record,tmp_path,artifacts(tmp_path))
    assert campaign.modes['blind']['robots_lost']==1  # No duplicate failure/replacement.


def test_return_claim_away_from_origin_does_not_allow_dose_reset(tmp_path):
    campaign=RadiationCampaign()
    campaign.prepare(tmp_path)
    campaign.finish(dict(id=1),tmp_path,artifacts(tmp_path,dead_blind=False,home=2.))
    assert campaign.status=='halted'
    assert campaign.modes['aware']['robots_recovered']==0
    assert campaign.modes['aware']['total_dose_usv']==.6
    with pytest.raises(ValueError):
        campaign.prepare(tmp_path)


def test_origin_wreck_blocks_further_deployment(tmp_path):
    campaign=RadiationCampaign()
    campaign.prepare(tmp_path)
    summaries=artifacts(tmp_path)
    wreck=json.loads((tmp_path/'blind/dead_robot.json').read_text())
    wreck['bounds']=[[-.4,-.3,0.],[.4,.3,.6]]
    (tmp_path/'blind/dead_robot.json').write_text(json.dumps(wreck))
    campaign.finish(dict(id=1),tmp_path,summaries)
    assert campaign.status=='halted' and 'origin' in campaign.reason
    assert origin_blocked(campaign.modes['blind']['wrecks'])


def test_package_sortie_count_is_not_mission_completion(tmp_path):
    campaign=RadiationCampaign(sorties=1)
    campaign.prepare(tmp_path)
    campaign.finish(dict(id=1),tmp_path,artifacts(tmp_path,dead_blind=False))
    assert campaign.status=='sortie_limit'
    assert campaign.history[0]['results']['aware']['outcome']=='recovered_for_swap'


def test_runtime_change_cannot_silently_reuse_campaign_memory(tmp_path):
    campaign=RadiationCampaign()
    campaign.prepare(tmp_path,'original')
    with pytest.raises(ValueError,match='configuration changed'):
        campaign.prepare(tmp_path,'changed')
    assert campaign.status=='halted'


def test_campaign_memory_retains_field_geometry_and_views_but_not_robot_dose(tmp_path):
    from dougie_sim.planning_worker import create_explorer
    from dougie_sim.campaign_memory import save_memory
    from dougie_sim.radiation import RadiationSample
    options=dict(gp_mode='windowed',gp_expert_capacity=8,gp_max_experts=2,survival_mode=True,
        radiation_policy='minimize-dose',radiation_awareness='aware')
    original=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    points=np.array([[x,y,0.] for x in np.arange(-1.5,1.5,.1) for y in np.arange(-1.5,1.5,.1)])
    original.observe_geometry(points,points)
    original.terrain.build_graph()
    original.visited.update(list(original.terrain.nodes)[:3])
    original.observe_radiation(RadiationSample(10.,(0.,0.,.44),20,.5,2.88,.1,.8,.9))
    save_memory(original,tmp_path/'campaign_memory.npz')
    original.gp.save(tmp_path/'gp_posterior.npz')
    query=[[0.,0.,.44],[.5,.5,.44]]
    expected=original.gp.predict(query)
    restored=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',
        dict(options,campaign_previous=str(tmp_path),campaign_origin=[0.,0.,0.]))
    assert restored.sample is None and restored.budget_enabled and restored.expected_dose_objective
    assert restored.exceeds_budget(2.1)
    assert restored.visited and restored.utility.exposure==original.utility.exposure
    np.testing.assert_array_equal(restored.home,[0.,0.,0.])
    for actual,before in zip(restored.gp.predict(query),expected):
        np.testing.assert_allclose(actual,before)
    blind=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',
        dict(options,radiation_awareness='blind',campaign_previous=str(tmp_path),campaign_origin=[0.,0.,0.]))
    assert blind.visited and all(np.all(v==0.) for v in blind.gp.predict(query))


def test_gp_geometry_clock_is_rebased_once_without_forgetting_counts(tmp_path):
    from dougie_sim.windowed_gp import WindowedGP
    gp=WindowedGP(capacity=8,max_experts=1,max_planes=0,store_dir=tmp_path)
    gp.update_counts([2.,2.,.75],10,.5)
    key=(0,0,0)
    gp._get(key).last_geometry_stamp=100.
    expected=gp.predict([[2.,2.,.75]])
    gp.start_new_mission()
    assert gp._get(key).last_geometry_stamp==-float('inf')
    gp._get(key).last_geometry_stamp=1.
    assert gp._get(key).last_geometry_stamp==1.
    np.testing.assert_allclose(gp.predict([[2.,2.,.75]]),expected)


def test_fixed_floor_reference_and_observed_progress_survive_new_sortie(tmp_path):
    from dougie_sim.comparison_stream import FloorCoverage
    sim=Simulation('reactor',controller='stand')
    coverage=FloorCoverage(sim,'reactor')
    coverage.seen[:10]=True
    coverage.reached_levels[:]=True
    np.savez_compressed(tmp_path/'reference.npz',xyz=coverage.points,flat=coverage.flat,
        level_ids=coverage.level_ids,level_heights_m=coverage.level_heights)
    np.savez_compressed(tmp_path/'progress.npz',xyz=coverage.points,seen=coverage.seen,reached_levels=coverage.reached_levels)
    restored=FloorCoverage.retained(tmp_path/'reference.npz',tmp_path/'progress.npz')
    np.testing.assert_array_equal(restored.points,coverage.points)
    np.testing.assert_array_equal(restored.seen,coverage.seen)
    assert restored.percent==coverage.percent


def survival_planner():
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.radiation import RadiationSample
    value=ClusteredDoseExplorer(budget=.03,mission_duration=1e12)
    value.rate_limited=False
    value.survival_mode=value.survey_until_complete=value.smooth_execution=True
    points=np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.gp=SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),.1),np.full(len(p),1.1)))
    value.sample=RadiationSample(1.,(0.,0.,.44),5,.5,1.,0.,0.,0.)
    value.observed_rates.append(1.)
    return value


def test_survival_admission_rechecks_executable_trips_before_returning_at_base(monkeypatch):
    from dougie_sim.clustered_planner import forecast_tree
    value=survival_planner()
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
        SimpleNamespace(value=lambda k:1,values=lambda keys:np.ones(len(keys)),cache={}))
    def pessimistic_grid(*args,**kwargs):
        duration,dose,peak=forecast_tree(*args,**kwargs)
        return ({k:v+(360. if duration[k]>0 else 0.) for k,v in duration.items()},
                {k:v+(.1 if duration[k]>0 else 0.) for k,v in dose.items()},peak)
    monkeypatch.setattr('dougie_sim.clustered_planner.forecast_tree',pessimistic_grid)
    value.replan(np.zeros(3),1.)
    assert not value.returning and value.state=='exploring' and value.path
    assert 0<value.route_forecast['checked_execution_candidates']<=8
    assert value.route_forecast['outbound_upper_dose_usv']+value.route_forecast['goal_return_upper_dose_usv']<value.budget


def test_execution_lookahead_cannot_choose_an_over_budget_alternative():
    from dougie_sim.path_motion import RouteSmoother
    value=survival_planner()
    nodes,_=value.terrain.build_graph()
    root=value.terrain.nearest([0.,0.,0.])
    key=value.terrain.nearest([.8,0.,0.])
    parent=value.terrain.routes(root)[3]
    value.smoother=RouteSmoother(value.terrain,value.gp,float('inf'),prediction='mean')
    value.survival_return_dose=dict.fromkeys(nodes,.1)
    value.survival_root_rate=1.
    rates=dict.fromkeys(nodes,1.)
    assert value.refine_frontier_route(key,np.zeros(3),parent,parent,1,1.,0.,rates) is None
    assert key in value.survival_dose_rejections
