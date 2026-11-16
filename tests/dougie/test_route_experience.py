from types import SimpleNamespace

import numpy as np

from dougie_sim.route_experience import RouteExperience
from dougie_sim.terrain import SurfaceMap
from dougie_sim.route_forecast import forecast_tree
from dougie_sim.campaign_memory import save_memory,restore_memory


def feed(experience):
    for t,x,dose in ((1.,.45,0.),(1.5,.55,.001),(2.,.45,.002),(2.5,.55,.004)):
        experience.observe(SimpleNamespace(stamp_s=t,position=[x,0.,.44],dose_usv=dose,position_valid=True))


def test_recovered_directed_observations_inflate_time_and_dose_not_reverse_or_discount():
    exp=RouteExperience()
    feed(exp)
    assert not exp.edges and exp.pending
    exp.commit()
    seconds,dose=exp.bounds([.45,0.,0.],[.55,0.,0.],.1)
    assert seconds>=.49999 and dose>=.00099
    assert exp.bounds([.55,0.,0.],[.45,0.,0.],.1)==(0.,0.)
    terrain=SurfaceMap()
    terrain.nodes={0:np.array([.45,0.,0.]),1:np.array([.55,0.,0.])}
    terrain.graph={0:[1],1:[0]}
    rates={0:.1,1:.1}
    parent=terrain.routes(0,rates)[3]
    base=forecast_tree(terrain,parent,rates)
    terrain.route_experience=exp
    learned=forecast_tree(terrain,parent,rates)
    assert learned[0][1]>=base[0][1] and learned[1][1]>base[1][1]
    reverse=forecast_tree(terrain,parent,rates,reverse=True)
    assert reverse[1][1]==base[1][1]


def test_experience_commits_only_on_recovery_and_restores_without_duplicate_samples(tmp_path):
    from dougie_sim.planning_worker import create_explorer
    opts=dict(no_dose_budget=True,radiation_policy='minimize-dose',campaign_strategy='agenda',radiation_awareness='aware')
    explorer=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',opts)
    points=np.array([[x,y,0.] for x in np.arange(-1.5,1.5,.1) for y in np.arange(-1.5,1.5,.1)])
    explorer.observe_geometry(points,points)
    explorer.terrain.build_graph()
    feed(explorer.terrain.route_experience)
    save_memory(explorer,tmp_path/'campaign_memory.npz')
    restored=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',opts)
    restore_memory(restored,tmp_path,[0.,0.,0.])
    assert not restored.terrain.route_experience.edges
    save_memory(explorer,tmp_path/'campaign_memory.npz',[[0.,0.,0.],[.1,0.,0.]])
    restore_memory(restored,tmp_path,[0.,0.,0.])
    assert restored.terrain.route_experience.metrics()['observations']==3
    assert restored.terrain.route_experience.last is None
