import numpy as np

from dougie_sim.terrain import SurfaceMap
from dougie_sim.campaign_memory import save_memory,restore_memory


def graph():
    t=SurfaceMap(resolution=.25)
    t.nodes={0:np.array([0.,0.,0.]),1:np.array([.25,0.,0.]),2:np.array([.25,.25,0.]),3:np.array([.5,0.,0.])}
    t.graph={0:[1,2],1:[0,3],2:[0,3],3:[1,2]}
    return t


def test_shared_path_changes_travel_preference_but_not_radiation_dose():
    t=graph()
    t.shared_routes=[np.array([[0.,0.,0.],[.25,.25,0.],[.5,0.,0.]])]
    t.refresh_shared_route_edges()
    # A longer observed route must not beat a short route when its radiation
    # penalty is substantially higher, despite the modest familiarity discount.
    rates={0:0.,1:0.,2:20.,3:0.}
    _,dose,_,parent=t.routes(0,rates,rate_scale=1.)
    assert t.path(parent,3)==[0,1,3] and dose[3]==0.
    # Previously used edges still obey current physical connectivity.
    t.graph[0].remove(2)
    t.refresh_shared_route_edges()
    assert frozenset((0,2)) not in t.shared_route_edges


def test_route_memory_is_accumulated_only_when_recovered(tmp_path):
    from dougie_sim.planning_worker import create_explorer
    from dougie_sim.radiation import RadiationSample
    options=dict(no_dose_budget=True,radiation_policy='minimize-dose',radiation_awareness='blind')
    first=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    points=np.array([[x,y,0.] for x in np.arange(-1.5,1.5,.1) for y in np.arange(-1.5,1.5,.1)])
    first.observe_geometry(points,points)
    first.terrain.build_graph()
    first.observe_radiation(RadiationSample(1.,(0.,0.,.44),0,.5,0.,0.,0.,0.))
    keys=list(first.terrain.nodes)
    edge=next((a,b) for a in keys for b in first.terrain.graph[a])
    path=np.array([first.terrain.nodes[edge[0]],first.terrain.nodes[edge[1]]])
    save_memory(first,tmp_path/'campaign_memory.npz',path)
    second=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    restore_memory(second,tmp_path,[0.,0.,0.])
    assert len(second.terrain.shared_routes)==1
    np.testing.assert_allclose(second.terrain.shared_routes[0],path)
    assert second.terrain.shared_route_edges
    # A subsequent lost sortie saves existing routes but contributes no new one.
    save_memory(second,tmp_path/'campaign_memory.npz')
    third=create_explorer((2.,100.,32,1e12,'poisson'),'clustered','conservative',options)
    restore_memory(third,tmp_path,[0.,0.,0.])
    assert len(third.terrain.shared_routes)==1
