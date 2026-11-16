import numpy as np
import pytest

from dougie_sim.path_motion import MotionRoute
from dougie_sim.return_alternatives import best_return_route
from dougie_sim.terrain import SurfaceMap


def scene():
    terrain=SurfaceMap()
    terrain.nodes={0:np.array([0.,0.,0.]),1:np.array([1.,0.,0.]),2:np.array([0.,1.,0.]),
        3:np.array([1.,1.,0.]),4:np.array([2.,0.,0.])}
    terrain.graph={0:[1,2],1:[0,4],2:[0,3],3:[2,4],4:[1,3]}
    upper={0:1.,1:100.,2:1.,3:1.,4:1.}
    class GP:
        def predict(self,points):
            rates=np.array([100. if abs(p[0]-1.)<.1 and abs(p[1])<.1 else 1. for p in points])
            return rates,np.full(len(rates),.2),rates
    def smoother(points,returning=False):
        assert returning
        return MotionRoute(points,[],dict())
    return terrain,upper,GP(),smoother


def test_lowest_upper_dose_return_uses_cool_admitted_route_not_short_hot_one():
    terrain,rates,gp,smoother=scene()
    baseline=terrain.routes(0)[3]
    result=best_return_route(terrain,0,4,terrain.nodes[0],baseline,rates,gp,smoother)
    assert result['baseline']['family']=='current'
    assert result['best']['family']=='upper_dose'
    assert result['best']['upper_dose_usv']<result['baseline']['upper_dose_usv']
    assert any(np.allclose(p,[1.,1.,0.]) for p in result['best']['raw'])
    np.testing.assert_allclose(result['best']['motion'].points,[terrain.nodes[0],*result['best']['raw']])


def test_confidently_excluded_connector_is_never_used_for_alternative():
    terrain,rates,gp,smoother=scene()
    baseline=terrain.routes(0)[3]
    result=best_return_route(terrain,0,4,terrain.nodes[0],baseline,rates,gp,smoother,excluded={3})
    assert result['best']['family']=='current'
    assert all(not any(np.allclose(p,[1.,1.,0.]) for p in item['raw']) for item in result['variants'])


def test_no_admissible_alternative_keeps_existing_route_and_local_measurement():
    terrain,rates,gp,smoother=scene()
    baseline=terrain.routes(0)[3]
    result=best_return_route(terrain,0,4,terrain.nodes[0],baseline,rates,gp,smoother,
        excluded={2,3},local_upper=200.)
    assert result['best']['family']=='current'
    assert result['best']['peak_rate_usv_h']>=200.


@pytest.mark.parametrize('finish_survey',[False,True])
def test_selected_return_executes_the_exact_forecast_certificate_and_recovery_can_invalidate_it(monkeypatch,finish_survey):
    from types import SimpleNamespace
    from dougie_sim.campaign_agenda import CampaignAgenda
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    from dougie_sim.planning_worker import matching_execution_route
    from dougie_sim.radiation import RadiationSample
    value=ClusteredDoseExplorer(budget=2.,mission_duration=1e12)
    value.rate_limited=False
    value.survival_mode=value.survey_until_complete=value.smooth_execution=True
    value.campaign_agenda=CampaignAgenda()
    value.radiation_awareness='aware'
    points=np.array([[x,y,0.] for x in np.arange(-2.,2.,.1) for y in np.arange(-2.,2.,.1)])
    value.observe_geometry(points,points)
    value.gp=SimpleNamespace(predict=lambda p:(np.ones(len(p)),np.full(len(p),.1),np.full(len(p),1.2)))
    value.sample=RadiationSample(5.,(.7,0.,.44),5,.5,1.,.1,0.,0.)
    value.home=np.zeros(3)
    value.returning=not finish_survey
    if finish_survey:
        value.frontier_confirmation_count=2
        value.last_frontier_confirmation=1.
        monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',lambda terrain:
            SimpleNamespace(values=lambda keys:np.zeros(len(keys)),value=lambda key:0,cache={}))
    value.replan(np.array([.7,0.,0.]),5.)
    assert value.returning and value.path
    certificate=matching_execution_route(value,value.path)
    assert certificate is value.execution_route and certificate is not None
    np.testing.assert_allclose(certificate.points[0],[.7,0.,0.])
    assert value.route_forecast['return_forecast_geometry']=='certified_upper_comparison'
    assert matching_execution_route(value,[]) is None
    assert matching_execution_route(value,[value.path[-1]+[.5,0.,0.]]) is None
