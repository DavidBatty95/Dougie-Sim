from types import SimpleNamespace

import numpy as np

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.radiation import RadiationSample


def explorer(monkeypatch,predict,gain=None):
    value = ClusteredDoseExplorer(mission_duration=1e12)
    value.budget_enabled = value.rate_limited = False
    value.exposure_policy = 'pathwise'
    value.survey_until_complete = value.smooth_execution = True
    support = np.array([[x,y,0.] for x in np.arange(-2.5,2.5,.1) for y in np.arange(-2.5,2.5,.1)])
    value.observe_geometry(support,support)
    value.gp = SimpleNamespace(predict=predict)
    value.sample = RadiationSample(1.,(0.,0.,.44),5,.5,1.,.1,.001,.002)
    value.observed_rates.append(1.)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',
        lambda terrain:SimpleNamespace(value=lambda key:gain(terrain.nodes[key]) if gain else 1,cache={}))
    return value


def test_no_forecast_benefit_keeps_geometric_execution(monkeypatch):
    value = explorer(monkeypatch,lambda p:(np.ones(len(p)),np.full(len(p),.1),np.full(len(p),1.2)))
    value.replan(np.zeros(3),1.)
    assert value.selected_utility['route_family'] == 'geometric'
    assert value.selected_utility['expected_dose_saving_fraction'] == 0.
    assert value.execution_route is not None


def test_radiation_does_not_send_goal_scheduler_to_low_progress_regions(monkeypatch):
    zero = lambda p:(np.zeros(len(p)),np.zeros(len(p)),np.zeros(len(p)))
    def radiation(points):
        mean = np.array([50. if p[0] > 0 else 1. for p in points])
        return mean,np.full(len(points),.1),mean*1.2
    baseline = explorer(monkeypatch,zero)
    baseline.replan(np.zeros(3),1.)
    aware = explorer(monkeypatch,radiation)
    aware.replan(np.zeros(3),1.)
    np.testing.assert_array_equal(aware.target,baseline.target)


def test_known_hot_route_has_a_model_dominating_certified_detour(monkeypatch):
    def radiation(points):
        mean = np.array([100. if .35 < p[0] < .85 and abs(p[1]) < .25 else 1. for p in points])
        return mean,np.full(len(points),.1),mean*1.2
    gain = lambda p:int(abs(p[0]-1.375)<.01 and abs(p[1]-.125)<.01)
    value = explorer(monkeypatch,radiation,gain)
    value.utility.exposure = {value.utility.cell([x,y,.44]):2. for x in np.arange(-2.,2.,.25) for y in np.arange(-2.,2.,.25)}
    value.replan(np.zeros(3),1.)
    detail = value.selected_utility
    assert detail['route_family'] in ('dose_weighted','dose_only')
    assert detail['selected_expected_dose_usv'] <= .95*detail['geometric_expected_dose_usv']
    assert detail['selected_upper_dose_usv'] <= 1.05*detail['geometric_upper_dose_usv']
    assert value.execution_route is not None
    assert any(abs(p[1]) >= .25 for p in value.execution_route.points if .35 < p[0] < .85)


def test_unmeasured_cold_prior_cannot_justify_a_detour(monkeypatch):
    def radiation(points):
        mean = np.array([100. if .35 < p[0] < .85 and abs(p[1]) < .25 else 1. for p in points])
        return mean,np.full(len(points),1.5),mean*1.2
    gain = lambda p:int(abs(p[0]-1.375)<.01 and abs(p[1]-.125)<.01)
    value = explorer(monkeypatch,radiation,gain)
    value.replan(np.zeros(3),1.)
    assert value.selected_utility['route_family'] == 'geometric'
    assert value.selected_utility['expected_dose_saving_fraction'] == 0.


def test_recovery_stop_or_replacement_invalidates_the_execution_certificate():
    from dougie_sim.planning_worker import matching_execution_route
    route = SimpleNamespace(points=[np.zeros(3),np.ones(3)])
    value = SimpleNamespace(execution_route=route,execution_route_raw=[np.ones(3)])
    assert matching_execution_route(value,[np.ones(3)]) is route
    assert matching_execution_route(value,[]) is None
    assert matching_execution_route(value,[-np.ones(3)]) is None
