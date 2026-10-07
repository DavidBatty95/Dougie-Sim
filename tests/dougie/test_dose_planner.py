import numpy as np

from dougie_sim.dose_planner import DoseExplorer
from dougie_sim.radiation import RadiationSample
from dougie_sim.terrain import SurfaceMap


def test_route_dose_integrates_rate_over_travel_time():
    terrain = SurfaceMap()
    a, b = (0, 0, 0), (1, 0, 0)
    terrain.nodes = {a: np.array([0., 0., 0.]), b: np.array([1., 0., 0.])}
    terrain.graph = {a: [b], b: [a]}
    _, dose, length, _ = terrain.routes(a, {a: 360., b: 360.}, speed=.1)
    assert length[b] == 1.
    assert dose[b] == 1.  # 360 uSv/hour * 10 seconds / 3600.


def test_low_remaining_budget_returns_instead_of_dispatching_exploration():
    mission = DoseExplorer(budget=.1)
    support = np.array([[x, y, 0.] for x in np.arange(-2., 2., .1) for y in np.arange(-2., 2., .1)])
    mission.observe_geometry(support, support, [0., 0., .44])
    mission.observe_radiation(RadiationSample(1., (0., 0., .44), 100, .5, 10., .1, .085, .09))
    mission.replan([0., 0., 0.], 1.)
    assert mission.returning
    assert mission.state == 'returned'
    assert not mission.path


def test_stale_radiation_input_cannot_dispatch_a_path():
    mission = DoseExplorer()
    support = np.array([[x, y, 0.] for x in np.arange(-1., 1., .1) for y in np.arange(-1., 1., .1)])
    mission.observe_geometry(support, support)
    mission.observe_radiation(RadiationSample(0., (0., 0., .44), 0, .5, 0., .1, 0., .0001))
    mission.replan([0., 0., 0.], 5.)
    assert not mission.path
    assert mission.state == 'waiting_for_fresh_sensors_or_support'
