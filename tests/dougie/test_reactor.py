import mujoco
import numpy as np

from dougie_sim.radiation import RadiationField
from dougie_sim.reactor import radiation_description
from dougie_sim.simulation import Simulation


def test_reactor_detail_separates_collisions_from_decoration_and_preserves_staging_floor():
    sim = Simulation('reactor', controller='stand')
    for name in ('vessel_domed_head', 'pump_motor_0', 'pipe_rack_post_0_0', 'service_alcove_shield'):
        geom = sim.model.geom(name)
        assert geom.contype > 0 and geom.conaffinity > 0
    for name in ('panel_screen_0', 'panel_vent_0_0', 'walkway_edge_0.75', 'motor_fin_0_0'):
        geom = sim.model.geom(name)
        assert geom.contype == geom.conaffinity == 0 and geom.group == 2
    group = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    hit = np.empty(1, dtype=np.int32)
    for x in (-.4, 0., .4):
        distance = mujoco.mj_ray(sim.model, sim.data, np.array([x, 0., .5]),
            np.array([0., 0., -1.]), group, 1, sim.base, hit)
        assert abs(distance-.5) < 1e-8


def test_nominal_and_resolved_reactor_shielding_match():
    sim = Simulation('reactor', controller='stand')
    field = RadiationField.from_mujoco(sim.model, sim.data)
    nominal = radiation_description()['shields']
    assert len(nominal) == len(field.shields) == 4
    for a, b in zip(nominal, field.shields):
        for key in ('lower', 'upper', 'attenuation_per_m'):
            np.testing.assert_allclose(a[key], b[key], atol=1e-12)
