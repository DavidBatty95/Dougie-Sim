import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from dougie_sim.facility import FLOOR_HEIGHT
from dougie_sim.radiation import RadiationField
from dougie_sim.simulation import Simulation, scene_xml


def test_facility_has_overlapping_full_height_rooms_and_explicit_shielding():
    sim = Simulation('facility', controller='stand')
    slab = sim.model.geom('upper_floor_main').id
    assert abs(sim.data.geom_xpos[slab, 2]+sim.model.geom_size[slab, 2]-FLOOR_HEIGHT) < 1e-10
    assert sim.data.geom_xpos[slab, 2]-sim.model.geom_size[slab, 2] > 2.
    field = RadiationField.from_mujoco(sim.model, sim.data)
    assert len(field.sources) == 3 and len(field.shields) >= 10
    point = [6.3, 3.7, .4]
    unshielded = RadiationField(dict(background_usv_h=field.background, sources=field.sources))
    assert field.rate(point) < unshielded.rate(point)
    packet = sim.lidar_packet()
    assert len(packet.supports) > 100
    assert not np.any((packet.supports[:, 2] > 2.) & (packet.supports[:, 2] < 2.3))


def test_facility_ramps_join_landings_without_vertical_lips():
    sim = Simulation('facility', controller='stand')
    group = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    geom = np.empty(1, dtype=np.int32)
    # Downward evaluator probes are fixtures only, never planner observations.
    for x, y, expected in ((1.01, 6.4, .0015), (8.99, 6.4, 1.1985),
                            (9.01, 6.4, 1.2), (8.99, 9.6, 1.2015),
                            (1.01, 9.6, 2.3985), (.99, 9.6, 2.4)):
        start = np.array([x, y, 4.])
        distance = mujoco.mj_ray(sim.model, sim.data, start, np.array([0., 0., -1.]), group, 1, sim.base, geom)
        assert abs(4.-distance-expected) < 1e-6


def test_layout_seed_changes_rooms_but_not_connector_geometry():
    a, b = (ET.fromstring(scene_xml('facility', layout_seed=seed)) for seed in (0, 1))
    assert a.find("worldbody/geom[@name='room_partition_0_0']").get('pos') != b.find("worldbody/geom[@name='room_partition_0_0']").get('pos')
    assert a.find("worldbody/geom[@name='service_ramp_lower']").attrib == b.find("worldbody/geom[@name='service_ramp_lower']").attrib
