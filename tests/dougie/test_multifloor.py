import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import torch

from dougie_sim.assets import ASSETS
from dougie_sim.layered_terrain import LayeredSurfaceMap, VoxelEvidence
from dougie_sim.mission_evaluation import MissionWitness
from dougie_sim.policy import DEFAULT_POLICY, PolicyContract
from dougie_sim.radiation import RadiationField
from dougie_sim.sensing import RangePacket
from dougie_sim.simulation import Simulation, scene_xml
from dougie_sim.terrain import SurfaceMap


def test_policy_adapter_preserves_exported_actions():
    contract = PolicyContract()
    adapter = contract.load()
    baseline = torch.jit.load(str(ASSETS/'policy/go2.pt')).eval()
    generator = torch.Generator().manual_seed(43)
    with torch.inference_mode():
        observations = torch.randn(32, 45, generator=generator)
        torch.testing.assert_close(adapter(observations), baseline(observations), rtol=0, atol=0)
    sim = Simulation()
    for _ in range(220):
        sim.step((.2, 0, 0))
    assert sim.metrics()['policy']['sha256'] == contract.config['sha256']


def test_invalid_policy_contracts_fail_before_execution(tmp_path):
    config = json.loads(DEFAULT_POLICY.read_text())
    config['artifact'] = str(ASSETS/'policy/go2.pt')
    path = tmp_path/'contract.json'
    for name, value in [('schema_version', 99), ('joint_order', list(reversed(config['joint_order']))),
                        ('policy_period_s', .04), ('torque_limit_nm', 30.), ('kp', float('nan'))]:
        path.write_text(json.dumps({**config, name: value}))
        with pytest.raises(ValueError):
            PolicyContract(path)
    path.write_text(json.dumps({**config, 'sha256': '0'*64}))
    with pytest.raises(ValueError, match='checksum'):
        PolicyContract(path).load()


def test_range_packet_is_detached_and_ceiling_is_not_support():
    root = ET.fromstring(scene_xml('flat'))
    ET.SubElement(root.find('worldbody'), 'geom', type='box', size='5 5 .1', pos='0 0 1.1')
    sim = Simulation(controller='stand', model_xml=ET.tostring(root, encoding='unicode'))
    packet = sim.lidar_packet()
    assert np.any(packet.points[:, 2] > .9)
    assert len(packet.supports) > 100
    assert np.max(packet.supports[:, 2]) < .01
    with pytest.raises(ValueError):
        packet.origin[0] = 1.
    origin = packet.origin.copy()
    sim.step()
    np.testing.assert_array_equal(origin, packet.origin)
    assert packet.sequence == 0 and packet.stamp_s == 0


def test_no_return_carves_free_space_but_hit_does_not_clear_behind_wall():
    occupancy = VoxelEvidence(resolution=.2)
    packet = RangePacket(0, 0, np.array([.01, .01, .01]), np.array([[[1., 0., 0.]]]),
                         np.array([[1.]]), 3., np.array([[False]]))
    occupancy.update(packet)
    assert occupancy.evidence[(5, 0, 0)] > 0
    assert (7, 0, 0) not in occupancy.evidence
    empty = RangePacket(1, 1, packet.origin, packet.directions, np.array([[-1.]]), 3., packet.support_mask)
    occupancy.update(empty)
    assert occupancy.evidence[(10, 0, 0)] < 0


def cloud(height):
    return np.array([[x, y, height(x)] for x in np.arange(-1.5, 1.51, .05)
                     for y in np.arange(-1., 1.01, .05)])


def test_piecewise_ramp_join_keeps_full_footprint_without_extrapolating_one_plane():
    terrain = LayeredSurfaceMap(radius=.59)
    slope = math.tan(math.radians(10.))
    supports = cloud(lambda x: slope*max(0., x))
    terrain.update(supports, supports)
    terrain.build_graph()
    start, end = terrain.nearest([-.7, 0., 0.]), terrain.nearest([.7, 0., slope*.7])
    assert start is not None and end is not None
    assert end in terrain.routes(start)[3]


def test_support_footprint_does_not_apply_body_clearance_twice():
    terrain = LayeredSurfaceMap(radius=.45)
    supports = cloud(lambda x: 0.)
    wall = np.array([[0., y, .4] for y in np.arange(-1., 1.01, .025)])
    terrain.update(np.vstack((supports, wall)), supports)
    terrain.build_graph()
    assert any(.49 < p[0] < .51 for p in terrain.nodes.values())
    assert all(abs(p[0]) >= .45 for p in terrain.nodes.values())


def test_gentler_fixture_preserves_height_and_local_source_challenge():
    from dougie_sim.multilevel import source_description
    original = Simulation('mezzanine')
    gentle = Simulation('mezzanine', mezzanine_angle=8.)
    a, b = original.model.geom('mezzanine_slab').id, gentle.model.geom('mezzanine_slab').id
    height_a = original.data.geom_xpos[a, 2]+original.model.geom_size[a, 2]
    height_b = gentle.data.geom_xpos[b, 2]+gentle.model.geom_size[b, 2]
    assert height_b == pytest.approx(height_a)
    ramp = gentle.model.geom('access_ramp_0').id
    R = gentle.data.geom_xmat[ramp].reshape(3, 3)
    assert math.atan2(R[2, 0], R[0, 0]) == pytest.approx(math.radians(8.))
    assert gentle.model.geom_size[ramp, 0] > original.model.geom_size[original.model.geom('access_ramp_0').id, 0]
    for sim, angle in ((original, 10.), (gentle, 8.)):
        slab = sim.model.geom('mezzanine_slab').id
        assert source_description(angle)['sources'][0]['position'][0]-sim.data.geom_xpos[slab, 0] == pytest.approx(.5)
    assert LayeredSurfaceMap().max_slope == pytest.approx(math.tan(math.radians(10.)))


def test_incremental_footprints_match_fresh_graph_after_obstacle():
    supports = cloud(lambda x: 0.)
    terrain = LayeredSurfaceMap()
    terrain.update(supports, supports)
    terrain.build_graph()
    original = set(terrain.nodes)
    terrain.build_graph()
    assert terrain.footprints_checked == 0
    obstacle = np.array([[0., 0., .3]])
    terrain.update(obstacle, np.empty((0, 3)))
    terrain.build_graph()
    fresh = LayeredSurfaceMap()
    fresh.update(supports, supports)
    fresh.update(obstacle, np.empty((0, 3)))
    fresh.build_graph()
    assert set(terrain.nodes) == set(fresh.nodes) < original
    assert terrain.graph == fresh.graph
    assert terrain.edge_kind == fresh.edge_kind


def test_layered_map_accepts_slope_but_rejects_equivalent_abrupt_step():
    for height, reachable in ((lambda x: .15*x, True), (lambda x: 0. if x < 0 else .06, False)):
        terrain = LayeredSurfaceMap()
        supports = cloud(height)
        terrain.update(supports, supports)
        terrain.build_graph()
        a, b = terrain.nearest([-.8, 0, height(-.8)]), terrain.nearest([.8, 0, height(.8)])
        assert a is not None and b is not None
        assert (b in terrain.routes(a)[0]) == reachable


def test_stacked_floors_have_no_vertical_shortcuts():
    terrain = LayeredSurfaceMap()
    supports = np.vstack((cloud(lambda x: 0.), cloud(lambda x: 2.5)))
    terrain.update(supports, supports)
    terrain.build_graph()
    lower, upper = terrain.nearest([0, 0, 0]), terrain.nearest([0, 0, 2.5])
    assert lower is not None and upper is not None and lower != upper
    assert upper not in terrain.routes(lower)[0]


def test_reverse_routes_use_original_edge_direction_and_time():
    terrain = SurfaceMap()
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([1., 0., 0.]), 2: np.array([2., 0., 0.])}
    terrain.graph = {0: [1], 1: [0, 2], 2: []}
    terrain.edge_time = lambda a, b, speed: {(0, 1): 10., (1, 0): 30., (1, 2): 5.}[a, b]
    rates = dict.fromkeys(terrain.nodes, 3600.)
    _, dose, _, _ = terrain.routes(0, rates, reverse=True)
    assert dose[1] == 30. and 2 not in dose
    assert terrain.routes(0, rates)[1][1] == 10.


def test_mezzanine_has_real_slab_shielding_and_connected_ramp_surface():
    sim = Simulation('mezzanine', controller='stand')
    field = RadiationField.from_mujoco(sim.model, sim.data)
    assert len(field.sources) == 2 and len(field.shields) == 2
    slab = next(s for s in field.shields if s['upper'][0] == 9.)
    assert slab['upper'][2] == pytest.approx(4*math.tan(math.radians(10)))
    unshielded = RadiationField(dict(background_usv_h=field.background, sources=field.sources))
    assert field.rate([7.5, 0., .4]) < unshielded.rate([7.5, 0., .4])
    terrain = LayeredSurfaceMap()
    packet = sim.lidar_packet()
    terrain.observe_packet(packet)
    terrain.build_graph()
    assert terrain.nearest([0, 0, 0]) is not None
    assert terrain.metrics()['ramp_edges'] > 0
    reachable = terrain.routes(terrain.nearest([0, 0, 0]))[0]
    assert max(terrain.nodes[k][2] for k in reachable) > .1


def test_3d_ramp_and_slab_returns_match_scalar_raycast():
    sim = Simulation('mezzanine', controller='stand')
    groups = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    geom = np.empty(1, dtype=np.int32)
    for position in ([0., 0., .32], [7., 0., 1.03]):
        sim.data.qpos[:3] = position  # Sensor fixture only, not execution.
        mujoco.mj_forward(sim.model, sim.data)
        packet = sim.lidar_packet()
        for direction, actual in zip(packet.directions.reshape(-1, 3)[::13], packet.distances.ravel()[::13]):
            expected = mujoco.mj_ray(sim.model, sim.data, packet.origin, direction, groups, 1, sim.base, geom)
            expected = expected if expected <= packet.max_range_m else -1.
            assert actual == pytest.approx(expected, abs=1e-10)


def test_rotated_radiation_shield_is_rejected_instead_of_miscomputed():
    root = ET.fromstring(scene_xml('mezzanine'))
    root.find("worldbody/geom[@name='lower_shield']").set('quat', '.9238795 0 0 .3826834')
    sim = Simulation(controller='stand', model_xml=ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError, match='axis-aligned'):
        RadiationField.from_mujoco(sim.model, sim.data)


def test_vertical_witness_requires_landing_dwell_then_lower_return(tmp_path):
    sim = Simulation('mezzanine', controller='stand')
    witness = MissionWitness(sim, 'mezzanine')
    witness.tick(sim)
    assert not witness.metrics()['vertical_round_trip_witness']
    height = 4*math.tan(math.radians(10))
    # Initialize test poses solely to exercise the evaluator, never the controller.
    sim.data.qpos[:3] = [6., 0., height+.32]
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(10):
        witness.tick(sim)
    assert witness.upper_stamp is None
    for _ in range(40):
        witness.tick(sim)
    assert witness.upper_stamp is not None and witness.lower_stamp is None
    sim.data.qpos[:3] = [0., 0., .32]
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(50):
        witness.tick(sim)
    assert witness.metrics()['vertical_round_trip_witness']
    witness.save(tmp_path/'motion.npz')
    with np.load(tmp_path/'motion.npz') as data:
        assert data['rows'].shape[1] == len(data['columns'])


def test_occupancy_capacity_is_bounded_and_reported():
    occupancy = VoxelEvidence(capacity=2)
    packet = RangePacket(0, 0, np.zeros(3), np.array([[[1., 0., 0.]]]),
                         np.array([[-1.]]), 3., np.array([[False]]))
    occupancy.update(packet)
    assert len(occupancy.evidence) == 2 and occupancy.saturated
