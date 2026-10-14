import hashlib
import json
from types import SimpleNamespace

import numpy as np
import torch

from dougie_sim.layered_terrain import LayeredSurfaceMap
from dougie_sim.perception import (ELEVATION_INDICES, OBSERVATIONS, RAY_COUNT, SCHEMA, SENSOR_CONTRACT,
                                   TrainingLidarObservation, range_features)
from dougie_sim.policy import DEFAULT_POLICY
from dougie_sim.simulation import Simulation
from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.radiation import RadiationSample
from dougie_sim.training import TRAINING_FEET, BodyContactBudget, incline_velocity_tracking, stopped_motion
from dougie_sim.perception import CoherentActorObservation


def test_sparse_policy_rays_are_a_subset_of_the_existing_sensor():
    sim = Simulation('mezzanine', controller='stand')
    full, sparse = sim.lidar_packet(), sim.lidar_packet(sparse=True)
    np.testing.assert_allclose(sparse.directions, full.directions[list(ELEVATION_INDICES), ::10], atol=1e-12)
    np.testing.assert_allclose(sparse.distances, full.distances[list(ELEVATION_INDICES), ::10], atol=1e-12)
    assert sparse.distances.size == RAY_COUNT


def test_reward_foot_order_matches_actual_model_contact_order():
    sim = Simulation(controller='stand')
    physical_order = tuple(sim.model.geom(i).name for i in range(sim.model.ngeom)
                           if sim.model.geom(i).name in ('FR', 'FL', 'RR', 'RL'))
    assert TRAINING_FEET == physical_order
    assert TRAINING_FEET != ('FR', 'FL', 'RR', 'RL')  # The action order is intentionally different.


def test_coherent_training_observation_preserves_the_deployment_channel_order():
    data = SimpleNamespace(root_link_ang_vel_b=torch.tensor([[1., 2., 3.]]),
        projected_gravity_b=torch.tensor([[0., 0., -1.]]),
        joint_pos=torch.arange(12).float()[None], default_joint_pos=torch.zeros(1, 12),
        joint_vel=torch.ones(1, 12)*2.)
    env = SimpleNamespace(num_envs=1, device='cpu', scene={'robot': SimpleNamespace(data=data)},
        action_manager=SimpleNamespace(action=torch.ones(1, 12)*.5),
        command_manager=SimpleNamespace(get_command=lambda _: torch.tensor([[.2, 0., .1]])))
    asset = SimpleNamespace(name='robot', joint_ids=[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
    observation = CoherentActorObservation(None, env)(env, asset, perceptive=False)
    assert observation.shape == (1, 45)
    torch.testing.assert_close(observation[0, :3], torch.tensor([.25, .5, .75]))
    torch.testing.assert_close(observation[0, 9:21], data.joint_pos[0, asset.joint_ids])
    torch.testing.assert_close(observation[0, 21:33], torch.full((12,), .1))


def test_incline_rewards_allow_vertical_motion_and_penalize_stopped_drift():
    command = torch.tensor([[.2, 0., 0.]])
    data = SimpleNamespace(root_link_lin_vel_b=torch.tensor([[.2, 0., .1]]),
                           root_link_ang_vel_b=torch.zeros(1, 3))
    env = SimpleNamespace(scene={'robot': SimpleNamespace(data=data)},
                          command_manager=SimpleNamespace(get_command=lambda _: command))
    torch.testing.assert_close(incline_velocity_tracking(env), torch.ones(1))
    assert stopped_motion(env).item() == 0.
    command.zero_()
    assert stopped_motion(env).item() > 0.


def test_training_contact_budget_detects_accumulated_light_impacts():
    force = torch.zeros(2, 1, 4, 3)
    force[0, 0, :, 0] = 3.
    data = SimpleNamespace(force_history=force)
    env = SimpleNamespace(num_envs=2, device='cpu', step_dt=.02,
                          scene={'nonfoot_contact': SimpleNamespace(data=data)})
    gate = BodyContactBudget(None, env)
    assert gate(env).tolist() == [False, False]  # 0.06 Ns
    assert gate(env).tolist() == [True, False]   # 0.12 Ns, despite peak only 3 N
    gate.reset(torch.tensor([0]))
    assert gate(env).tolist() == [False, False]


def test_training_and_deployment_range_encoding_and_hold_age_match():
    distances = np.linspace(.1, 6., RAY_COUNT, dtype=np.float32)
    distances[:3] = [-1., np.nan, .01]
    data = SimpleNamespace(distances=torch.from_numpy(distances[None].copy()))
    env = SimpleNamespace(num_envs=1, device='cpu', step_dt=.02, episode_length_buf=torch.zeros(1, dtype=torch.long),
                          scene={'policy_lidar': SimpleNamespace(data=data)})
    observation = TrainingLidarObservation(None, env)
    actual = observation(env, corrupt=False).numpy()[0]
    np.testing.assert_allclose(actual, range_features(distances, 0.), atol=1e-7)
    data.distances[:] = -1.
    env.episode_length_buf[:] = 3
    np.testing.assert_allclose(observation(env, corrupt=False).numpy()[0], range_features(distances, .06), atol=1e-7)
    env.episode_length_buf[:] = 5
    np.testing.assert_allclose(observation(env, corrupt=False).numpy()[0], range_features(np.full(RAY_COUNT, -1.), 0.), atol=1e-7)
    observation.reset()
    env.episode_length_buf[:] = 0
    assert observation(env, corrupt=False)[0, -1] == 0.


def test_version_two_policy_executes_with_the_declared_sensor_contract(tmp_path):
    actor = torch.nn.Linear(OBSERVATIONS, 12)
    with torch.no_grad():
        actor.weight.zero_()
        actor.bias.zero_()
    path = tmp_path/'policy.pt'
    torch.jit.script(actor).save(str(path))
    config = json.loads(DEFAULT_POLICY.read_text())
    config.update(schema_version=2, observation_schema=SCHEMA, range_sensor=SENSOR_CONTRACT,
                  artifact=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    manifest = tmp_path/'policy.json'
    manifest.write_text(json.dumps(config))
    sim = Simulation(policy_config=manifest)
    observation = sim.observation([0, 0, 0])
    assert observation.shape == (OBSERVATIONS,) and np.isfinite(observation).all()
    for _ in range(120):
        sim.step()
    assert sim.metrics()['policy']['schema'] == SCHEMA


def test_floor_quantization_does_not_double_count_floating_point_zero():
    terrain = LayeredSurfaceMap()
    points = np.array([[.01, .01, -1e-15], [.01, .01, 1e-15]])
    terrain.update(points, points)
    assert len(terrain.support_returns) == 1


def test_cached_map_is_invalidated_by_a_new_local_obstacle():
    floor = np.array([[x, y, 0.] for x in np.arange(-3., 3., .1) for y in np.arange(-1., 1., .1)])
    obstacle = np.array([[1., y, .3] for y in np.arange(-1., 1., .1)])
    incremental, rebuilt = LayeredSurfaceMap(), LayeredSurfaceMap()
    incremental.update(floor, floor)
    incremental.build_graph()
    incremental.update(obstacle, np.empty((0, 3)))
    incremental.build_graph()
    rebuilt.update(floor, floor)
    rebuilt.update(obstacle, np.empty((0, 3)))
    rebuilt.build_graph()
    assert incremental.graph == rebuilt.graph
    assert incremental.reused_columns > 0


def test_visible_floor_below_ramp_does_not_destroy_the_upper_route():
    floor = np.array([[x, y, 0.] for x in np.arange(-1., 3.51, .05) for y in np.arange(-1., 1.01, .05)])
    ramp = np.array([[x, y, .15*(x-1)] for x in np.arange(1., 3.51, .05) for y in np.arange(-1., 1.01, .05)])
    terrain = LayeredSurfaceMap()
    # First see the ramp, later see the real but unusable floor beneath it.
    terrain.update(ramp, ramp)
    terrain.update(floor, floor)
    terrain.build_graph()
    root = terrain.nearest([0, 0, 0])
    reachable = terrain.routes(root)[0]
    assert max(terrain.nodes[k][2] for k in reachable) > .25


def test_explorer_can_choose_a_short_supported_inspection_viewpoint():
    planner = ClusteredDoseExplorer(budget=5.)
    terrain = LayeredSurfaceMap()
    keys = [(i, 0, 0) for i in range(3)]
    terrain.nodes = {key: np.array([.1+i*.2, .1, 0.]) for i, key in enumerate(keys)}
    terrain.graph = {keys[0]: [keys[1]], keys[1]: [keys[0], keys[2]], keys[2]: [keys[1]]}
    terrain.surfaces = dict.fromkeys(keys, 0.)
    terrain.build_graph = lambda: (terrain.nodes, terrain.graph)
    planner.terrain = terrain
    planner.observe_radiation(RadiationSample(1., (.1, .1, .44), 5, .5, .72, .1, .001, .002))
    planner.replan([.1, .1, 0.], 1.)
    assert planner.state == 'exploring'
    assert planner.last_reason == 'near_frontier_inspection'
    np.testing.assert_allclose(planner.target, [.5, .1, 0.])


def test_platform_edges_require_the_full_body_footprint():
    points = np.array([[x, y, 1.2] for x in np.arange(0., 3., .05) for y in np.arange(0., 3., .05)])
    terrain = LayeredSurfaceMap()
    terrain.update(points, points)
    terrain.build_graph()
    assert terrain.nearest([1.5, 1.5, 1.2]) is not None
    assert all(.39 < p[0] < 2.61 and .39 < p[1] < 2.61 for p in terrain.nodes.values())


def test_height_acquisition_uses_observed_slopes_and_sampled_height_only():
    terrain = LayeredSurfaceMap()
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([1., 0., .04]), 2: np.array([2., 0., .4])}
    terrain.planes = {0: np.array([0., 0., 0.]), 1: np.array([.15, 0., .04]), 2: np.array([0., 0., .4])}
    assert terrain.acquisition_bonus(0, {0}) == 0.
    assert terrain.acquisition_bonus(1, {0}) > 0.
    assert terrain.acquisition_bonus(2, {0}) > terrain.acquisition_bonus(2, {0, 2})
