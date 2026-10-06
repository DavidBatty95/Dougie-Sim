"""CPU checks for the GPU training contract (run in the MJLab uv environment)."""
from types import SimpleNamespace
import importlib

import numpy as np
import pytest

@pytest.fixture
def training():
    # Runtime skips: ROS launch-testing imports files during directory
    # discovery, where a module-level skip can suppress unrelated tests.
    pytest.importorskip('torch')
    pytest.importorskip('mujoco')
    pytest.importorskip('mjlab')
    return importlib.import_module('g1_payload_training')


def test_import_matches_source_and_arm_initialization_remains_trainable(training):
    torch = training.torch
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.mlp = torch.nn.Sequential(
                torch.nn.Linear(480, 512), torch.nn.ELU(),
                torch.nn.Linear(512, 256), torch.nn.ELU(),
                torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 29),
            )
    torch.set_num_threads(2)
    actor = Actor()
    assert training.load_onnx_actor(actor, training.SOURCE) < 2e-5
    observations = torch.randn(4, 480)
    masked = observations.clone()
    columns = [start + history*29 + joint for start in (45, 190, 335)
               for history in range(5) for joint in training.ARM_POLICY_IDS]
    masked[:, columns] = 0.
    with torch.no_grad():
        expected = actor.mlp(masked)
    training.initialize_arm_inputs(actor)
    actual = actor.mlp(observations)
    torch.testing.assert_close(actual, expected)
    actual.square().sum().backward()
    assert actor.mlp[0].weight.grad[:, columns].abs().sum() > 0


def test_payload_has_real_mass_and_cube_inertia_at_the_wrist(training):
    model = training.payload_spec().compile()
    payload = model.body('carried_payload')
    wrist = model.body('right_wrist_yaw_link')
    assert model.body_parentid[payload.id] == wrist.id
    assert model.body_jntnum[payload.id] == 0
    np.testing.assert_allclose(payload.pos, (.19, .01, 0.))
    np.testing.assert_allclose(payload.mass, .15)
    np.testing.assert_allclose(payload.inertia, np.full(3, .15*.11**2/6))


def test_virtual_cup_axis_matches_mujoco_nominal_carry_pose(training):
    model = training.payload_spec().compile()
    data = training.mujoco.MjData(model)
    joints = dict(zip(training.POLICY_NAMES,training.UNITREE_RL_LAB_G1_DEFAULT_POS))
    joints.update(dict(zip(training.G1_29DOF_JOINTS[22:],training.PAYLOAD_V1_POSTURES[0])))
    data.qpos[3:7] = (1.,0.,0.,0.)
    for name,value in joints.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    training.mujoco.mj_forward(model,data)
    R = data.xmat[model.body('carried_payload').id].reshape(3,3)
    np.testing.assert_allclose(R@training.cup_up_axis(),(0.,0.,1.),atol=1e-4)


def test_fluid_objective_preserves_actor_contract_and_covers_native_yaw_range(training):
    cfg = training.build_env(2,42,objective='fluid')
    assert cfg.observations['actor'].history_length == 5
    assert 'base_lin_vel' not in cfg.observations['actor'].terms
    assert cfg.commands['twist'].yaw_speed_max == .9
    assert cfg.rewards['fluid_carry'].weight < 0.
    assert cfg.rewards['track_angular_velocity'].weight > 0.


def test_episode_resamples_mass_before_writing_physics_and_scopes_arm_gains(training):
    torch = training.torch
    action = SimpleNamespace(mass=torch.zeros(2), held=torch.zeros(2, dtype=torch.bool))
    def sample(ids):
        action.mass[ids] = torch.tensor((0., .35))
        action.held[ids] = torch.tensor((False, True))
    action.sample_episode = sample
    robot = SimpleNamespace(
        find_bodies=lambda _: ([0], ['carried_payload']),
        indexing=SimpleNamespace(body_ids=torch.tensor([0]), ctrl_ids=torch.arange(29)),
        actuator_names=tuple(training.G1_29DOF_JOINTS),
    )
    model = SimpleNamespace(body_mass=torch.zeros(2, 1), body_inertia=torch.zeros(2, 1, 3),
                            actuator_gainprm=torch.full((2, 29, 10), 123.),
                            actuator_biasprm=torch.full((2, 29, 10), 123.))
    env = SimpleNamespace(action_manager=SimpleNamespace(get_term=lambda _: action),
                          scene={'robot': robot}, sim=SimpleNamespace(model=model), num_envs=2, device='cpu')
    training.reset_payload(env, None)
    torch.testing.assert_close(model.body_mass[:, 0], torch.tensor((1e-6, .35)))
    torch.testing.assert_close(model.body_inertia[1, 0], torch.full((3,), .35*.11**2/6))
    assert torch.all(model.actuator_gainprm[0, 15:, 0] == 40.)
    assert torch.all(model.actuator_gainprm[1, 15:, 0] == 160.)
    assert torch.all(model.actuator_biasprm[0, 15:, 2] == -10.)
    assert torch.all(model.actuator_biasprm[1, 15:, 2] == -4.)
    assert torch.all(model.actuator_gainprm[:, :15] == 123.)
def test_native_inertial_configuration_matches_torso_geometry_and_motor_limits(training):
    import numpy as np
    native = training.native_robot_model()
    aligned = training.native_inertial_payload_spec().compile()
    for name in ('torso_link','waist_roll_link','waist_yaw_link'):
        a,b = native.body(name).id,aligned.body(name).id
        assert aligned.body_mass[b] == pytest.approx(native.body_mass[a])
        np.testing.assert_allclose(aligned.body_pos[b],native.body_pos[a])
        np.testing.assert_allclose(aligned.body_inertia[b],native.body_inertia[a])
    cfg = training.build_env(2,42,dynamics='native_inertial',max_delay_steps=2)
    actuators = cfg.scene.entities['robot'].articulation.actuators
    assert actuators[1].effort_limit == 88.
    assert actuators[12].effort_limit == 88.
    assert cfg.actions['joint_pos'].max_delay_steps == 2
