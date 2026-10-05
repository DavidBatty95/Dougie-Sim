#!/usr/bin/env python3
"""Fine-tune the deployed 480/29 G1 actor for externally controlled arms/load.

Run through scripts/run_mjlab_docker.sh (uv), with MJLAB_WITH_GPUS=1.
Training truth is confined to rewards/critic/evaluation. Actor inputs retain the
deployed IMU, joint encoders, command and action-history contract.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnx
from onnx import numpy_helper
import torch
import warp as wp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import (
    G1_29DOF_JOINTS, G1_UNITREE_RL_LAB_BODY_KP, G1_UNITREE_RL_LAB_BODY_KD,
)
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import (
    UNITREE_RL_LAB_G1_JOINT_IDS_MAP, UNITREE_RL_LAB_G1_DEFAULT_POS,
)
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.asset_zoo.robots.unitree_g1.g1_constants import get_spec
from mjlab.envs import ManagerBasedRlEnv
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg, RecomputeLevel, requires_model_fields
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.config.g1.env_cfgs import unitree_g1_flat_env_cfg
from mjlab.tasks.velocity.config.g1.rl_cfg import unitree_g1_ppo_runner_cfg
from mjlab.tasks.velocity.mdp import StandCreepStopVelocityCommandCfg
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
from mjlab.utils.os import dump_yaml

POLICY_NAMES = tuple(G1_29DOF_JOINTS[i] for i in UNITREE_RL_LAB_G1_JOINT_IDS_MAP)
ARM_POLICY_IDS = tuple(i for i, sdk in enumerate(UNITREE_RL_LAB_G1_JOINT_IDS_MAP) if sdk >= 15)
SOURCE = ROOT / 'vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx'
URDF = ROOT / 'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf'


def arm_pose_bank(include_reaches=False):
    poses = [
        [-.92, -1.04, .01, -.20, 1.45, .89, -.65],
        [-1.10, -.60, .10, -.10, 1.10, .94, -.70],
        [-.54, -.76, .05, -.27, .94, .62, -.43],
    ]
    if not include_reaches:
        return poses
    kin = ArmKinematics(URDF.read_text(), tool_offset=(.12,.01,0.))
    rest = dict(zip(POLICY_NAMES, UNITREE_RL_LAB_G1_DEFAULT_POS))
    targets = [(.30,-.13,.36)] + [(x,y,z) for x in (.30,.36,.42,.48)
               for y in (-.08,-.15,-.22) for z in (.20,.28,.36,.40)]
    for target in targets:
        solution,error = kin.solve(target, rest, np.eye(3))
        if error < .012:
            poses.append([solution[name] for name in G1_29DOF_JOINTS[22:]])
    return poses


def payload_spec():
    spec = get_spec()
    for joint in spec.joints:
        if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            joint.damping = [.05, 0., 0.]
            joint.frictionloss = .1 if 'wrist' in joint.name else .2
    payload = spec.body('right_wrist_yaw_link').add_body(name='carried_payload', pos=[.19, .01, 0.])
    payload.add_geom(name='payload_collision', type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[.055, .055, .055], mass=.15, rgba=[.85, .08, .06, 1.])
    return spec


@dataclass(kw_only=True)
class PayloadActionCfg(JointPositionActionCfg):
    max_payload: float = .5
    profile: str = 'mixed'
    pose_index: int = 0
    pose_bank: str = 'carry'
    arm_transitions: bool = False

    def build(self, env):
        return PayloadAction(self, env)


class PayloadAction(JointPositionAction):
    def _find_targets(self, cfg):
        return self._entity.find_joints(POLICY_NAMES, preserve_order=True)

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.held = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.mass = torch.zeros(self.num_envs, device=self.device)
        self.arm_commands = self._offset.expand(self.num_envs, -1).clone()
        self.arm_targets = self.arm_commands.clone()
        self.arm_ids = torch.tensor(ARM_POLICY_IDS, device=self.device)
        self.joint_ids = self._target_ids
        self.postures = torch.tensor(arm_pose_bank(cfg.pose_bank == 'reach'), device=self.device)
        self.next_pose_step = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.right_ids = torch.tensor([POLICY_NAMES.index(name) for name in G1_29DOF_JOINTS[22:]], device=self.device)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if not hasattr(self, 'held'):
            return
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        self.arm_commands[env_ids] = self._offset.expand(self.num_envs, -1)[env_ids]

    def sample_episode(self, env_ids):
        count = len(env_ids)
        random = torch.rand(count, device=self.device)
        if self.cfg.profile == 'mixed':
            self.held[env_ids] = random >= .2
            self.mass[env_ids] = torch.where(random >= .4,
                .05 + torch.rand(count, device=self.device) * (self.cfg.max_payload - .05), 0.)
        else:
            self.held[env_ids] = self.cfg.profile != 'unloaded'
            self.mass[env_ids] = {'unloaded': 0., 'held_empty': 0., 'loaded': .15, 'heavy': .35}[self.cfg.profile]
        self.arm_commands[env_ids] = self._offset.expand(self.num_envs, -1)[env_ids]
        self.arm_targets[env_ids] = self.arm_commands[env_ids]
        poses = self.postures[torch.randint(len(self.postures), (count,), device=self.device)]
        if self.cfg.profile != 'mixed':
            poses = self.postures[self.cfg.pose_index].expand(count, -1)
        noise = (torch.rand_like(poses) - .5) * .12 if self.cfg.profile == 'mixed' else 0.
        self.arm_targets[env_ids[:, None], self.right_ids[None, :]] = poses + noise
        self.next_pose_step[env_ids] = round(8. / self._env.step_dt)

    def process_actions(self, actions):
        super().process_actions(actions)
        if self.cfg.profile == 'mixed' and self.cfg.arm_transitions:
            ids = torch.where(self.held & (self._env.episode_length_buf >= self.next_pose_step))[0]
            if len(ids):
                poses = self.postures[torch.randint(len(self.postures), (len(ids),), device=self.device)]
                self.arm_targets[ids[:, None], self.right_ids[None, :]] = poses
                self.next_pose_step[ids] += round(8. / self._env.step_dt)
        self.arm_commands += torch.clamp(self.arm_targets - self.arm_commands,
                                          -.35 * self._env.step_dt, .35 * self._env.step_dt)
        self._processed_actions[:, self.arm_ids] = torch.where(self.held[:, None],
            self.arm_commands[:, self.arm_ids], self._processed_actions[:, self.arm_ids])


@requires_model_fields('body_mass', 'body_inertia', 'actuator_gainprm', 'actuator_biasprm', recompute=RecomputeLevel.set_const)
def reset_payload(env, env_ids):
    action = env.action_manager.get_term('joint_pos')
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    # Reset events run before ActionManager.reset; sample here, not there.
    action.sample_episode(env_ids)
    robot = env.scene['robot']
    local_ids, _ = robot.find_bodies('carried_payload')
    body = robot.indexing.body_ids[local_ids[0]]
    mass = action.mass[env_ids].clamp_min(1e-6)
    env.sim.model.body_mass[env_ids, body] = mass
    env.sim.model.body_inertia[env_ids, body] = mass[:, None] * (.11**2 / 6.)
    for sdk, name in enumerate(G1_29DOF_JOINTS[15:], start=15):
        local = next(i for i, actuator in enumerate(robot.actuator_names) if name in actuator)
        actuator = robot.indexing.ctrl_ids[local]
        kp = torch.where(action.held[env_ids], 160., G1_UNITREE_RL_LAB_BODY_KP[sdk])
        kd = torch.where(action.held[env_ids], 4., G1_UNITREE_RL_LAB_BODY_KD[sdk])
        env.sim.model.actuator_gainprm[env_ids, actuator, 0] = kp
        env.sim.model.actuator_biasprm[env_ids, actuator, 1] = -kp
        env.sim.model.actuator_biasprm[env_ids, actuator, 2] = -kd


def joint_positions(env):
    action = env.action_manager.get_term('joint_pos')
    return env.scene['robot'].data.joint_pos[:, action.joint_ids] - action._offset


def joint_velocities(env):
    action = env.action_manager.get_term('joint_pos')
    return env.scene['robot'].data.joint_vel[:, action.joint_ids]


def build_env(num_envs, seed, profile='mixed', pose_index=0, pose_bank='carry', arm_transitions=False):
    cfg = unitree_g1_flat_env_cfg()
    cfg.seed = seed
    cfg.scene.num_envs = num_envs
    cfg.episode_length_s = 30.
    cfg.decimation = 10
    cfg.sim.mujoco.timestep = .002
    robot = cfg.scene.entities['robot']
    robot.spec_fn = payload_spec
    robot.init_state.pos = (0., 0., .793)
    robot.init_state.joint_pos = dict(zip(POLICY_NAMES, UNITREE_RL_LAB_G1_DEFAULT_POS))
    actuators = []
    for sdk, name in enumerate(G1_29DOF_JOINTS):
        effort = 5. if 'wrist_pitch' in name or 'wrist_yaw' in name else 25. if sdk >= 15 else 139. if 'knee' in name or 'hip_roll' in name else 50. if 'ankle' in name or 'waist' in name else 88.
        actuators.append(BuiltinPositionActuatorCfg(target_names_expr=(name,),
            stiffness=160. if sdk >= 15 and profile != 'unloaded' else G1_UNITREE_RL_LAB_BODY_KP[sdk],
            damping=4. if sdk >= 15 and profile != 'unloaded' else G1_UNITREE_RL_LAB_BODY_KD[sdk],
            effort_limit=effort, armature=.01))
    robot.articulation.actuators = tuple(actuators)
    cfg.actions = {'joint_pos': PayloadActionCfg(entity_name='robot', actuator_names=('.*',),
                    scale=.25, use_default_offset=True, profile=profile, pose_index=pose_index,
                    pose_bank=pose_bank, arm_transitions=arm_transitions)}
    actor_terms = {
        'base_ang_vel': ObservationTermCfg(func=mdp.builtin_sensor, params={'sensor_name': 'robot/imu_ang_vel'}, scale=.2),
        'projected_gravity': ObservationTermCfg(func=mdp.projected_gravity),
        'velocity_commands': ObservationTermCfg(func=mdp.generated_commands, params={'command_name': 'twist'}),
        'joint_pos_rel': ObservationTermCfg(func=joint_positions),
        'joint_vel_rel': ObservationTermCfg(func=joint_velocities, scale=.05),
        'last_action': ObservationTermCfg(func=mdp.last_action),
    }
    cfg.observations = {
        'actor': ObservationGroupCfg(terms=actor_terms, concatenate_terms=True, history_length=5, enable_corruption=False),
        'critic': ObservationGroupCfg(terms={**actor_terms, 'base_lin_vel': ObservationTermCfg(func=mdp.base_lin_vel)}, concatenate_terms=True, enable_corruption=False),
    }
    cfg.commands['twist'] = StandCreepStopVelocityCommandCfg(entity_name='robot',
        resampling_time_range=(2., 5.), heading_command=False, debug_vis=False,
        low_speed_probability=.15, moderate_speed_probability=.30,
        lateral_probability=.15, arc_probability=.10, diagonal_probability=.10,
        reverse_probability=.25, low_speed_range=(.12, .25), moderate_speed_range=(.25, .65),
        lateral_speed_min=.15, lateral_speed_max=.45, yaw_speed_min=.3, yaw_speed_max=.65,
        diagonal_speed_range=(.15, .4), initial_standing_probability=.25,
        moving_stop_probability=.3, standing_hold_probability=.15,
        ranges=StandCreepStopVelocityCommandCfg.Ranges(lin_vel_x=(-.45, .65), lin_vel_y=(-.45, .45), ang_vel_z=(-.65, .65)))
    cfg.events = {name: cfg.events[name] for name in ('reset_base', 'reset_robot_joints')}
    cfg.events['reset_base'].params['pose_range'] = {'x': (0., 0.), 'y': (0., 0.), 'z': (0., 0.), 'yaw': (-math.pi, math.pi)}
    cfg.events['payload'] = EventTermCfg(func=reset_payload, mode='reset')
    cfg.curriculum = {}
    cfg.rewards.pop('pose', None)  # External hand poses must not be penalized as bad posture.
    cfg.rewards['track_linear_velocity'].params['std'] = .25
    cfg.rewards['track_angular_velocity'].params['std'] = .35
    cfg.rewards['action_rate_l2'].weight = -.02
    return cfg


def load_onnx_actor(actor, path):
    model = onnx.load(str(path))
    state = {item.name.removeprefix('actor.'): torch.from_numpy(numpy_helper.to_array(item).copy())
             for item in model.graph.initializer}
    actor.mlp.load_state_dict(state, strict=True)
    # A conversion parity check prevents a silent architecture/order conversion.
    from onnx.reference import ReferenceEvaluator
    samples = np.random.default_rng(0).normal(0., .2, (8, 480)).astype(np.float32)
    expected = ReferenceEvaluator(model).run(None, {model.graph.input[0].name: samples})[0]
    with torch.no_grad():
        actual = actor.mlp(torch.as_tensor(samples, device=next(actor.parameters()).device)).cpu().numpy()
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=2e-5)
    return float(np.max(np.abs(actual - expected)))


def initialize_arm_inputs(actor):
    """Start from the nominal-arm ablation while retaining trainable arm inputs.

    Unlike a runtime observation mask, these columns receive gradients and learn
    from the real measured arm state during PPO. Record this as a separate
    warm-start baseline so its effect is not attributed to RL training.
    """
    columns = [start + history*29 + joint for start in (45, 190, 335)
               for history in range(5) for joint in ARM_POLICY_IDS]
    with torch.no_grad():
        actor.mlp[0].weight[:, columns] = 0.


def evaluate(runner, wrapped, profile, output):
    env = wrapped.unwrapped
    profiles = ('unloaded', 'held_empty', 'loaded', 'heavy') if profile == 'mixed' else (profile,)
    schedule = (
        ('settle', 5., (0., 0., 0.)), ('forward', 4., (.4, 0., 0.)),
        ('stop_forward', 2., (0., 0., 0.)), ('turn_left', 3., (0., 0., .5)),
        ('turn_right', 3., (0., 0., -.5)), ('reverse', 3., (-.3, 0., 0.)),
        ('lateral', 3., (0., .3, 0.)), ('diagonal', 3., (.3, -.2, 0.)),
        ('final_stop', 2., (0., 0., 0.)),
    )
    reports = []
    runner.alg.actor.eval()
    for case in profiles:
        action = env.action_manager.get_term('joint_pos')
        action.cfg.profile = case
        observations, _ = wrapped.reset()
        command = env.command_manager.get_term('twist')
        alive = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
        phases = []
        for name, duration, values in schedule:
            count, error_xy, error_yaw, speed_peak = 0, 0., 0., 0.
            for _ in range(round(duration / env.step_dt)):
                command.time_left.fill_(1000.)
                command.is_standing_env[:] = not any(values)
                command.vel_command_b[:] = torch.tensor(values, device=env.device)
                with torch.no_grad():
                    controls = runner.alg.actor(observations)
                    observations, _, dones, _ = wrapped.step(controls)
                alive &= ~dones.bool()
                velocity = env.scene['robot'].data.root_link_lin_vel_b[:, :2]
                yaw_rate = env.scene['robot'].data.root_link_ang_vel_b[:, 2]
                if alive.any():
                    error_xy += torch.linalg.vector_norm(velocity[alive] - torch.tensor(values[:2], device=env.device), dim=-1).sum().item()
                    error_yaw += (yaw_rate[alive] - values[2]).abs().sum().item()
                    speed_peak = max(speed_peak, torch.linalg.vector_norm(velocity[alive], dim=-1).max().item())
                    count += int(alive.sum())
            phases.append(dict(phase=name, command=values, survivors=int(alive.sum()),
                               mean_planar_error_m_s=error_xy/count if count else None,
                               mean_yaw_error_rad_s=error_yaw/count if count else None,
                               peak_planar_speed_m_s=speed_peak))
        report = dict(profile=case, environments=env.num_envs, survivors=int(alive.sum()),
                      pose_index=action.cfg.pose_index,
                      failures=int((~alive).sum()), phases=phases,
                      payload_mass_kg={'unloaded': 0., 'held_empty': 0., 'loaded': .15, 'heavy': .35}[case])
        reports.append(report)
        output.write_text(json.dumps(reports, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'train', 'evaluate'), default='smoke')
    parser.add_argument('--num-envs', type=int, default=64)
    parser.add_argument('--iterations', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--arm-init', choices=('pretrained', 'zero'), default='pretrained')
    parser.add_argument('--pose-index', type=int, default=0, help='Fixed evaluation pose (3 is the close pregrasp reach)')
    parser.add_argument('--pose-bank', choices=('carry','reach'), default='carry')
    parser.add_argument('--arm-transitions', action='store_true', help='Resample held-arm targets every eight seconds during training')
    parser.add_argument('--profile', choices=('mixed', 'unloaded', 'held_empty', 'loaded', 'heavy'), default='mixed')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.num_envs <= 0 or args.iterations <= 0 or args.seed < 0 or args.pose_index < 0:
        parser.error('Environment/iteration counts must be positive and seed/pose index nonnegative')
    if args.pose_bank == 'carry' and args.pose_index > 2:
        parser.error('Pose indices above 2 require --pose-bank reach')
    torch.set_num_threads(4)
    wp.config.kernel_cache_dir = str(ROOT / 'vendor/mjlab/logs/.warp_payload_cache')
    torch.manual_seed(args.seed)
    # vendor/ is a writable host bind mount; /workspace/logs is ephemeral.
    output = args.output or ROOT / 'vendor/mjlab/logs/payload_training' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True, exist_ok=False)
    cfg = build_env(args.num_envs, args.seed, args.profile, args.pose_index, args.pose_bank, args.arm_transitions)
    agent = unitree_g1_ppo_runner_cfg()
    agent.actor.obs_normalization = False
    agent.critic.obs_normalization = True
    agent.actor.distribution_cfg['init_std'] = .1
    agent.algorithm.learning_rate = 1e-5
    agent.algorithm.schedule = 'fixed'
    agent.algorithm.entropy_coef = .002
    agent.save_interval = 25
    agent.logger = 'tensorboard'
    agent.seed = args.seed
    agent.experiment_name = 'g1_payload'
    manifest = dict(mode=args.mode, seed=args.seed, num_envs=args.num_envs, iterations=args.iterations,
                    profile=args.profile, source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                    arm_input_initialization=args.arm_init,
                    evaluation_pose_index=args.pose_index,
                    pose_bank=args.pose_bank, arm_transitions=args.arm_transitions,
                    optimizer_learning_rates=dict(actor=1e-5, critic=3e-4),
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    checkpoint=None if args.checkpoint is None else str(args.checkpoint),
                    actor_contract='unitree_rl_lab_g1_29dof: 480 observations / 29 actions; measured arms',
                    payload_mass_range_kg=[0., .5], hand_offset_m=[.19, .01, 0.], base_pinning=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output / 'training_source.py').write_bytes(Path(__file__).read_bytes())
    for source in (URDF, ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/arm_kinematics.py'):
        (output / source.name).write_bytes(source.read_bytes())
    dump_yaml(output / 'env.yaml', asdict(cfg))
    dump_yaml(output / 'agent.yaml', asdict(agent))
    env = ManagerBasedRlEnv(cfg, device='cuda:0')
    wrapped = RslRlVecEnvWrapper(env, clip_actions=None)
    runner = VelocityOnPolicyRunner(wrapped, asdict(agent), str(output), device='cuda:0')
    parity = load_onnx_actor(runner.alg.actor, SOURCE)
    print(f'ONNX conversion maximum error: {parity}', flush=True)
    if args.checkpoint:
        runner.load(str(args.checkpoint), load_cfg={'actor': True, 'critic': True,
                    'optimizer': False, 'iteration': True, 'rnd': False})
    elif args.arm_init == 'zero':
        initialize_arm_inputs(runner.alg.actor)
    # The actor is pretrained, but ONNX has no critic checkpoint. Fit that
    # fresh value function faster without applying its learning rate to actor.
    runner.alg.optimizer = torch.optim.Adam([
        {'params': runner.alg.actor.parameters(), 'lr': 1e-5},
        {'params': runner.alg.critic.parameters(), 'lr': 3e-4},
    ])
    action = env.action_manager.get_term('joint_pos')
    print('Payload reset audit:', dict(held=int(action.held.sum()), mass_min=float(action.mass.min()), mass_max=float(action.mass.max())), flush=True)
    try:
        if args.mode in ('smoke', 'train'):
            runner.learn(num_learning_iterations=2 if args.mode == 'smoke' else args.iterations,
                         init_at_random_ep_len=False)
            runner.save(str(output / 'final.pt'))
            actor = copy.deepcopy(runner.alg.actor.mlp).cpu().eval()
            torch.onnx.export(actor, torch.zeros(1, 480), str(output / 'policy.onnx'),
                              input_names=['obs'], output_names=['actions'], opset_version=17, dynamo=False)
            print(f'Checkpoint: {output / "final.pt"}', flush=True)
        else:
            report = evaluate(runner, wrapped, args.profile, output / 'evaluation.json')
            (output / 'evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    finally:
        env.close()


if __name__ == '__main__':
    main()
