"""Go2 GPU PPO foundation task, separate from the CPU/ROS deployment runtime.

The foundation actor uses proprio-45; --perceptive adds the same occluded sparse
LiDAR used in deployment. Privileged height/contact data is critic/reward-only.
"""
import argparse
from contextlib import nullcontext, redirect_stdout
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import tarfile
import xml.etree.ElementTree as ET

import numpy as np
import torch

from .assets import ASSETS, ROOT, ensure_assets
from .policy import JOINTS, PolicyContract

# ContactMatch expands in physical XML order, independently of policy joint order.
# Every per-foot reward tensor must use this same order.
TRAINING_FEET = ('FL', 'FR', 'RL', 'RR')


def commanded_progress(env):
    """Reward actual signed progress; standing must not win velocity tracking."""
    command = env.command_manager.get_command('twist')[:, :2]
    velocity = env.scene['robot'].data.root_link_lin_vel_b[:, :2]
    norm2 = command.square().sum(dim=1)
    fraction = (command*velocity).sum(dim=1)/norm2.clamp(min=.0064)
    return fraction.clamp(-1., 1.2)*(norm2 > .0064).float()


def commanded_stall(env):
    command = env.command_manager.get_command('twist')[:, :2]
    velocity = env.scene['robot'].data.root_link_lin_vel_b[:, :2]
    norm2 = command.square().sum(dim=1)
    fraction = (command*velocity).sum(dim=1)/norm2.clamp(min=.0064)
    return (1.-fraction/.5).clamp(0., 1.)*(norm2 > .0064)*(env.episode_length_buf*env.step_dt > .5)


def incline_velocity_tracking(env, std=.2):
    # Vertical motion is legitimate on a ramp; do not reward flattening it to zero.
    command = env.command_manager.get_command('twist')[:, :2]
    velocity = env.scene['robot'].data.root_link_lin_vel_b[:, :2]
    return torch.exp(-(command-velocity).square().sum(dim=1)/(std*std))


def stopped_motion(env):
    command = env.command_manager.get_command('twist')
    data = env.scene['robot'].data
    moving = data.root_link_lin_vel_b.square().sum(dim=1)+.1*data.root_link_ang_vel_b.square().sum(dim=1)
    return moving*(command.norm(dim=1) < .02)


class BodyContactBudget:
    """Training contact gate aligned with the independent qualification limits.

    Contact sensing is privileged reward/termination information, never actor input.
    GPU per-geom net-force history approximates the native per-contact impulse.
    """
    def __init__(self, cfg, env):
        self.impulse = torch.zeros(env.num_envs, device=env.device)

    def reset(self, env_ids=None):
        self.impulse[slice(None) if env_ids is None else env_ids] = 0.

    def __call__(self, env):
        force = env.scene['nonfoot_contact'].data.force_history.norm(dim=-1)
        self.impulse += (force*(force > 1.)).sum(dim=(1, 2))*(env.step_dt/force.shape[2])
        return (self.impulse > .1) | (force.amax(dim=(1, 2)) > 25.)


def robot_spec():
    import mujoco
    root = ET.parse(ASSETS/'go2/go2.xml').getroot()
    root.find('compiler').set('meshdir', str(ASSETS/'go2/assets'))
    # mjlab installs explicit PD motors; retaining the original motors duplicates
    # actuation. Physical collision geometry and inertias stay intact.
    for tag in ('actuator', 'keyframe', 'sensor', 'option'):
        child = root.find(tag)
        if child is not None:
            root.remove(child)
    for body in root.iter('body'):
        for index, geom in enumerate(list(body.findall('geom'))):
            if geom.get('class') == 'visual':
                body.remove(geom)
            elif not geom.get('name'):
                geom.set('name', body.get('name', 'body')+f'_collision_{index}')
        if body.get('name', '').endswith('_foot'):
            ET.SubElement(body, 'site', name=body.get('name'), pos='0 0 0', size='.01')
    # No meshes are required by the preserved physical primitives.
    for mesh in list(root.find('asset').findall('mesh')):
        root.find('asset').remove(mesh)
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))


def configure_scene(spec):
    # Native 3.3.6 deployment has no MULTICCD flag. Newer MuJoCo defaults
    # enable it, which also rejects the pinned model's nonzero margins in Warp.
    spec.option.enableflags = 0


def make_env(num_envs=64, terrain='flat', seed=1, perceptive=False):
    from mjlab.actuator import IdealPdActuatorCfg
    from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
    from mjlab.envs import mdp
    from mjlab.envs.mdp.actions.actions import JointPositionAction, JointPositionActionCfg
    from mjlab.managers import RewardTermCfg, TerminationTermCfg
    from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
    from mjlab.managers.scene_entity_config import SceneEntityCfg
    from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RingPatternCfg, RayCastSensorCfg
    from mjlab.tasks.velocity import mdp as velocity_mdp
    from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
    from mjlab.terrains.config import flat, pyramid_stairs, pyramid_stairs_inv, hf_pyramid_slope, hf_pyramid_slope_inv
    from mjlab.terrains import TerrainGeneratorCfg

    contract = PolicyContract()
    c = contract.config
    cfg = make_velocity_env_cfg()
    cfg.seed, cfg.scene.num_envs = seed, num_envs
    cfg.scene.spec_fn = configure_scene
    cfg.scene.entities = {'robot': EntityCfg(
        spec_fn=robot_spec,
        init_state=EntityCfg.InitialStateCfg(pos=(0., 0., .32),
            joint_pos={name: float(q) for name, q in zip(JOINTS, contract.default)}, joint_vel={'.*': 0.}),
        articulation=EntityArticulationInfoCfg(actuators=(IdealPdActuatorCfg(
            target_names_expr=JOINTS, stiffness=c['kp'], damping=c['kd'], effort_limit=c['torque_limit_nm']),))) }
    feet = tuple(leg+'_foot' for leg in TRAINING_FEET)
    for sensor in cfg.scene.sensors:
        if sensor.name == 'terrain_scan':
            sensor.frame.name = 'base_link'
        elif sensor.name == 'foot_height_scan':
            sensor.frame = tuple(ObjRef(type='site', name=name, entity='robot') for name in feet)
            sensor.pattern = RingPatternCfg.single_ring(radius=.025, num_samples=4)
    contact = ContactSensorCfg(name='feet_ground_contact',
        primary=ContactMatch(mode='geom', pattern=TRAINING_FEET, entity='robot'),
        secondary=ContactMatch(mode='body', pattern='terrain'), fields=('found', 'force'),
        reduce='netforce', num_slots=1, track_air_time=True)
    body_contact = ContactSensorCfg(name='nonfoot_contact',
        primary=ContactMatch(mode='geom', pattern='.*_collision_.*', entity='robot'),
        secondary=ContactMatch(mode='body', pattern='terrain'), fields=('found', 'force'),
        reduce='netforce', num_slots=1)
    cfg.scene.sensors += (contact, body_contact)
    if perceptive:
        cfg.rewards['track_linear_velocity'].weight = 3.
        cfg.rewards['track_linear_velocity'].params['std'] = .2
        cfg.rewards['commanded_progress'] = RewardTermCfg(func=commanded_progress, weight=1.5)
        cfg.rewards['commanded_stall'] = RewardTermCfg(func=commanded_stall, weight=-5.)
        from .perception import SparseLidarPattern, TrainingLidarObservation
        cfg.scene.sensors += (RayCastSensorCfg(name='policy_lidar',
            frame=ObjRef(type='body', name='base_link', entity='robot'),
            pattern=SparseLidarPattern(), ray_alignment='yaw', max_distance=6.,
            include_geom_groups=(0,), exclude_parent_body=True),)
        body_contact.history_length = 4
    asset = SceneEntityCfg('robot', joint_names=JOINTS, preserve_order=True)
    terms = {
        'angular_velocity': ObservationTermCfg(func=mdp.base_ang_vel, scale=.25),
        'gravity': ObservationTermCfg(func=mdp.projected_gravity),
        'command': ObservationTermCfg(func=mdp.generated_commands, params={'command_name': 'twist'}),
        'joint_position': ObservationTermCfg(func=mdp.joint_pos_rel, params={'asset_cfg': asset}),
        'joint_velocity': ObservationTermCfg(func=mdp.joint_vel_rel, params={'asset_cfg': asset}, scale=.05),
        'previous_action': ObservationTermCfg(func=mdp.last_action),
    }
    for term in terms.values():
        term.clip = (-100., 100.)
    if perceptive:
        terms['lidar'] = ObservationTermCfg(func=TrainingLidarObservation, params={'corrupt': True})
    cfg.observations['actor'] = ObservationGroupCfg(terms=terms, concatenate_terms=True, enable_corruption=False)
    cfg.observations['critic'] = ObservationGroupCfg(terms={**terms,
        'linear_velocity': ObservationTermCfg(func=mdp.base_lin_vel),
        'height_scan': ObservationTermCfg(func=mdp.height_scan, params={'sensor_name': 'terrain_scan'})},
        concatenate_terms=True, enable_corruption=False)
    class OrderedAction(JointPositionAction):
        def _find_targets(self, action_cfg):
            # mjlab 1.6's joint-action lookup ignores preserve_order. Explicitly
            # resolve our contract rather than following XML's FL/FR/RL/RR order.
            return self._entity.find_joints(action_cfg.actuator_names, preserve_order=True)

    class OrderedActionCfg(JointPositionActionCfg):
        def build(self, env):
            return OrderedAction(self, env)

    action = OrderedActionCfg(entity_name='robot', actuator_names=JOINTS, use_default_offset=True)
    cfg.actions['joint_pos'] = action
    action.actuator_names = JOINTS
    action.scale = {name: float(scale) for name, scale in zip(JOINTS, contract.action_scale)}
    action.clip = {'.*': (-100., 100.)}
    action.preserve_order = True
    cfg.viewer.body_name = 'base_link'
    cfg.rewards['upright'].params['asset_cfg'].body_names = ('base_link',)
    cfg.rewards['body_ang_vel'].params['asset_cfg'].body_names = ('base_link',)
    for name in ('foot_clearance', 'foot_slip'):
        cfg.rewards[name].params['asset_cfg'].site_names = feet
        cfg.rewards[name].params['asset_cfg'].preserve_order = True
    for mode, scale in (('standing', .12), ('walking', .4), ('running', .6)):
        cfg.rewards['pose'].params['std_'+mode] = {'.*': scale}
    cfg.rewards['nonfoot_contact'] = RewardTermCfg(func=velocity_mdp.self_collision_cost,
        weight=-2., params={'sensor_name': 'nonfoot_contact'})
    if perceptive:
        cfg.rewards['nonfoot_contact'].weight = -4.
        cfg.rewards['nonfoot_contact'].params['force_threshold'] = 1.
        cfg.terminations['forceful_body_contact'] = TerminationTermCfg(func=BodyContactBudget)
        cfg.rewards['foot_clearance'].weight = -2.
        cfg.rewards['foot_clearance'].params['target_height'] = .1
        cfg.rewards['foot_swing_height'].weight = -.35
        cfg.rewards['foot_swing_height'].params['target_height'] = .1
        cfg.rewards['pose'].weight = .3
    # Keep the foundation experiment near deployment's speed envelope.
    cfg.commands['twist'].ranges.lin_vel_x = (-.2, .4)
    cfg.commands['twist'].ranges.lin_vel_y = (-.15, .15)
    cfg.commands['twist'].ranges.ang_vel_z = (-.5, .5)
    cfg.events['base_com'].params['asset_cfg'].body_names = ('base_link',)
    cfg.events['foot_friction'].params['asset_cfg'].geom_names = ('FR', 'FL', 'RR', 'RL')
    cfg.events['foot_friction'].params['ranges'] = (.4, .4)
    if perceptive:
        cfg.events['foot_friction'].params['ranges'] = (.35, .65)
        cfg.commands['twist'].resampling_time_range = (6., 10.)
        cfg.commands['twist'].rel_standing_envs = .15
        cfg.commands['twist'].rel_forward_envs = .7
        cfg.commands['twist'].ranges.lin_vel_x = (.08, .35)
        cfg.commands['twist'].ranges.lin_vel_y = (-.05, .05)
        cfg.episode_length_s = 40.
    # Introduce randomization after baseline contract validation, not before it.
    for name in ('push_robot', 'encoder_bias', 'base_com'):
        cfg.events.pop(name)
    cfg.curriculum.pop('command_vel')
    cfg.sim.mujoco.timestep = .005
    cfg.sim.mujoco.integrator = 'euler'
    # Warp's native box CCD rejects the asset's 1 mm margins. Preserve those
    # margins and record this collision-backend difference for native evaluation.
    cfg.sim.mujoco.disableflags = ('multiccd', 'nativeccd')
    cfg.sim.mujoco.cone = 'elliptic'
    cfg.sim.mujoco.impratio = 100.
    cfg.sim.mujoco.iterations = 100
    cfg.sim.nconmax = 128
    cfg.sim.njmax = 2000
    cfg.decimation = 4
    if terrain == 'flat':
        cfg.scene.terrain.terrain_type = 'plane'
        cfg.scene.terrain.terrain_generator = None
        cfg.curriculum = {}
        cfg.terminations.pop('out_of_terrain_bounds')
    elif terrain == 'inclines':
        cfg.scene.terrain.max_init_terrain_level = 1
        gradients = (math.tan(math.radians(2.)), math.tan(math.radians(14.)))
        cfg.scene.terrain.terrain_generator = TerrainGeneratorCfg(
            size=(8., 8.), num_rows=6, num_cols=3, border_width=5., curriculum=True,
            sub_terrains={'flat': flat(proportion=.2),
                'incline': hf_pyramid_slope(proportion=.4, slope_range=gradients, platform_width=2.),
                'decline': hf_pyramid_slope_inv(proportion=.4, slope_range=gradients, platform_width=2.)})
        cfg.rewards['track_linear_velocity'].func = incline_velocity_tracking
        cfg.rewards['track_linear_velocity'].params = {'std': .2}
        cfg.rewards['upright'].params['terrain_sensor_names'] = ('terrain_scan',)
        # Allow the established gait to choose its chassis pitch rather than
        # forcing exact parallel alignment with every local heightfield patch.
        cfg.rewards['upright'].params['std'] = 1.
        cfg.rewards['upright'].weight = .3
        cfg.rewards['foot_slip'].weight = -.3
        cfg.rewards['stopped_motion'] = RewardTermCfg(func=stopped_motion, weight=-2.)
        cfg.commands['twist'].rel_standing_envs = .2
        cfg.commands['twist'].resampling_time_range = (4., 8.)
        # The independent long-ramp probe includes a 20 ms observation delay.
        # Train through 0..40 ms lag, held for multiple control periods.
        from .perception import CoherentActorObservation
        cfg.observations['actor'].terms = {'coherent_state': ObservationTermCfg(
            func=CoherentActorObservation, params={'asset_cfg': asset, 'perceptive': perceptive},
            delay_min_lag=0, delay_max_lag=2, delay_update_period=5, delay_hold_prob=.8)}
    else:
        cfg.scene.terrain.max_init_terrain_level = 2 if perceptive and terrain == 'stairs' else 0
        max_rise = .15 if terrain == 'stairs' else .06
        cfg.scene.terrain.terrain_generator = TerrainGeneratorCfg(
            size=(8., 8.), num_rows=6, num_cols=3, border_width=5., curriculum=True,
            sub_terrains={'flat': flat(proportion=.2),
                'up': pyramid_stairs(proportion=.4, step_height_range=(.02, max_rise), step_width=.4),
                'down': pyramid_stairs_inv(proportion=.4, step_height_range=(.02, max_rise), step_width=.4)})
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--num-envs', type=int, default=64)
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--terrain', choices=('flat', 'small-stairs', 'stairs', 'inclines'), default='flat')
    parser.add_argument('--perceptive', action='store_true', help='622-value actor with occluded LiDAR, noise/dropout and contact-aware curriculum')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--smoke', action='store_true', help='baseline actor rollout and contract checks, no PPO update')
    parser.add_argument('--resume', type=Path, help='PPO checkpoint produced by this task')
    parser.add_argument('--export-only', action='store_true', help='export and verify --resume without further learning')
    parser.add_argument('--initialize-baseline', action='store_true',
                        help='initialize actor weights from pinned policy; critic and optimizer start fresh')
    parser.add_argument('--residual-scale', type=float, default=0.,
                        help='optional maximum deterministic action correction around a frozen baseline')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--quiet', action='store_true', help='write iteration output to training.log')
    args = parser.parse_args()
    if args.num_envs < 1 or args.iterations < 1:
        parser.error('environment count and iterations must be positive')
    if args.resume and args.initialize_baseline:
        parser.error('--resume and --initialize-baseline are mutually exclusive')
    if not math.isfinite(args.residual_scale) or args.residual_scale < 0:
        parser.error('--residual-scale must be nonnegative and finite')
    if args.residual_scale and not (args.initialize_baseline or args.resume):
        parser.error('--residual-scale requires --initialize-baseline or a residual checkpoint')
    if args.export_only and not args.resume:
        parser.error('--export-only requires --resume')
    if args.resume:
        previous = json.loads((args.resume.parent/'manifest.json').read_text())
        if previous['arguments'].get('perceptive', False) != args.perceptive:
            parser.error('resume must preserve the checkpoint observation schema')
        if previous['arguments'].get('residual_scale', 0.) != args.residual_scale:
            parser.error('resume must preserve the checkpoint residual scale')
    if not torch.cuda.is_available():
        parser.error('GPU training requires CUDA; use the isolated .dougie/training environment')
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
    from mjlab.tasks.velocity.config.go1.rl_cfg import unitree_go1_ppo_runner_cfg
    ensure_assets()
    torch.set_num_threads(1)
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('training_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        device=torch.cuda.get_device_name(), policy_contract=PolicyContract().config,
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (ROOT/'dougie_sim').glob('*.py')},
        packages={name: importlib.metadata.version(name) for name in
                  ('mjlab', 'mujoco', 'mujoco-warp', 'warp-lang', 'torch', 'rsl-rl-lib')},
        observation_scope=('actor: proprio-45 plus 10 Hz occluded sparse LiDAR with validity/age' if args.perceptive else
                           'actor: deployment proprio-45')+'; critic: privileged velocity and height',
        reward_foot_order=list(TRAINING_FEET),
        resume_sha256=hashlib.sha256(args.resume.read_bytes()).hexdigest() if args.resume else None,
        qualification='training development only; no new locomotion capability established')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with tarfile.open(output/'source_snapshot.tar.gz', 'w:gz') as archive:
        for name in manifest['source_sha256']:
            archive.add(ROOT/name, arcname=name)
    (output/'requirements.lock.txt').write_text(subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'], text=True))
    cfg = make_env(args.num_envs, args.terrain, args.seed, args.perceptive)
    from mjlab.utils.os import dump_yaml
    dump_yaml(output/'environment.yaml', asdict(cfg))
    started = time.perf_counter()
    env = ManagerBasedRlEnv(cfg, device='cuda:0')
    try:
        contact_order = tuple(n.removeprefix('robot/') for n in env.scene['feet_ground_contact'].primary_names)
        height_order = tuple(f.name.removesuffix('_foot') for f in env.scene['foot_height_scan'].cfg.frame)
        if contact_order != height_order or contact_order != TRAINING_FEET:
            raise RuntimeError(f'per-foot reward ordering mismatch: contact={contact_order}, height={height_order}')
        action_term = env.action_manager.get_term('joint_pos')
        if tuple(action_term._target_names) != JOINTS:
            raise RuntimeError(f'Go2 action order mismatch: {action_term._target_names}')
        wrapped = RslRlVecEnvWrapper(env, clip_actions=100.)
        obs = wrapped.get_observations()
        from .perception import OBSERVATIONS, SCHEMA, SENSOR_CONTRACT
        observation_count = OBSERVATIONS if args.perceptive else 45
        if obs['actor'].shape != (args.num_envs, observation_count):
            raise RuntimeError('training observation does not match deployment contract')
        if args.smoke:
            actor = PolicyContract().load().to('cuda')
            with torch.inference_mode():
                for _ in range(args.iterations*24):
                    actions = actor(obs['actor'][:, :45])
                    obs, reward, dones, _ = wrapped.step(actions)
                    if not torch.isfinite(obs['actor']).all() or not torch.isfinite(reward).all():
                        raise RuntimeError('nonfinite GPU rollout')
            result = dict(mode='contract_smoke', transitions=args.iterations*24*args.num_envs,
                          finite=True, actor_observations=observation_count, action_order=list(action_term._target_names))
        else:
            agent = unitree_go1_ppo_runner_cfg()
            agent.experiment_name = 'dougie_go2_foundation'
            agent.logger = 'tensorboard'
            agent.upload_model = False
            agent.seed = args.seed
            agent.max_iterations = args.iterations
            agent.save_interval = max(1, min(50, args.iterations))
            if args.initialize_baseline:
                agent.actor.obs_normalization = False  # Pinned actor has identity normalization.
                agent.actor.distribution_cfg['init_std'] = .3 if args.perceptive and args.terrain != 'inclines' else .15
                agent.algorithm.learning_rate = 1e-4
                agent.algorithm.entropy_coef = .002
                if args.residual_scale:
                    agent.actor.distribution_cfg['init_std'] = .05
            agent_config = asdict(agent)
            if args.resume:
                agent_config = json.loads((args.resume.parent/'agent.json').read_text())
            (output/'agent.json').write_text(json.dumps(agent_config, indent=2)+'\n')
            runner = MjlabOnPolicyRunner(wrapped, agent_config, str(output), 'cuda:0')
            if args.residual_scale:
                from .residual_policy import BoundedResidual
                actor = runner.alg.get_policy()
                if actor.obs_normalization:
                    raise RuntimeError('residual baseline requires unnormalized deployment observations')
                # Reuse the existing trainable MLP so PPO's optimizer keeps its parameters.
                correction = actor.mlp
                last = [layer for layer in correction.modules() if isinstance(layer, torch.nn.Linear)][-1]
                torch.nn.init.zeros_(last.weight)
                torch.nn.init.zeros_(last.bias)
                original = PolicyContract().load().to('cuda')
                actor.mlp = BoundedResidual(original, correction, args.residual_scale)
                with torch.inference_mode():
                    torch.testing.assert_close(actor(obs), original(obs['actor'][:, :45]), rtol=1e-5, atol=1e-6)
            elif args.initialize_baseline:
                original = PolicyContract().load().to('cuda')
                actor = runner.alg.get_policy()
                weights = {k.removeprefix('actor.'): v for k, v in original.state_dict().items()}
                if args.perceptive:
                    expanded = torch.zeros_like(actor.mlp.state_dict()['0.weight'])
                    expanded[:, :45] = weights['0.weight']
                    weights['0.weight'] = expanded
                actor.mlp.load_state_dict(weights)
                with torch.inference_mode():
                    torch.testing.assert_close(actor(obs), original(obs['actor'][:, :45]), rtol=1e-5, atol=1e-6)
            if args.resume:
                runner.load(str(args.resume))
            if not args.export_only:
                with (output/'training.log').open('w') as log:
                    with redirect_stdout(log) if args.quiet else nullcontext():
                        runner.learn(args.iterations, init_at_random_ep_len=True)
            runner.save(str(output/'checkpoint.pt'))
            actor = runner.alg.get_policy().eval()
            runner.export_policy_to_jit(str(output), 'policy.pt')
            observations = wrapped.get_observations()
            with torch.inference_mode():
                expected = actor(observations).cpu()
                exported = torch.jit.load(str(output/'policy.pt'), map_location='cpu').eval()
                actual = exported(observations['actor'].cpu())
                torch.testing.assert_close(actual, expected, rtol=1e-4, atol=1e-5)
            np.savez_compressed(output/'export_parity.npz', observations=observations['actor'].cpu().numpy(),
                                actions=expected.numpy())
            deployment = {**PolicyContract().config, 'name': output.name,
                          'artifact': 'policy.pt', 'sha256': hashlib.sha256((output/'policy.pt').read_bytes()).hexdigest(),
                          'qualification': 'UNQUALIFIED training checkpoint; independent native evaluation required'}
            if args.perceptive:
                deployment.update(schema_version=2, observation_schema=SCHEMA, range_sensor=SENSOR_CONTRACT)
            (output/'policy.json').write_text(json.dumps(deployment, indent=2)+'\n')
            iterations = 0 if args.export_only else args.iterations
            result = dict(mode='policy_export' if args.export_only else 'ppo_training', iterations=iterations,
                          transitions=iterations*24*args.num_envs, export_parity=True)
        torch.cuda.synchronize()
        result.update(wall_time_s=time.perf_counter()-started,
                      peak_torch_memory_bytes=torch.cuda.max_memory_allocated())
        (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(dict(output=str(output), **result), indent=2))
    finally:
        env.close()


if __name__ == '__main__':
    main()
