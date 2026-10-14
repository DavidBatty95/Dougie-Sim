"""Go2 torque-driven MuJoCo simulation. No root-pose control or base pinning.

Policy observation/action contract adapted from fan-ziqi/rl_sar's Go2 robot_lab
configuration (Copyright 2024-2025 Ziqi Fan, Apache-2.0); see assets manifest.
"""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

from .assets import ASSETS
from .policy import JOINTS, PolicyContract

DEFAULT = np.tile([0., .8, -1.5], 4)
ACTION_SCALE = np.tile([.125, .25, .25], 4)
WORLDS = ('flat', 'room', 'office', 'reactor', 'stairs', 'mezzanine', 'facility')


def rotation(quaternion):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.asarray(quaternion, dtype=float))
    return matrix.reshape(3, 3)


def scene_xml(world='flat', people=0, radiation_sources=None, layout_seed=0, landmark_count=0, landmark_seed=0):
    if world not in WORLDS or not 0 <= people <= 8:
        raise ValueError('invalid world or people count')
    root = ET.parse(ASSETS / 'go2/go2.xml').getroot()
    root.attrib['model'] = 'Dougie Unitree Go2'
    root.find('compiler').set('meshdir', str(ASSETS / 'go2/assets'))
    root.find('option').set('timestep', '0.005')
    # Reject invisible visual meshes in ray scans; collision geometry is group 3.
    root.remove(root.find('keyframe'))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'headlight', ambient='.4 .4 .4', diffuse='.7 .7 .7')
    ET.SubElement(visual, 'global', azimuth='130', elevation='-25')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', name='floor_tex', type='2d', builtin='checker',
                  rgb1='.12 .2 .25', rgb2='.2 .3 .35', width='256', height='256')
    ET.SubElement(asset, 'material', name='floor_mat', texture='floor_tex', texrepeat='12 12', texuniform='true')
    body = root.find('worldbody')
    ET.SubElement(body, 'light', pos='0 0 6', dir='0 0 -1', directional='true')
    ET.SubElement(body, 'geom', name='floor', type='plane', size='0 0 .1',
                  material='floor_mat', friction='.8 .02 .01')
    if world in ('room', 'office'):
        obstacles = [(6, 0, .1, 6), (-6, 0, .1, 6), (0, 6, 6, .1), (0, -6, 6, .1)]
        if world == 'office':
            obstacles += [(1.5, 3.5, .1, 2.4), (1.5, -3.5, .1, 2.4),
                          (-2.5, 2.5, 1., .5), (3.5, -2.5, .5, 1.)]
        for i, (x, y, sx, sy) in enumerate(obstacles):
            ET.SubElement(body, 'geom', name=f'obstacle_{i}', type='box',
                          pos=f'{x} {y} .65', size=f'{sx} {sy} .65', rgba='.6 .65 .7 1')
    if world in ('reactor', 'stairs', 'mezzanine', 'facility'):
        from .reactor import add_reactor, stairs
        if world in ('reactor', 'mezzanine', 'facility'):
            # Declare geom user storage before compiling attenuation-bearing slabs.
            if world == 'facility':
                from .facility import add_facility, source_description
                add_facility(body, layout_seed)
            elif world == 'mezzanine':
                from .multilevel import add_mezzanine, source_description
                add_mezzanine(body)
            else:
                add_reactor(body)
            from .reactor import radiation_description
            description = (source_description() if world in ('mezzanine', 'facility') else radiation_description()) if radiation_sources is None else radiation_sources
            ET.SubElement(root, 'size', nuser_site='3', nuser_geom='1')
            custom = ET.SubElement(root, 'custom')
            ET.SubElement(custom, 'numeric', name='radiation_background_usv_h', data=str(description['background_usv_h']))
            for i, source in enumerate(description['sources']):
                leak = source.get('kind', 'hotspot') == 'leak'
                ET.SubElement(body, 'site', name='radiation_source_'+source.get('id', str(i)),
                    type='sphere', pos=' '.join(map(str, source['position'])), size='.09', group='5',
                    rgba='1 .55 .05 1' if leak else '1 .05 .1 1',
                    user=f"{source['strength_usv_h_m2']} {source['core_radius_m']} {int(leak)}")
            if world == 'reactor':
                for name, attenuation in (('shield_west', 3.5), ('shield_south', 3.5), ('service_partition', 2.)):
                    body.find(f"geom[@name='{name}']").set('user', str(attenuation))
        else:
            stairs(body)
    for i in range(people):
        person = ET.SubElement(body, 'body', name=f'person_{i}', mocap='true', pos=f'{2+i*.5} 2 .85')
        ET.SubElement(person, 'geom', name=f'person_geom_{i}', type='capsule',
                      size='.25 .6', rgba='.9 .45 .15 1')
    if landmark_count:
        if world != 'facility':
            raise ValueError('discovery landmarks currently require the facility world')
        from .landmarks import add_facility_landmarks
        add_facility_landmarks(root, landmark_count, landmark_seed)
    return ET.tostring(root, encoding='unicode')


class Simulation:
    def __init__(self, world='flat', people=0, controller='policy', *, model_xml=None, radiation_sources=None, policy_config=None, layout_seed=0, landmark_count=0, landmark_seed=0):
        if controller not in ('policy', 'stand'):
            raise ValueError('unknown controller')
        self.policy_contract = PolicyContract(policy_config)
        self.default = self.policy_contract.default
        self.scene_xml = model_xml if model_xml is not None else scene_xml(world, people, radiation_sources, layout_seed, landmark_count, landmark_seed)
        self.model = mujoco.MjModel.from_xml_string(self.scene_xml)
        self.control_decimation = round(.02 / self.model.opt.timestep)
        if self.control_decimation < 1 or not math.isclose(self.control_decimation*self.model.opt.timestep, .02):
            raise ValueError('physics timestep must divide the 20 ms policy period')
        self.data = mujoco.MjData(self.model)
        self.people = people
        self.joint_ids = np.array([self.model.joint(name).id for name in JOINTS])
        self.qadr = self.model.jnt_qposadr[self.joint_ids]
        self.vadr = self.model.jnt_dofadr[self.joint_ids]
        self.actuators = np.array([self.model.actuator(name.removesuffix('_joint')).id for name in JOINTS])
        self.base = self.model.body('base_link').id
        # Exclude the whole robot (not just the base body) from virtual LiDAR.
        for geom in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom])
            while body_id and body_id != self.base:
                body_id = int(self.model.body_parentid[body_id])
            if body_id == self.base and self.model.geom_group[geom] != 2:
                self.model.geom_group[geom] = 4
        self.data.qpos[:7] = [0, 0, .32, 1, 0, 0, 0]
        self.data.qpos[self.qadr] = self.default
        self.action = np.zeros(12)
        self.target = self.default.copy()
        self.counter = 0
        self.controller = controller
        torch.set_num_threads(1)
        self.policy = self.policy_contract.load() if controller == 'policy' else None
        self.last_command = np.zeros(3)
        self._policy_scan = None
        self.ray_box_plane_ids = [i for i in range(self.model.ngeom)
            if self.model.geom_group[i] not in (2, 4) and self.model.geom_rgba[i, 3] > 0
            and self.model.geom_type[i] in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE)]
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        R = rotation(self.data.qpos[3:7])
        proprio = np.clip(np.concatenate((self.data.qvel[3:6] * .25,
                      R.T @ np.array([0., 0., -1.]), command,
                       self.data.qpos[self.qadr] - self.default,
                       self.data.qvel[self.vadr] * .05, self.action)),
                       -self.policy_contract.config['observation_clip'],
                       self.policy_contract.config['observation_clip']).astype(np.float32)
        if not self.policy_contract.perceptive:
            return proprio
        from .perception import PERIOD_S, range_features
        if self._policy_scan is None or self.data.time-self._policy_scan.stamp_s >= PERIOD_S-1e-9:
            self._policy_scan = self.lidar_packet(sparse=True)
        return np.r_[proprio, range_features(self._policy_scan.distances, self.data.time-self._policy_scan.stamp_s)]

    def step(self, command=(0., 0., 0.)):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('command must contain three finite velocities')
        command = np.clip(command, [-.5, -.3, -.8], [.6, .3, .8])
        if self.data.time < 1.:
            command = np.zeros(3)
        if self.counter % self.control_decimation == 0:
            self.last_command = command.copy()
            if self.policy is not None and self.data.time >= .5:
                with torch.inference_mode():
                    action = self.policy(torch.from_numpy(self.observation(command)).unsqueeze(0)).numpy().reshape(-1)
                if action.shape != (12,) or not np.isfinite(action).all():
                    raise RuntimeError('invalid Go2 policy output')
                clip = self.policy_contract.config['action_clip']
                self.action = np.clip(action, -clip, clip)
                self.target = self.default + self.policy_contract.action_scale * self.action
            else:
                self.target = self.default.copy()
        c = self.policy_contract.config
        torque = c['kp'] * (self.target - self.data.qpos[self.qadr]) - c['kd'] * self.data.qvel[self.vadr]
        self.data.ctrl[self.actuators] = np.clip(torque, -c['torque_limit_nm'], c['torque_limit_nm'])
        for i in range(self.people):
            self.data.mocap_pos[i] = [2. + i * .5, 2. * math.sin(.35 * self.data.time + i), .85]
        mujoco.mj_step(self.model, self.data)
        self.counter += 1
        if not np.isfinite(self.data.qpos).all() or self.data.qpos[2] < .13 or rotation(self.data.qpos[3:7])[2, 2] < .45:
            raise RuntimeError('Go2 fell or physics state became invalid')

    def scan(self, rays=180, max_range=8.):
        """Ideal, horizontal virtual range scanner; not a Go2 L1 sensor model."""
        yaw = math.atan2(self.data.xmat[self.base, 3], self.data.xmat[self.base, 0])
        start = self.data.qpos[:3].copy()
        start[2] += .12
        group = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
        angles = np.linspace(-math.pi, math.pi, rays, endpoint=False)+yaw
        directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(rays)))
        distances = np.empty(rays)
        ids = np.empty(rays, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, start, directions.reshape(-1), group,
                          1, self.base, ids, distances, rays, max_range)
        distances[(distances < .05) | (distances > max_range)] = np.inf
        return distances.tolist()

    def lidar3d(self, max_range=6.):
        packet = self.lidar_packet(max_range)
        return packet.points, packet.supports

    def lidar_packet(self, max_range=6., *, sparse=False):
        """Organized ideal 3-D returns and normals estimated from adjacent rays.

        The simulator returns no object IDs, layout, or radiation-source metadata.
        Scan is gravity aligned using ideal orientation, rather than an L1 model.
        """
        azimuth = np.linspace(-math.pi, math.pi, 240, endpoint=False)
        elevation = np.r_[-np.arctan2(.45, np.arange(.12, 6.1, .12)), [0., .15, .35, .6]]
        if sparse:
            from .perception import beam_angles
            azimuth, elevation = beam_angles()
        yaw = math.atan2(self.data.xmat[self.base, 3], self.data.xmat[self.base, 0])
        a, e = np.meshgrid(azimuth+yaw, elevation)
        directions = np.stack((np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)), axis=-1)
        start = self.data.qpos[:3].copy()
        start[2] += .12
        n = a.size
        distances = np.empty(n)
        ids = np.empty(n, dtype=np.int32)
        groups = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
        mujoco.mj_multiRay(self.model, self.data, start, directions.reshape(-1), groups,
                          1, self.base, ids, distances, n, max_range)
        from .sensing import correct_box_plane_rays
        correct_box_plane_rays(self.model, self.data, start, directions, distances, self.ray_box_plane_ids)
        distances[distances > max_range] = -1.
        distances = distances.reshape(a.shape)
        from .sensing import RangePacket, support_mask
        return RangePacket(float(self.data.time), self.counter, start, directions, distances,
                           max_range, support_mask(start, directions, distances, max_range))

    def metrics(self):
        return dict(simulation_time_s=float(self.data.time), base_position=self.data.qpos[:3].tolist(),
                    upright=float(rotation(self.data.qpos[3:7])[2, 2]), controller=self.controller,
                    command=self.last_command.tolist(), joints=len(JOINTS),
                    odometry_source='mujoco_ground_truth', scan_source='ideal_mujoco_raycast',
                    policy=self.policy_contract.identity())
