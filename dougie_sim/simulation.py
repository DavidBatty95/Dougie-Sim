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

JOINTS = tuple(f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
               for joint in ('hip', 'thigh', 'calf'))
DEFAULT = np.tile([0., .8, -1.5], 4)
ACTION_SCALE = np.tile([.125, .25, .25], 4)
WORLDS = ('flat', 'room', 'office', 'reactor', 'stairs')


def rotation(quaternion):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.asarray(quaternion, dtype=float))
    return matrix.reshape(3, 3)


def scene_xml(world='flat', people=0):
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
    if world in ('reactor', 'stairs'):
        from .reactor import add_reactor, stairs
        if world == 'reactor':
            add_reactor(body)
            from .reactor import radiation_description
            ET.SubElement(root, 'size', nuser_site='2', nuser_geom='1')
            for i, source in enumerate(radiation_description()['sources']):
                ET.SubElement(body, 'site', name=f'radiation_source_{i}',
                    pos=' '.join(map(str, source['position'])), size='.08', group='5',
                    user=f"{source['strength_usv_h_m2']} {source['core_radius_m']}")
            for name, attenuation in (('shield_west', 3.5), ('shield_south', 3.5), ('service_partition', 2.)):
                body.find(f"geom[@name='{name}']").set('user', str(attenuation))
        else:
            stairs(body)
    for i in range(people):
        person = ET.SubElement(body, 'body', name=f'person_{i}', mocap='true', pos=f'{2+i*.5} 2 .85')
        ET.SubElement(person, 'geom', name=f'person_geom_{i}', type='capsule',
                      size='.25 .6', rgba='.9 .45 .15 1')
    return ET.tostring(root, encoding='unicode')


class Simulation:
    def __init__(self, world='flat', people=0, controller='policy'):
        self.model = mujoco.MjModel.from_xml_string(scene_xml(world, people))
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
        self.data.qpos[self.qadr] = DEFAULT
        self.action = np.zeros(12)
        self.target = DEFAULT.copy()
        self.counter = 0
        self.controller = controller
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(ASSETS / 'policy/go2.pt'), map_location='cpu').eval() if controller == 'policy' else None
        self.last_command = np.zeros(3)
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        R = rotation(self.data.qpos[3:7])
        return np.clip(np.concatenate((self.data.qvel[3:6] * .25,
                      R.T @ np.array([0., 0., -1.]), command,
                      self.data.qpos[self.qadr] - DEFAULT,
                      self.data.qvel[self.vadr] * .05, self.action)), -100, 100).astype(np.float32)

    def step(self, command=(0., 0., 0.)):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('command must contain three finite velocities')
        command = np.clip(command, [-.5, -.3, -.8], [.6, .3, .8])
        if self.data.time < 1.:
            command = np.zeros(3)
        if self.counter % 4 == 0:
            self.last_command = command.copy()
            if self.policy is not None and self.data.time >= .5:
                with torch.inference_mode():
                    action = self.policy(torch.from_numpy(self.observation(command)).unsqueeze(0)).numpy().reshape(-1)
                if action.shape != (12,) or not np.isfinite(action).all():
                    raise RuntimeError('invalid Go2 policy output')
                self.action = np.clip(action, -100, 100)
                self.target = DEFAULT + ACTION_SCALE * self.action
            else:
                self.target = DEFAULT.copy()
        torque = 20. * (self.target - self.data.qpos[self.qadr]) - .5 * self.data.qvel[self.vadr]
        self.data.ctrl[self.actuators] = np.clip(torque, -23.5, 23.5)
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
        geom = np.empty(1, dtype=np.int32)
        ranges = []
        for angle in np.linspace(-math.pi, math.pi, rays, endpoint=False):
            direction = np.array([math.cos(yaw + angle), math.sin(yaw + angle), 0.])
            distance = mujoco.mj_ray(self.model, self.data, start, direction, group, 1, self.base, geom)
            ranges.append(float(distance) if 0.05 <= distance <= max_range else float('inf'))
        return ranges

    def lidar3d(self, max_range=6.):
        """Organized ideal 3-D returns and normals estimated from adjacent rays.

        The simulator returns no object IDs, layout, or radiation-source metadata.
        Scan is gravity aligned using ideal orientation, rather than an L1 model.
        """
        azimuth = np.linspace(-math.pi, math.pi, 240, endpoint=False)
        elevation = np.r_[-np.arctan2(.45, np.arange(.12, 6.1, .12)), [0., .15, .35, .6]]
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
        distances = distances.reshape(a.shape)
        valid = (distances > .05) & (distances <= max_range)
        points = start+directions*distances[:, :, None]
        across = np.roll(points, -1, axis=1)-np.roll(points, 1, axis=1)
        radial = np.roll(points, -1, axis=0)-np.roll(points, 1, axis=0)
        normals = np.cross(across, radial)
        lengths = np.linalg.norm(normals, axis=2)
        horizontal = (np.abs(normals[:, :, 2])/np.maximum(lengths, 1e-12) > .92)
        horizontal &= valid & np.roll(valid, 1, axis=0) & np.roll(valid, -1, axis=0)
        horizontal &= np.roll(valid, 1, axis=1) & np.roll(valid, -1, axis=1)
        horizontal &= (np.linalg.norm(radial, axis=2) < .65)
        horizontal[[0, -1], :] = False
        return points[valid], points[horizontal]

    def metrics(self):
        return dict(simulation_time_s=float(self.data.time), base_position=self.data.qpos[:3].tolist(),
                    upright=float(rotation(self.data.qpos[3:7])[2, 2]), controller=self.controller,
                    command=self.last_command.tolist(), joints=len(JOINTS),
                    odometry_source='mujoco_ground_truth', scan_source='ideal_mujoco_raycast')
