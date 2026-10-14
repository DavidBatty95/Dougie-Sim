import hashlib
import json
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from dougie_sim.assets import ROOT
from dougie_sim.policy import JOINTS
from dougie_sim.robot_description import Go2Description
from dougie_sim.simulation import Simulation


def test_all_demo_configs_enable_the_go2_robot_model():
    for name in ('dougie.rviz', 'dougie_robot.rviz', 'dougie_radiation.rviz', 'dougie_facility.rviz'):
        displays = yaml.safe_load((ROOT/'config'/name).read_text())['Visualization Manager']['Displays']
        models = [d for d in displays if d['Class'] == 'rviz_default_plugins/RobotModel' and d['Enabled']]
        assert len(models) == 1
        assert models[0]['Description Topic']['Value'] == '/robot_description'
        assert models[0]['Visual Enabled'] and not models[0]['Collision Enabled']


def test_rviz_uses_verified_go2_meshes_and_twelve_joints():
    sim = Simulation(controller='stand')
    description = ET.fromstring(Go2Description(sim).xml)
    assert description.get('name') == 'dougie_unitree_go2'
    assert {j.get('name') for j in description.findall("joint[@type='revolute']")} == set(JOINTS)
    assert len(description.findall('link')) == 17
    manifest = json.loads((ROOT/'config/dougie_assets.json').read_text())
    hashes = {Path(item['path']).name: item['sha256'] for item in manifest['files']}
    meshes = description.findall('link/visual/geometry/mesh')
    assert len(meshes) >= 20
    for mesh in meshes:
        path = Path(unquote(urlparse(mesh.get('filename')).path))
        assert hashlib.sha256(path.read_bytes()).hexdigest() == hashes[path.name]
        assert mesh.get('scale') == '1 1 1'


def test_urdf_forward_kinematics_and_published_transforms_match_mujoco():
    sim = Simulation(controller='stand')
    adapter = Go2Description(sim)
    description = ET.fromstring(adapter.xml)
    rng = np.random.default_rng(28)
    for _ in range(4):
        sim.data.qpos[sim.qadr] += rng.uniform(-.08, .08, 12)
        sim.data.qpos[:3] = rng.uniform(-1, 1, 3)
        q = Rotation.from_rotvec(rng.uniform(-.3, .3, 3)).as_quat()
        sim.data.qpos[3:7] = q[[3, 0, 1, 2]]
        mujoco.mj_forward(sim.model, sim.data)
        root = np.eye(4)
        root[:3, :3] = Rotation.from_quat(q).as_matrix()
        root[:3, 3] = sim.data.qpos[:3]
        frames = {'base_link': root}
        runtime = {child: (p, quat) for _, child, p, quat in adapter.transforms()}
        for joint in description.findall('joint'):
            origin = joint.find('origin')
            relative = np.eye(4)
            relative[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
            relative[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
            if joint.get('type') == 'revolute':
                index = sim.model.joint(joint.get('name')).id
                angle = sim.data.qpos[sim.model.jnt_qposadr[index]]
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                relative[:3, :3] @= Rotation.from_rotvec(axis*angle).as_matrix()
            child = joint.find('child').get('link')
            frame = frames[joint.find('parent').get('link')]@relative
            frames[child] = frame
            np.testing.assert_allclose(frame[:3, 3], sim.data.body(child).xpos, atol=1e-10)
            np.testing.assert_allclose(frame[:3, :3], sim.data.body(child).xmat.reshape(3, 3), atol=1e-10)
            p, quat = runtime[child]
            np.testing.assert_allclose(p, relative[:3, 3], atol=1e-10)
            np.testing.assert_allclose(Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix(), relative[:3, :3], atol=1e-10)
