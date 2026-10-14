"""RViz URDF and articulated TF from the checksum-verified Go2 model.

Visual origins come from the original MJCF, not MuJoCo's recentered compiled
mesh frames. Physical joint axes, limits and link origins come from the model.
"""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .assets import ASSETS
from .policy import JOINTS


def numbers(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def rpy(quaternion):
    q = np.asarray(quaternion, dtype=float)
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_euler('xyz')


class Go2Description:
    def __init__(self, sim):
        self.sim = sim
        source = ET.parse(ASSETS/'go2/go2.xml').getroot()
        root_body = source.find("worldbody/body[@name='base_link']")
        robot = ET.Element('robot', name='dougie_unitree_go2')
        mesh_files = {m.get('name', m.get('file', '').rsplit('.', 1)[0]): m.get('file')
                      for m in source.findall('asset/mesh')}
        for material in source.findall('asset/material'):
            entry = ET.SubElement(robot, 'material', name=material.get('name'))
            ET.SubElement(entry, 'color', rgba=material.get('rgba', '1 1 1 1'))
        self.links = []
        self.joints = []

        def visit(body, parent=None):
            name = body.get('name')
            self.links.append(name)
            body_id = sim.model.body(name).id
            link = ET.SubElement(robot, 'link', name=name)
            for index, geom in enumerate(body.findall('geom')):
                if geom.get('class') != 'visual':
                    continue
                mesh = ASSETS/'go2/assets'/mesh_files[geom.get('mesh')]
                if not mesh.is_file():
                    raise FileNotFoundError(mesh)
                visual = ET.SubElement(link, 'visual', name=f'{name}_visual_{index}')
                quaternion = np.fromstring(geom.get('quat', '1 0 0 0'), sep=' ')
                ET.SubElement(visual, 'origin', xyz=geom.get('pos', '0 0 0'), rpy=numbers(rpy(quaternion)))
                geometry = ET.SubElement(visual, 'geometry')
                ET.SubElement(geometry, 'mesh', filename=mesh.resolve().as_uri(), scale='1 1 1')
                if geom.get('material'):
                    ET.SubElement(visual, 'material', name=geom.get('material'))
            if parent is not None:
                joint_element = body.find('joint')
                joint_id = sim.model.joint(joint_element.get('name')).id if joint_element is not None else None
                if joint_id is not None and not np.allclose(sim.model.jnt_pos[joint_id], 0.):
                    raise ValueError('Go2 URDF adapter requires body-origin joints')
                joint_name = joint_element.get('name') if joint_element is not None else name+'_fixed'
                joint = ET.SubElement(robot, 'joint', name=joint_name, type='revolute' if joint_id is not None else 'fixed')
                ET.SubElement(joint, 'parent', link=parent)
                ET.SubElement(joint, 'child', link=name)
                position = sim.model.body_pos[body_id].copy()
                quaternion = sim.model.body_quat[body_id].copy()
                ET.SubElement(joint, 'origin', xyz=numbers(position), rpy=numbers(rpy(quaternion)))
                if joint_id is not None:
                    ET.SubElement(joint, 'axis', xyz=numbers(sim.model.jnt_axis[joint_id]))
                    lo, hi = sim.model.jnt_range[joint_id]
                    ET.SubElement(joint, 'limit', lower=str(lo), upper=str(hi), effort='23.5', velocity='30')
                self.joints.append((parent, name, position, quaternion, joint_id))
            for child in body.findall('body'):
                visit(child, name)

        visit(root_body)
        if {sim.model.joint(j).name for *_, j in self.joints if j is not None} != set(JOINTS):
            raise ValueError('RViz model joint set differs from the deployed Go2')
        self.xml = ET.tostring(robot, encoding='unicode')

    def transforms(self):
        """Current joint-coordinate transforms, consistent with /joint_states."""
        for parent, child, position, zero_quaternion, joint in self.joints:
            quaternion = zero_quaternion.copy()
            if joint is not None:
                angle = float(self.sim.data.qpos[self.sim.model.jnt_qposadr[joint]])
                delta = np.r_[math.cos(angle/2), self.sim.model.jnt_axis[joint]*math.sin(angle/2)]
                mujoco.mju_mulQuat(quaternion, zero_quaternion, delta)
            yield parent, child, position, quaternion
