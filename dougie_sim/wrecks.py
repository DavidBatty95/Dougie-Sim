"""Freeze a failed robot's actual articulated pose as a static collision body."""
from copy import deepcopy
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def freeze_robot(sim,identifier):
    original=ET.fromstring(sim.scene_xml).find("worldbody/body[@name='base_link']")
    if original is None:
        raise ValueError('robot body is missing')
    body=deepcopy(original)
    prefix=f'wreck_{identifier}'
    for node in body.iter('body'):
        index=sim.model.body(node.get('name')).id
        parent=int(sim.model.body_parentid[index])
        R=sim.data.xmat[index].reshape(3,3)
        if node is body:
            position=sim.data.xpos[index]
            attitude=R
        else:
            parent_R=sim.data.xmat[parent].reshape(3,3)
            position=parent_R.T@(sim.data.xpos[index]-sim.data.xpos[parent])
            attitude=parent_R.T@R
        quaternion=np.empty(4)
        mujoco.mju_mat2Quat(quaternion,attitude.reshape(-1))
        node.set('name',prefix+'_'+node.get('name'))
        node.set('pos',' '.join(map(str,position)))
        node.set('quat',' '.join(map(str,quaternion)))
        for child in list(node):
            if child.tag in ('joint','freejoint','inertial','site','camera'):
                node.remove(child)
    for i,geom in enumerate(body.iter('geom')):
        geom.set('name',f'{prefix}_geom_{i}')
        geom.attrib.pop('material',None)
        geom.set('rgba','.48 .18 .14 1')
        geom.set('user','0')  # A wreck is an obstacle, not an invented radiation shield.
    bounds=[]
    for index in sim.robot_geoms:
        if sim.model.geom_group[index]==2:
            continue
        kind=sim.model.geom_type[index]
        size=sim.model.geom_size[index].copy()
        if kind==mujoco.mjtGeom.mjGEOM_SPHERE:
            size[:]=size[0]
        elif kind in (mujoco.mjtGeom.mjGEOM_CAPSULE,mujoco.mjtGeom.mjGEOM_CYLINDER):
            size=np.array([size[0],size[0],size[1]+(size[0] if kind==mujoco.mjtGeom.mjGEOM_CAPSULE else 0.)])
        extent=np.abs(sim.data.geom_xmat[index].reshape(3,3))@size
        bounds.extend((sim.data.geom_xpos[index]-extent,sim.data.geom_xpos[index]+extent))
    return dict(id=identifier,position=sim.data.qpos[:3].tolist(),
        quaternion=sim.data.qpos[3:7].tolist(),
        bounds=[np.min(bounds,axis=0).tolist(),np.max(bounds,axis=0).tolist()],
        xml=ET.tostring(body,encoding='unicode'),
        model='Frozen articulated failure pose; static collision and range geometry')


def add_wrecks(worldbody,wrecks):
    for wreck in wrecks:
        element=ET.fromstring(wreck['xml'])
        if element.tag!='body' or not element.get('name','').startswith('wreck_') or any(
                node.tag in ('joint','freejoint') for node in element.iter()):
            raise ValueError('invalid static wreck geometry')
        worldbody.append(element)


def origin_blocked(wrecks):
    lower,upper=np.array([-.45,-.4,-.1]),np.array([.45,.4,.8])
    return any(np.all(np.asarray(w['bounds'][1])>=lower) and np.all(np.asarray(w['bounds'][0])<=upper) for w in wrecks)
