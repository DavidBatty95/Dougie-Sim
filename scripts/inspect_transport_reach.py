#!/usr/bin/env python3
"""Offline robot-geometry audit; simulator poses are never mission inputs."""
from pathlib import Path
import json
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics


def main():
    model = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('unitree_rl_lab_stand').id)
    mujoco.mj_forward(model,data)
    kin = ArmKinematics((ROOT/'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf').read_text())
    joints = {model.joint(i).name:float(data.qpos[model.jnt_qposadr[i]]) for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE}
    wrist = data.body('right_wrist_yaw_link')
    pelvis = data.body('pelvis')
    actual = (wrist.xpos-pelvis.xpos)@pelvis.xmat.reshape(3,3)
    predicted = kin.forward(joints)[0]
    mesh = model.mesh('right_rubber_hand').id
    geom = next(i for i in range(model.ngeom) if model.geom_dataid[i] == mesh and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH)
    start,count = model.mesh_vertadr[mesh],model.mesh_vertnum[mesh]
    points = (model.mesh_vert[start:start+count]@data.geom_xmat[geom].reshape(3,3).T+data.geom_xpos[geom]-wrist.xpos)@wrist.xmat.reshape(3,3)
    print(json.dumps(dict(urdf_wrist=predicted.tolist(),sim_wrist=actual.tolist(),fk_error_m=float(np.linalg.norm(actual-predicted)),
                         rubber_hand_bounds_wrist=[points.min(axis=0).tolist(),points.max(axis=0).tolist()]),indent=2))
    for target in ((.43,-.15,.34),(.39,-.15,.21),(.30,-.15,.34),(.28,-.08,.36)):
        solution,error = kin.solve(target,joints)
        print('wrist target',target,'error',error,'solution',solution)
    kin.tool_offset = np.array((.12,.01,0.))
    for target in ((.36,-.15,.34),(.36,-.15,.21),(.28,-.15,.36)):
        solution,error = kin.solve(target,joints,np.eye(3))
        print('hand target',target,'error',error,'orientation error',np.linalg.norm(kin.forward(dict(joints,**solution))[1]-np.eye(3)), 'solution',solution)
    for target in ((.30,-.136,.334),(.39,-.22,.335),(.36,-.20,.37)):
        solution,error = kin.solve(target,joints,np.eye(3))
        contacts = {}
        for fraction in np.linspace(0.,1.,41):
            for name,value in solution.items():
                joint = model.joint(name)
                data.qpos[joint.qposadr[0]] = joints[name]+fraction*(value-joints[name])
            mujoco.mj_forward(model,data)
            for contact in data.contact:
                bodies = tuple(model.body(model.geom_bodyid[g]).name for g in (contact.geom1,contact.geom2))
                contacts[bodies] = min(contacts.get(bodies,0.),float(contact.dist))
        print('native swept self contacts',target,contacts)
    scene = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_lab.xml'))
    state = mujoco.MjData(scene)
    for gap in (.32,.26,.22):
        mujoco.mj_resetDataKeyframe(scene,state,scene.key('transport_unitree_rl_lab_stand').id)
        state.qpos[0] = 1.45-gap
        state.qpos[1] = .15
        mujoco.mj_forward(scene,state)
        contacts = []
        for contact in state.contact:
            names = [scene.body(scene.geom_bodyid[g]).name for g in (contact.geom1,contact.geom2)]
            if any(name.startswith('station_') for name in names):
                contacts.append((names,float(contact.dist)))
        print('standing table gap',gap,'station contacts',contacts)


if __name__ == '__main__':
    main()
