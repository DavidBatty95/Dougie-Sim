#!/usr/bin/env python3
"""Compare training-source rigid body properties with the native robot model."""
import json
import mujoco
import numpy as np

from g1_payload_training import ROOT,payload_spec,G1_29DOF_JOINTS


def main():
    native = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'))
    training = payload_spec().compile()
    differences = []
    for name in G1_29DOF_JOINTS:
        a,b = native.joint(name).id,training.joint(name).id
        body_a,body_b = native.jnt_bodyid[a],training.jnt_bodyid[b]
        for label,first,second in (
            ('mass',native.body_mass[body_a],training.body_mass[body_b]),
            ('inertia',native.body_inertia[body_a],training.body_inertia[body_b]),
            ('ipos',native.body_ipos[body_a],training.body_ipos[body_b]),
            ('body_pos',native.body_pos[body_a],training.body_pos[body_b]),
            ('axis',native.jnt_axis[a],training.jnt_axis[b]),
            ('range',native.jnt_range[a],training.jnt_range[b]),
        ):
            if not np.allclose(first,second,rtol=1e-5,atol=1e-6):
                differences.append(dict(joint=name,field=label,native=np.asarray(first).tolist(),training=np.asarray(second).tolist()))
    for sensor in ('imu_gyro','imu_quat'):
        sid = native.sensor(sensor).id
        site = native.sensor_objid[sid]
        differences.append(dict(native_sensor=sensor,site=mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_SITE,site),
            body=mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_BODY,native.site_bodyid[site]),site_quat=native.site_quat[site].tolist()))
    print(json.dumps(differences,indent=2))


if __name__=='__main__':
    main()
