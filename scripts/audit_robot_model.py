#!/usr/bin/env python3
"""Check nominal G1 joint/hand/IMU assets against the maintained description."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    description = ET.parse(ROOT/'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf').getroot()
    limits = {joint.get('name'):joint.find('limit') for joint in description.findall('joint')}
    errors,models = [],{}
    for filename in ('g1_29dof.xml','g1_29dof_brainco_physical.xml'):
        model = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1'/filename))
        for name in G1_29DOF_JOINTS:
            joint = model.joint(name)
            expected = [float(limits[name].get(key)) for key in ('lower','upper')]
            if not np.allclose(model.jnt_range[joint.id],expected,atol=1e-4):
                errors.append(filename+': joint range differs: '+name)
            if np.count_nonzero(model.actuator_trnid[:,0]==joint.id)!=1:
                errors.append(filename+': body joint actuator mapping differs: '+name)
        sensors = {}
        for name in ('imu_gyro','imu_acc','camera_gyro','camera_accel','livox_gyro','livox_accel'):
            sensor = model.sensor(name)
            sensors[name] = int(model.sensor_dim[sensor.id])
            if sensors[name]!=3:
                errors.append(filename+': invalid IMU dimensions: '+name)
        fingers = sum('proximal' in (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i) or '') for i in range(model.njnt))
        if 'physical' in filename and fingers!=10:
            errors.append('articulated model must include ten proximal finger joints')
        models[filename] = dict(mass_kg=float(sum(model.body_mass)),joints=model.njnt,
                               body_actuators=model.nu,proximal_fingers=fingers,sensors=sensors)
    result = dict(passed=not errors,errors=errors,models=models,mujoco_version=mujoco.__version__,
        description_mass_kg=sum(float(node.get('value')) for node in description.findall('link/inertial/mass')),
        caveats=['URDF and native torso/waist revisions differ; native-inertial training is explicit',
                 'Hand current/friction, sensor noise/extrinsics and firmware are not physically calibrated'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
