#!/usr/bin/env python3
"""Isolate ONNX deployment on native MuJoCo geometry, without ROS scheduling.

This is a dynamics diagnostic, not a sensor-only mission acceptance run.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
from unitree_g1_mujoco_bridge.policy_safety import clamp_joint_targets
from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--delay-steps',type=int,default=0,help='Whole 20 ms policy periods of application delay')
    parser.add_argument('--yaw',type=float,default=0.,help='Yaw command after five seconds standing')
    parser.add_argument('--held-empty',action='store_true',help='Externally compose payload_v1 arms, without an attached load')
    args = parser.parse_args()
    if args.delay_steps<0 or args.duration<=0:
        parser.error('nonnegative delay and positive duration required')
    scene = ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = .002
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'transport_unitree_rl_lab_stand')
    if key<0:
        raise ValueError('native stand keyframe missing')
    mujoco.mj_resetDataKeyframe(model,data,key)
    mujoco.mj_forward(model,data)
    joint_ids = np.array([model.joint(name).id for name in G1_29DOF_JOINTS])
    qids,dids = model.jnt_qposadr[joint_ids],model.jnt_dofadr[joint_ids]
    aids = np.array([np.flatnonzero(model.actuator_trnid[:,0]==index)[0] for index in joint_ids])
    kp,kd = np.asarray(G1_UNITREE_RL_LAB_BODY_KP),np.asarray(G1_UNITREE_RL_LAB_BODY_KD)
    kp,kd = kp.copy(),kd.copy()
    if args.held_empty:
        kp[15:],kd[15:] = 160.,4.
    adapter = UnitreeRLLabG1PolicyAdapter()
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.policy),sess_options=options,providers=['CPUExecutionProvider'])
    target = np.asarray(adapter.sdk_default_positions)
    arm_target = target.copy()
    arm_goal = target.copy()
    arm_goal[22:] = PAYLOAD_V1_POSTURES[0]
    action = np.zeros(29)
    queue = deque()
    rows = []
    start = data.qpos[:2].copy()
    fallen = False
    for step in range(round(args.duration/.002)):
        if step%10==0:
            command = [0.,0.,args.yaw if data.time>=5. else 0.]
            observation = adapter.build_observation(command,data.qpos[qids],data.qvel[dids],
                data.sensor('imu_gyro').data,data.sensor('imu_quat').data,action)
            proposed = session.run(None,{session.get_inputs()[0].name:observation})[0].reshape(-1)
            queue.append(proposed)
            if len(queue)>args.delay_steps:
                action = queue.popleft()
                target = np.asarray(clamp_joint_targets(adapter.action_targets(action)))
            if args.held_empty:
                arm_target += np.clip(arm_goal-arm_target,-.35*.02,.35*.02)
                target[15:] = arm_target[15:]
            quat = data.sensor('imu_quat').data
            tilt = float(np.arccos(np.clip(1.-2.*(quat[1]**2+quat[2]**2),-1.,1.)))
            rows.append(dict(time=float(data.time),xy=data.qpos[:2].tolist(),height=float(data.qpos[2]),
                             tilt=tilt,yaw=float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1.-2*(quat[2]**2+quat[3]**2))),
                             command=command,action_norm=float(np.linalg.norm(action))))
            if tilt>.9 or data.qpos[2]<.45:
                fallen = True
                break
        data.ctrl[aids] = kp*(target-data.qpos[qids])-kd*data.qvel[dids]
        mujoco.mj_step(model,data)
    result = dict(policy=str(args.policy),policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
                  scene=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                  mujoco_version=mujoco.__version__,scope='native geometry, synchronous CPU MuJoCo; no ROS or camera; optional prescribed empty-hand arm composition',
                  delay_steps=args.delay_steps,yaw=args.yaw,held_empty=args.held_empty,requested_duration_s=args.duration,
                  yaw_displacement_rad=float(np.unwrap([row['yaw'] for row in rows])[-1]-rows[0]['yaw']),
                  elapsed_s=float(data.time),fallen=fallen,displacement_m=float(np.linalg.norm(data.qpos[:2]-start)),
                  max_tilt_rad=max(row['tilt'] for row in rows),samples=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))


if __name__=='__main__':
    main()
