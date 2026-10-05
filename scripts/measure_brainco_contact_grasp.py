#!/usr/bin/env python3
"""Contact-only hand bench, with explicitly prescribed wrist motion (not a mobile demo)."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from brainco_hand_model import SCENES, ROOT, CHANNELS, add_hand


def fixture(size, x, depth):
    root = ET.Element('mujoco',model='brainco_contact_bench')
    ET.SubElement(root,'compiler',angle='radian',meshdir=str(SCENES/'meshes'))
    ET.SubElement(root,'option',timestep='.001',integrator='implicitfast',iterations='100')
    ET.SubElement(root,'asset')
    world = ET.SubElement(root,'worldbody')
    height = 1.08
    ET.SubElement(world,'geom',name='table',type='plane',size='1 1 .01',pos=f'0 0 {height-depth-size/2}',friction='1 .005 .0001')
    wrist = ET.SubElement(world,'body',name='right_wrist_yaw_link',mocap='true',pos=f'0 0 {height}',quat='.70710678 -.70710678 0 0')
    add_hand(root,wrist,'right')
    box = ET.SubElement(world,'body',name='box',pos=f'{x} 0 {height-depth}')
    ET.SubElement(box,'freejoint',name='box_free')
    ET.SubElement(box,'geom',name='box',type='box',size=' '.join([str(size/2)]*3),mass='.15',friction='1 .005 .0001')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


def run(size,x,depth):
    model = fixture(size,x,depth)
    data = mujoco.MjData(model)
    assert not np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    joints = [model.joint('right_'+name+'_joint').id for name in CHANNELS]
    qids,dofs = model.jnt_qposadr[joints],model.jnt_dofadr[joints]
    lo,hi = model.jnt_range[joints].T
    reference = np.zeros(6)
    speed = np.array((2.5303,2.6175,2.2685,2.2685,2.2685,2.2685))
    limits = np.array((.18,.08,.18,.18,.18,.18))
    box = model.body('box').id
    observations = []
    initial = None
    for step in range(9000):
        t = step*.001
        if t < 1.:
            command = np.array((0.,min(1.,t*2),0.,0.,0.,0.))
        elif t < 7.:
            fingers = min(1.,(t-1.)/1.0)
            thumb = min(1.,max(0.,(t-1.5)/1.0))
            command = np.array((thumb,1.,fingers,fingers,fingers,fingers))
        else:
            command = np.zeros(6)
        desired = lo+command*(hi-lo)
        reference += np.clip(desired-reference,-speed*.001,speed*.001)
        data.qfrc_applied[dofs] = np.clip(4*(reference-data.qpos[qids])-.04*data.qvel[dofs],-limits,limits)
        data.mocap_pos[0,2] = 1.08+min(.15,max(0.,t-3.)*.05)
        mujoco.mj_step(model,data)
        if step == 2999:
            initial = data.xpos[box].copy()
        if step % 100 == 0:
            contacts = set()
            self_contacts = []
            for contact in data.contact:
                bodies = [int(model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]
                if box in bodies:
                    other = bodies[1] if bodies[0] == box else bodies[0]
                    contacts.add(model.body(other).name)
                elif all(body > 0 for body in bodies):
                    self_contacts.append([*[model.body(body).name for body in bodies],float(contact.dist)])
            observations.append(dict(time=t,box=data.xpos[box].tolist(),contacts=sorted(contacts),
                                     self_contacts=self_contacts,
                                     fingers=((data.qpos[qids]-lo)/(hi-lo)).tolist()))
    held = [row for row in observations if 6. <= row['time'] < 7.]
    lift = min(row['box'][2]-initial[2] for row in held)
    opposed = all(any('thumb' in name for name in row['contacts']) and
                  any(any(finger in name for finger in ('index','middle','ring','pinky')) for name in row['contacts']) for row in held)
    result = dict(role='prescribed_wrist_contact_bench',box_size_m=size,mass_kg=.15,x=x,depth=depth,
                  minimum_held_lift_m=lift,opposing_finger_contacts=opposed,
                  released_drop_m=held[-1]['box'][2]-observations[-1]['box'][2],
                  success=bool(lift>.10 and opposed and held[-1]['box'][2]-observations[-1]['box'][2]>.08),
                  object_welds=False,observations=observations)
    result['held_self_contacts'] = held[-1]['self_contacts']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size',type=float,default=.05)
    parser.add_argument('--x',type=float,default=.19)
    parser.add_argument('--depth',type=float,default=.045)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result = run(args.size,args.x,args.depth)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='observations'},indent=2))
    return 0 if result['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
