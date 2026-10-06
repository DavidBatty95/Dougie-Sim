#!/usr/bin/env python3
"""Generate articulated BrainCo hands from the checked-in URDF, not rubber props."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1'
URDF = ROOT/'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf'
CHANNELS = ('thumb_proximal','thumb_metacarpal','index_proximal','middle_proximal','ring_proximal','pinky_proximal')


def quaternion(rpy):
    r,p,y = [float(v)/2 for v in rpy.split()]
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return ' '.join(map(str,(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)))


def inertial(link,body):
    source = link.find('inertial')
    if source is None:
        return
    origin,tensor = source.find('origin'),source.find('inertia')
    if origin is not None and origin.get('rpy','0 0 0') != '0 0 0':
        raise ValueError('Rotated inertia requires explicit tensor conversion')
    ET.SubElement(body,'inertial',pos=origin.get('xyz','0 0 0') if origin is not None else '0 0 0',
        mass=source.find('mass').get('value'),fullinertia=' '.join(tensor.get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))


def add_hand(root,wrist,side):
    description = ET.parse(URDF).getroot()
    links = {link.get('name'):link for link in description.findall('link')}
    children = {}
    for joint in description.findall('joint'):
        children.setdefault(joint.find('parent').get('link'),[]).append(joint)
    asset = root.find('asset')
    equalities = root.find('equality')
    if equalities is None:
        equalities = ET.SubElement(root,'equality')
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root,'contact')
    created = []

    def visit(parent,parent_link):
        for source in children.get(parent_link,[]):
            name,child = source.get('name'),source.find('child').get('link')
            if parent_link.endswith('wrist_yaw_link') and child not in (f'{side}_base_link',f'{side}_base2_link'):
                continue
            origin = source.find('origin')
            body = ET.SubElement(parent,'body',name=child,pos=origin.get('xyz','0 0 0'),quat=quaternion(origin.get('rpy','0 0 0')))
            # Adjacent CAD housings overlap at their hinges. Explicitly retain
            # this exclusion even when the isolated palm is fixed to a bench.
            ET.SubElement(contact,'exclude',body1=parent.get('name'),body2=child)
            if child == f'{side}_thumb_proximal_link' or child == f'{side}_thumb_proximal_Link':
                ET.SubElement(contact,'exclude',body1=f'{side}_base_link',body2=child)
            limit = source.find('limit')
            if source.get('type') != 'fixed' and limit is not None and float(limit.get('upper')) > float(limit.get('lower')):
                ET.SubElement(body,'joint',name=name,type='hinge',axis=source.find('axis').get('xyz'),
                    range=limit.get('lower')+' '+limit.get('upper'),damping='.04',armature='.0001',frictionloss='.002')
                created.append(name)
                mimic = source.find('mimic')
                if mimic is not None:
                    ET.SubElement(equalities,'joint',name=name+'_coupling',joint1=name,joint2=mimic.get('joint'),
                        polycoef=f"{mimic.get('offset','0')} {mimic.get('multiplier','1')} 0 0 0",solref='.008 1')
            link = links[child]
            inertial(link,body)
            mesh = link.find('collision/geometry/mesh')
            if mesh is not None:
                filename = Path(mesh.get('filename')).name
                mesh_name = 'brainco_'+child
                ET.SubElement(asset,'mesh',name=mesh_name,file='../../../../unitree_g1_description/meshes/'+filename)
                collision_origin = link.find('collision/origin')
                ET.SubElement(body,'geom',name=mesh_name+'_collision',type='mesh',mesh=mesh_name,
                    pos=collision_origin.get('xyz','0 0 0') if collision_origin is not None else '0 0 0',
                    quat=quaternion(collision_origin.get('rpy','0 0 0') if collision_origin is not None else '0 0 0'),
                    rgba='.75 .77 .8 1',friction='1 .005 .0001',condim='4',solref='.008 1')
            visit(body,child)
    visit(wrist,f'{side}_wrist_yaw_link')
    return created


def build_robot():
    root = ET.parse(SCENES/'g1_29dof.xml').getroot()
    original_joints = [j.get('name') for j in root.findall('worldbody//joint') if j.get('type') != 'free']
    key_values = {}
    for key in root.findall('keyframe/key'):
        values = key.get('qpos').split()
        key_values[key.get('name')] = (values[:7],dict(zip(original_joints,values[7:])))
    description = ET.parse(URDF).getroot()
    for side in ('left','right'):
        wrist = root.find(f".//body[@name='{side}_wrist_yaw_link']")
        for geom in list(wrist.findall('geom')):
            if geom.get('mesh','').endswith('rubber_hand'):
                wrist.remove(geom)
        wrist.remove(wrist.find('inertial'))
        inertial(description.find(f"link[@name='{side}_wrist_yaw_link']"),wrist)
        add_hand(root,wrist,side)
    names = [j.get('name') for j in root.findall('worldbody//joint') if j.get('type') != 'free']
    for key in root.findall('keyframe/key'):
        prefix,values = key_values[key.get('name')]
        key.set('qpos',' '.join(prefix+[values.get(name,'0') for name in names]))
    root.set('model','g1_29dof_brainco_physical')
    ET.indent(root)
    return root


def generate():
    path = SCENES/'g1_29dof_brainco_physical.xml'
    ET.ElementTree(build_robot()).write(path,encoding='unicode')
    scene = ET.parse(SCENES/'scene_29dof.xml')
    scene.getroot().find('include').set('file',path.name)
    scene.write(SCENES/'scene_29dof_brainco_hardware.xml',encoding='unicode')
    return path


if __name__=='__main__':
    print(generate())
