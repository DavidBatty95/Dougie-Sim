#!/usr/bin/env python3
"""Generate physical multi-table transport fixtures; layouts are evaluator data."""
import argparse
import json
import xml.etree.ElementTree as ET

from avoidance_worlds import ROOT, SCENES, generate as generate_navigation_worlds, scene
from world_appearance import sign_png

COLORS = {'red': '0.85 0.08 0.06 1', 'blue': '0.05 0.20 0.85 1', 'green': '0.06 0.65 0.18 1'}
TRANSPORT_WORLDS = {
    'transport_aisle': {'base':'medium_open','size':[12,10],'label':'CARRY LAB',
                        'stations': [((2.,0.),(4.,1.4))]},
    'transport_lab': {'base':'medium_open','size':[12,10],'label':'TRANSPORT LAB',
                      'stations': [((2.,0.),(-2.,1.4))]},
    'workshop': {'base': 'medium_open', 'size': [12,10], 'label': 'WORKSHOP',
                 'stations': [((-4.5,-2.2),(4.5,2.2)), ((-4.5,2.2),(4.5,-2.2))]},
    'office_delivery': {'base': 'large_office', 'size': [16,14], 'label': 'OFFICES',
                        'stations': [((-6.,-4.7),(6.,4.7)), ((-6.,0.),(6.,0.)), ((-6.,4.7),(6.,-4.7))]},
    'plaza_delivery': {'base': 'plaza', 'size': [26,24], 'label': 'DELIVERY',
                       'stations': [((-7.3,.9),(6.7,-2.4)), ((-7.3,-6.4),(7.3,4.5))]},
}
TABLE_HEIGHT = .95
TABLE_HALF_SIZE = (.55,.50,.04)
BOX_HALF_SIZE = .055
PAD_HALF_SIZE = .18
INITIAL_KEYFRAME = 'transport_unitree_rl_lab_stand'


def add_station(root, color, location, pickup):
    x,y = location
    identifier = ('pickup_' if pickup else 'delivery_') + color
    world = root.find('worldbody')
    station = ET.SubElement(world,'body',name='station_'+identifier,pos=f'{x} {y} 0')
    ET.SubElement(station,'geom',name=identifier+'_top',type='box',
                  pos=f'0 0 {TABLE_HEIGHT-TABLE_HALF_SIZE[2]}',size=' '.join(map(str,TABLE_HALF_SIZE)),
                  rgba='.46 .31 .18 1',contype='1',conaffinity='1',friction='1 .01 .001')
    leg_half_height=(TABLE_HEIGHT-2*TABLE_HALF_SIZE[2])/2
    for i,(lx,ly) in enumerate(((-.44,-.39),(-.44,.39),(.44,-.39),(.44,.39))):
        ET.SubElement(station,'geom',name=f'{identifier}_leg_{i}',type='box',
                      pos=f'{lx} {ly} {leg_half_height}',size=f'.035 .035 {leg_half_height}',
                      rgba='.12 .14 .16 1',contype='1',conaffinity='1')
    # Boxes and pads face the central aisle; their centers remain supported.
    inward = 1. if x < 0 else -1.
    offset = .47 if pickup else TABLE_HALF_SIZE[0]-PAD_HALF_SIZE-.025
    center = [x+inward*offset,y,TABLE_HEIGHT]
    if pickup:
        center[2] += BOX_HALF_SIZE+.002
        body=ET.SubElement(world,'body',name=f'manip_object_box_{color}',pos=' '.join(map(str,center)))
        ET.SubElement(body,'freejoint',name=f'box_{color}_freejoint')
        ET.SubElement(body,'geom',name=f'box_{color}_geom',type='box',
                      size=f'{BOX_HALF_SIZE} {BOX_HALF_SIZE} {BOX_HALF_SIZE}',mass='.15',
                      rgba=COLORS[color],friction='1 .01 .001',contype='1',conaffinity='1')
    else:
        # A filled colour square remains visible in the low-resolution RGB-D
        # camera; a one-pixel outline fragments at ordinary viewing distances.
        marker=ET.SubElement(world,'body',name=f'manip_surface_target_{color}',
                             pos=f'{center[0]} {center[1]} {TABLE_HEIGHT+.002}')
        ET.SubElement(marker,'geom',name=f'target_{color}_region',type='box',
                      size=f'{PAD_HALF_SIZE} {PAD_HALF_SIZE} .002',rgba=COLORS[color],contype='0',conaffinity='0')
        # Keep the scoring region as the body's sole geom: the object-state
        # protocol reports one box per body. Decoration belongs to a child.
        outline = ET.SubElement(marker,'body',name=f'target_{color}_outline')
        for i,(px,py,sx,sy) in enumerate(((-PAD_HALF_SIZE,0,.012,PAD_HALF_SIZE),
                                         (PAD_HALF_SIZE,0,.012,PAD_HALF_SIZE),
                                         (0,-PAD_HALF_SIZE,PAD_HALF_SIZE,.012),
                                         (0,PAD_HALF_SIZE,PAD_HALF_SIZE,.012))):
            ET.SubElement(outline,'geom',name=f'target_{color}_edge_{i}',type='box',
                          pos=f'{px} {py} .001',size=f'{sx} {sy} .002',rgba=COLORS[color],contype='0',conaffinity='0')
    return {'station':identifier,'table_center':[x,y], 'table_top_m':TABLE_HEIGHT,
            'object_or_pad_center':center}


def build_world(name):
    definition=TRANSPORT_WORLDS[name]
    if definition['base']=='plaza':
        root=ET.parse(SCENES/'scene_29dof_pedestrians.xml').getroot()
        ET.SubElement(root,'include',file='natural_crowd_assets.xml')
        ET.SubElement(root,'include',file='natural_crowd_bodies.xml')
        custom=ET.SubElement(root,'custom')
        ET.SubElement(custom,'numeric',name='avoidance_world_half_extents',data='8 7')
    else:
        root=ET.fromstring(scene(definition['base']))
        root.find("asset/texture[@name='world_sign']").set('file',f'transport_sign_{name}.png')
    root.set('model','G1 transport '+name)
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root,'custom')
    ET.SubElement(custom,'numeric',name='transport_hand_capture',data='.12 .01 0 .09')
    root.find('visual/global').set('elevation','-48')
    equalities=ET.SubElement(root,'equality')
    keys=ET.SubElement(root,'keyframe')
    robot=ET.parse(SCENES/'g1_29dof.xml').getroot()
    qpos=robot.find("keyframe/key[@name='unitree_rl_lab_stand']").get('qpos').split()
    pairs=[]
    layout=[]
    for color,(pickup,delivery) in zip(COLORS,definition['stations']):
        source=add_station(root,color,pickup,True)
        destination=add_station(root,color,delivery,False)
        qpos += [str(v) for v in source['object_or_pad_center']] + ['1','0','0','0']
        for side in ('left','right'):
            ET.SubElement(equalities,'weld',name=f'grasp_{side}_box_{color}',
                          body1=f'{side}_wrist_yaw_link',body2=f'manip_object_box_{color}',active='false')
        pairs.append({'object_id':f'box_{color}','destination_id':f'target_{color}','color':color})
        layout.append({'color':color,'pickup':source,'delivery':destination})
    ET.SubElement(keys,'key',name=INITIAL_KEYFRAME,qpos=' '.join(qpos))
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')+'\n', pairs, layout


def generate():
    generate_navigation_worlds()
    manifests={}
    for name,definition in TRANSPORT_WORLDS.items():
        xml,pairs,layout=build_world(name)
        (SCENES/f'scene_29dof_transport_{name}.xml').write_text(xml)
        if definition['base']!='plaza':
            (SCENES/f'transport_sign_{name}.png').write_bytes(sign_png(definition['label']))
        # Pair files contain no locations; layout files are simulator/scorer-only.
        (SCENES/f'transport_pairs_{name}.json').write_text(json.dumps({'pairs':pairs},indent=2)+'\n')
        manifests[name]={'initial_keyframe':INITIAL_KEYFRAME,'size_m':definition['size'],
                         'evaluation_layout':layout,'grasp_model':'capture_radius_weld', 'base_pin_available':False}
    (SCENES/'transport_evaluation_layouts.json').write_text(json.dumps(manifests,indent=2)+'\n')
    print(f'Generated {len(manifests)} physical transport fixtures')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    generate()
