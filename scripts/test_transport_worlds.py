import xml.etree.ElementTree as ET
import math

from transport_worlds import TRANSPORT_WORLDS, build_world, camera_robot_xml, INITIAL_KEYFRAME, TABLE_HALF_SIZE, BOX_HALF_SIZE, PAD_HALF_SIZE


def test_transport_fixtures_have_supported_pairs_and_no_base_pin():
    for name, definition in TRANSPORT_WORLDS.items():
        text,pairs,layout = build_world(name)
        root = ET.fromstring(text)
        assert len(pairs) == len(definition['stations'])
        assert len({p['object_id'] for p in pairs}) == len(pairs)
        assert len({p['destination_id'] for p in pairs}) == len(pairs)
        assert all(set(p)=={'object_id','destination_id','color'} for p in pairs)
        assert root.find("equality/weld[@name='manipulation_base_pin']") is None
        assert len(root.find('equality').findall('weld')) == 2*len(pairs)
        for pair in pairs:
            marker = root.find(f"worldbody/body[@name='manip_surface_{pair['destination_id']}']")
            assert len(marker.findall('geom')) == 1  # Object-state protocol contract.
            assert marker.find('geom').get('rgba') != '0 0 0 0'
        assert len(root.find(f"keyframe/key[@name='{INITIAL_KEYFRAME}']").get('qpos').split()) == 36+7*len(pairs)
        for entry in layout:
            for station in ('pickup','delivery'):
                record=entry[station]
                dx=abs(record['object_or_pad_center'][0]-record['table_center'][0])
                extent=BOX_HALF_SIZE if station=='pickup' else PAD_HALF_SIZE+.012
                assert dx+extent < TABLE_HALF_SIZE[0]
                top=root.find(f"worldbody/body[@name='station_{record['station']}']/geom")
                assert top.get('contype') == top.get('conaffinity') == '1'


def test_dual_demo_contains_exactly_two_coloured_cubes_and_four_physical_tables():
    text,pairs,layout = build_world('transport_dual')
    root = ET.fromstring(text)
    assert [pair['color'] for pair in pairs] == ['red','blue']
    assert len([body for body in root.findall('worldbody/body') if body.get('name','').startswith('station_')]) == 4
    assert len(root.findall('worldbody/body/freejoint')) == 2
    assert len(layout) == 2
    assert all('position' not in pair and 'table_center' not in pair for pair in pairs)


def test_tabletop_camera_pitch_matches_declared_optical_axis():
    pitch = TRANSPORT_WORLDS['transport_dual']['camera_pitch_deg']
    root = ET.fromstring(camera_robot_xml(pitch))
    w,x,y,z = map(float,root.find(".//site[@name='d435i_color_site']").get('quat').split())
    forward = (2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y))
    assert abs(forward[0]-math.cos(math.radians(pitch))) < 1e-12
    assert abs(forward[1]) < 1e-12
    assert abs(forward[2]+math.sin(math.radians(pitch))) < 1e-12
