import math
import xml.etree.ElementTree as ET
from collections import deque

from avoidance_worlds import WORLDS, scene
from run_avoidance_matrix import command_for


def test_worlds_have_connected_robot_clear_space():
    for name, world in WORLDS.items():
        root = ET.fromstring(scene(name))
        x, y = (n / 2 for n in world['size'])
        assert root.find('custom/numeric').get('data') == f'{x} {y}'
        # Conservative robot footprint; all accessible cells must connect to spawn.
        free = set()
        for ix in range(-int(x*5), int(x*5)+1):
            for iy in range(-int(y*5), int(y*5)+1):
                px, py = ix / 5, iy / 5
                if abs(px) >= x-.55 or abs(py) >= y-.55:
                    continue
                if any(abs(px-bx) <= sx+.55 and abs(py-by) <= sy+.55 for bx, by, sx, sy in world['boxes']):
                    continue
                free.add((ix, iy))
        assert (0, 0) in free
        queue, seen = deque([(0, 0)]), {(0, 0)}
        while queue:
            ix, iy = queue.popleft()
            for next_cell in ((ix+1, iy), (ix-1, iy), (ix, iy+1), (ix, iy-1)):
                if next_cell in free and next_cell not in seen:
                    seen.add(next_cell)
                    queue.append(next_cell)
        assert seen == free, name
        assert len(root.findall('worldbody/geom')) > 30 + len(world['boxes'])
        colliders = [g for g in root.findall('worldbody/geom') if g.get('name', '').startswith('world_box_')]
        assert len(colliders) == 4 + len(world['boxes'])


def test_matrix_keeps_onboard_contract(tmp_path):
    command = command_for(tmp_path, 'medium_doorway', 44, 4, 3, '/workspace/benchmark.yaml')
    assert command[command.index('--scene')+1] == 'scene_29dof_avoidance_medium_doorway.xml'
    assert 'onboard_only:=true' in command
    assert 'nav2_static_scene_map:=false' in command
    assert 'tracker_use_sensor_input:=true' in command
    assert '--visual-session' not in command
