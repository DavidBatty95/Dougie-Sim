"""Two-storey service building. Geometry is simulator/evaluator information only."""
import math
import xml.etree.ElementTree as ET

import numpy as np

from .reactor import box

FLOOR_HEIGHT = 2.4


def slab(body, name, x0, x1, y0, y1, height, thickness=.18):
    return box(body, name, ((x0+x1)/2, (y0+y1)/2, height-thickness/2),
               ((x1-x0)/2, (y1-y0)/2, thickness/2), '.38 .46 .51 1', user='3.5')


def ramp(body, name, x0, x1, y, z0, z1, width=2.4):
    angle = math.atan2(z1-z0, x1-x0)
    length = math.hypot(x1-x0, z1-z0)
    thickness = .06
    quat = f'{math.cos(angle/2)} 0 {-math.sin(angle/2)} 0'
    box(body, name, ((x0+x1)/2+thickness*math.sin(angle)/2, y,
        (z0+z1)/2-thickness*math.cos(angle)/2), (length/2, width/2, thickness/2),
        '.55 .59 .61 1', quat=quat)
    for side in (-1, 1):
        for height in (.5, 1.):
            box(body, f'{name}_rail_{side}_{height}', ((x0+x1)/2, y+side*width/2, (z0+z1)/2+height),
                (length/2, .035, .035), '.94 .69 .12 1', quat=quat)
        for i, fraction in enumerate(np.linspace(0., 1., 5)):
            x, z = x0+fraction*(x1-x0), z0+fraction*(z1-z0)
            box(body, f'{name}_post_{side}_{i}', (x, y+side*width/2, z+.5), (.035, .035, .5), '.7 .72 .73 1')


def add_facility(body, layout_seed=0):
    """Full-height overlapping rooms, switchback service ramps, and a staircase.

    The 15 cm staircase is an unqualified challenge, not a hidden ramp. The
    baseline slope mapper must reject it and discover the ramp alternative.
    """
    rng = np.random.default_rng(layout_seed)
    for name, pos, size in (
        ('outer_north', (6.5, 12., 2.7), (9.5, .15, 2.7)),
        ('outer_south', (6.5, -4., 2.7), (9.5, .15, 2.7)),
        ('outer_west', (-3., 4., 2.7), (.15, 8., 2.7)),
        ('outer_east', (16., 4., 2.7), (.15, 8., 2.7)),
    ):
        box(body, name, pos, size, '.68 .72 .73 1', user='1.5')

    # Slabs leave a genuine stairwell opening at x=9.7..14.5, y=1..3.
    slab(body, 'upper_floor_main', -2.8, 9.7, -3.8, 5., FLOOR_HEIGHT)
    slab(body, 'upper_floor_south', 9.7, 15.8, -3.8, 1., FLOOR_HEIGHT)
    slab(body, 'upper_floor_north', 9.7, 15.8, 3., 5., FLOOR_HEIGHT)
    slab(body, 'upper_floor_east', 14.5, 15.8, 1., 3., FLOOR_HEIGHT)
    slab(body, 'upper_floor_access', -2.8, 1., 5., 10.8, FLOOR_HEIGHT)
    # Two 8 m flights at 8.53 degrees, with a wide intermediate turning landing.
    ramp(body, 'service_ramp_lower', 1., 9., 6.4, 0., 1.2)
    slab(body, 'intermediate_landing', 9., 12., 5.2, 10.8, 1.2)
    ramp(body, 'service_ramp_upper', 1., 9., 9.6, 2.4, 1.2)
    for i in range(16):
        height = (i+1)*.15
        x = 14.5-(i+.5)*.3
        box(body, f'industrial_step_{i:02d}', (x, 2., height/2), (.15, 1., height/2), '.58 .6 .61 1')

    # Segmented room partitions leave 1.8 m door openings. No closed-door oracle.
    door = 1.+float(rng.uniform(-.25, .25))
    for level, z in enumerate((0., FLOOR_HEIGHT)):
        for side, (y0, y1) in enumerate(((-3.8, door-.9), (door+.9, 4.8))):
            box(body, f'room_partition_{level}_{side}', (4., (y0+y1)/2, z+1.05),
                (.1, (y1-y0)/2, 1.05), '.6 .68 .71 1', user='1.8')
        box(body, f'door_header_{level}', (4., door, z+2.14), (.1, .9, .14), '.4 .48 .53 1')
        for i, x in enumerate((1.5, 6., 8.)):
            box(body, f'equipment_{level}_{i}', (x, -2.9+float(rng.uniform(-.12, .12)), z+.65),
                (.6, .45, .65), '.2 .38 .48 1')
        # Broad posts support the slab but preserve a clear central corridor.
        for i, x in enumerate((-.8, 7.5, 14.8)):
            box(body, f'column_{level}_{i}', (x, 4.5, z+1.1), (.12, .12, 1.1), '.5 .54 .57 1')
        for i, x in enumerate((5.5, 6.3)):
            ET.SubElement(body, 'geom', name=f'drum_{level}_{i}', type='cylinder',
                          pos=f'{x} 3.7 {z+.4}', size='.25 .4', rgba='.85 .63 .1 1')

    # Rails around exposed upper-floor/stairwell edges, with connector entrances open.
    for name, pos, size in (
        ('upper_north_rail', (5.35, 5., 3.4), (4.35, .035, .035)),
        ('stair_south_rail', (12.1, 1., 3.4), (2.4, .035, .035)),
        ('stair_north_rail', (12.1, 3., 3.4), (2.4, .035, .035)),
        ('access_west_rail', (-2.8, 7.9, 3.4), (.035, 2.9, .035)),
        ('access_north_rail', (-.9, 10.8, 3.4), (1.9, .035, .035)),
        ('middle_east_rail', (12., 8., 2.2), (.035, 2.8, .035)),
    ):
        box(body, name, pos, size, '.94 .69 .12 1')


def source_description():
    return dict(background_usv_h=.2, sources=[
        dict(id='ground_pump_leak', kind='leak', position=[6., -2.4, .45],
             strength_usv_h_m2=20., core_radius_m=.1),
        dict(id='upper_drum_hotspot', kind='hotspot', position=[6.3, 3.7, 2.85],
             strength_usv_h_m2=30., core_radius_m=.1),
        dict(id='stairwell_leak', kind='leak', position=[13., 2., 1.],
             strength_usv_h_m2=12., core_radius_m=.1),
    ], shields=[])
