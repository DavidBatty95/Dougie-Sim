"""Procedural reactor-themed facility, shielding, and a small stair test course.

Scene geometry belongs to the simulator. Exploration consumes sensor returns,
not these coordinates. This is an illustrative facility, not an actual plant.
"""
import xml.etree.ElementTree as ET


def box(body, name, pos, size, color='.5 .53 .55 1', **attrs):
    return ET.SubElement(body, 'geom', name=name, type='box', pos=' '.join(map(str, pos)),
                         size=' '.join(map(str, size)), rgba=color, **attrs)


def stairs(body, x=1., y=0., rise=.06, count=6, tread=.4, width=1.6):
    for i in range(count):
        height = (i+1)*rise
        box(body, f'step_up_{i}', (x+(i+.5)*tread, y, height/2),
            (tread/2, width/2, height/2), '.55 .57 .58 1', friction='.9 .02 .01')
    top = x+count*tread
    height = count*rise
    box(body, 'inspection_deck', (top+.8, y, height/2), (.8, width/2, height/2),
        '.3 .37 .4 1', friction='.9 .02 .01')
    for i in range(count):
        h = (count-i)*rise
        box(body, f'step_down_{i}', (top+1.6+(i+.5)*tread, y, h/2),
            (tread/2, width/2, h/2), '.55 .57 .58 1', friction='.9 .02 .01')


def add_reactor(body):
    # Separate service rooms, hot-cell shielding, equipment and service aisles.
    for name, pos, size in (
        ('north_wall', (0, 6, 1.5), (10, .2, 1.5)),
        ('south_wall', (0, -6, 1.5), (10, .2, 1.5)),
        ('east_wall', (10, 0, 1.5), (.2, 6, 1.5)),
        ('west_wall', (-10, 0, 1.5), (.2, 6, 1.5)),
        ('shield_west', (3, 3.8, 1.5), (.3, 2.2, 1.5)),
        ('shield_south', (5.6, 1.6, 1.5), (1.9, .3, 1.5)),
        ('service_partition', (-3.5, 3.8, 1.4), (.15, 2.2, 1.4)),
        ('control_panel_a', (-7.5, -4.8, .9), (1.5, .4, .9)),
        ('control_panel_b', (-5., -4.8, .9), (.7, .4, .9)),
        ('pump_base', (-5., 2.5, .3), (.9, .7, .3)),
    ):
        box(body, name, pos, size)
    ET.SubElement(body, 'geom', name='reactor_vessel', type='cylinder', pos='5.8 3.8 1.5',
                  size='1.1 1.5', rgba='.25 .35 .4 1')
    for i, y in enumerate((2.8, 4.8)):
        ET.SubElement(body, 'geom', name=f'coolant_pipe_{i}', type='capsule',
                      fromto=f'-8 {y} 2.8 8 {y} 2.8', size='.15', rgba='.1 .35 .65 1')
    for i, x in enumerate((-6., -5.2, -4.4)):
        ET.SubElement(body, 'geom', name=f'waste_drum_{i}', type='cylinder', pos=f'{x} 4.8 .45',
                      size='.28 .45', rgba='.85 .65 .05 1')
    # Painted aisle guides / hazard bands are visual-only.
    for i in range(24):
        box(body, f'aisle_mark_{i}', (-8+i*.7, -.9, .003), (.22, .035, .002),
            '.95 .72 .08 1', contype='0', conaffinity='0', group='2')
    stairs(body, x=1., y=-3., width=2.)
    # Low irregular service-floor plates, away from the initial staging area.
    for i, (x, y, h) in enumerate(((-2., -2., .025), (-2.6, -2.3, .04), (-3.2, -2., .02))):
        box(body, f'floor_plate_{i}', (x, y, h/2), (.28, .3, h/2), '.35 .36 .38 1')
    add_service_detail(body)


def add_service_detail(body):
    """Illustrative industrial fittings; primary aisles and spawn stay clear."""
    visual = dict(contype='0', conaffinity='0', group='2')

    def pipe(name, start, end, radius=.07, color='.18 .42 .58 1', decorative=False):
        return ET.SubElement(body, 'geom', name=name, type='capsule',
            fromto=' '.join(map(str, (*start, *end))), size=str(radius), rgba=color,
            **(visual if decorative else {}))

    # Vessel cap, flanges and instrumentation: silhouette remains readable overhead.
    ET.SubElement(body, 'geom', name='vessel_domed_head', type='ellipsoid',
        pos='5.8 3.8 3', size='1.1 1.1 .38', rgba='.38 .48 .53 1')
    for i, z in enumerate((.22, 2.75)):
        ET.SubElement(body, 'geom', name=f'vessel_flange_{i}', type='cylinder',
            pos=f'5.8 3.8 {z}', size='1.18 .08', rgba='.6 .66 .69 1')
    pipe('vessel_top_nozzle', (5.8, 3.8, 3.2), (5.8, 3.8, 3.7), .16)
    pipe('vessel_pressure_branch', (5.8, 3.8, 3.6), (6.5, 3.8, 3.6), .09)
    ET.SubElement(body, 'geom', name='vessel_gauge', type='cylinder',
        pos='6.5 3.8 3.66', size='.16 .04', rgba='.95 .95 .85 1', **visual)

    # Twin pumps on the existing plinth, with motors, coupling guards and feed lines.
    for i, y in enumerate((2.15, 2.85)):
        pipe(f'pump_motor_{i}', (-5.65, y, .9), (-5.05, y, .9), .22, '.18 .35 .38 1')
        pipe(f'pump_coupling_{i}', (-5.05, y, .9), (-4.75, y, .9), .12, '.9 .65 .1 1')
        ET.SubElement(body, 'geom', name=f'pump_housing_{i}', type='sphere',
            pos=f'-4.55 {y} .9', size='.26', rgba='.24 .45 .49 1')
        pipe(f'pump_feed_{i}', (-4.55, y, 1.1), (-4.55, y, 2.8), .085)
        for j in range(5):
            box(body, f'motor_fin_{i}_{j}', (-5.6+j*.12, y, 1.12), (.025, .18, .035), '.5 .59 .6 1', **visual)

    # Pipe racks and valve handwheels along the perimeter, outside walking lanes.
    for i, x in enumerate((-8.6, -1.5, 8.7)):
        for j, y in enumerate((2.8, 4.8)):
            box(body, f'pipe_rack_post_{i}_{j}', (x, y, 1.35), (.065, .065, 1.35), '.42 .48 .51 1')
        box(body, f'pipe_rack_beam_{i}', (x, 3.8, 2.65), (.09, 1.1, .08), '.42 .48 .51 1')
    for i, y in enumerate((2.8, 4.8)):
        pipe(f'coolant_downcomer_{i}', (-8., y, 2.8), (-8., y, .8), .13)
        pipe(f'valve_stem_{i}', (-8., y, 1.35), (-7.65, y, 1.35), .04)
        # Handwheel spokes are decorative and do not add tiny collision hazards.
        pipe(f'valve_spoke_y_{i}', (-7.65, y-.19, 1.35), (-7.65, y+.19, 1.35), .025, '.85 .15 .08 1', True)
        pipe(f'valve_spoke_z_{i}', (-7.65, y, 1.16), (-7.65, y, 1.54), .025, '.85 .15 .08 1', True)

    # Control cabinet fronts: screen, status lamps, handles and ventilation slots.
    for i, x in enumerate((-8.2, -7.3, -6.4, -5.)):
        box(body, f'panel_screen_frame_{i}', (x, -4.385, 1.35), (.27, .025, .23), '.08 .12 .15 1', **visual)
        box(body, f'panel_screen_{i}', (x, -4.35, 1.35), (.23, .008, .19), '.1 .65 .65 1', **visual)
        for j, color in enumerate(('.15 .9 .25 1', '.95 .6 .05 1', '.9 .1 .08 1')):
            box(body, f'panel_lamp_{i}_{j}', (x-.18+j*.18, -4.35, .96), (.035, .012, .035), color, **visual)
        for j in range(4):
            box(body, f'panel_vent_{i}_{j}', (x, -4.35, .35+j*.075), (.22, .008, .013), '.12 .17 .19 1', **visual)
    for side in (-1, 1):
        box(body, f'cable_tray_side_{side}', (-4.5, -4.2+side*.2, 2.55), (4.3, .035, .1), '.38 .43 .47 1')
    for i in range(18):
        box(body, f'cable_tray_rung_{i}', (-8.6+i*.48, -4.2, 2.5), (.025, .2, .025), '.4 .47 .5 1', **visual)

    # Spare-parts bay and a short shielding return form an additional service alcove.
    for i, x in enumerate((-8.8, -7.4)):
        box(body, f'spares_crate_{i}', (x, -.1, .45), (.5, .55, .45), '.38 .3 .2 1')
        for z in (.18, .72):
            box(body, f'crate_band_{i}_{z}', (x, -.66, z), (.5, .012, .025), '.15 .18 .2 1', **visual)
    box(body, 'service_alcove_shield', (-8.3, 1.25, 1.1), (1.3, .12, 1.1), '.58 .61 .58 1', user='1.2')
    # Visual floor markings never create raised steps or LiDAR support returns.
    for y in (-1.15, .75):
        box(body, f'walkway_edge_{y}', (-.2, y, .004), (8.8, .025, .002), '.88 .88 .73 1', **visual)
    for i in range(10):
        box(body, f'pump_hazard_stripe_{i}', (-6.+i*.2, 1.64, .004), (.07, .10, .002),
            '.95 .72 .08 1' if i % 2 else '.08 .1 .12 1', **visual)
    for i, x in enumerate((-6., 0., 6.)):
        box(body, f'overhead_light_{i}', (x, -.4, 3.35), (.65, .14, .045), '.9 .95 1 1', **visual)
        ET.SubElement(body, 'light', name=f'work_light_{i}', pos=f'{x} -.4 3.25', dir='0 0 -1',
            diffuse='.35 .38 .4', specular='.1 .1 .1', castshadow='false')


def radiation_description(profile='baseline', source_config=None):
    from .source_config import load_source_profile
    description = load_source_profile(profile, source_config)
    description['shields'] = [
        dict(lower=[2.7, 1.6, 0.], upper=[3.3, 6., 3.], attenuation_per_m=3.5),
        dict(lower=[3.7, 1.3, 0.], upper=[7.5, 1.9, 3.], attenuation_per_m=3.5),
        dict(lower=[-3.65, 1.6, 0.], upper=[-3.35, 6., 2.8], attenuation_per_m=2.),
        dict(lower=[-9.6, 1.13, 0.], upper=[-7., 1.37, 2.2], attenuation_per_m=1.2),
    ]
    return description
