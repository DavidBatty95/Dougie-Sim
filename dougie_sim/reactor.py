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


def radiation_description(profile='baseline', source_config=None):
    from .source_config import load_source_profile
    description = load_source_profile(profile, source_config)
    description['shields'] = [
        dict(lower=[2.7, 1.6, 0.], upper=[3.3, 6., 3.], attenuation_per_m=3.5),
        dict(lower=[3.7, 1.3, 0.], upper=[7.5, 1.9, 3.], attenuation_per_m=3.5),
        dict(lower=[-3.65, 1.6, 0.], upper=[-3.35, 6., 2.8], attenuation_per_m=2.),
    ]
    return description
