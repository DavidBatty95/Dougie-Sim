"""Simulator-only two-connector mezzanine fixture; no coordinates enter autonomy."""
import math

from .reactor import box


def add_mezzanine(body, angle_deg=10.):
    """Fixed-height gallery; optional gentler access preserves the original 10° case."""
    if not math.isfinite(angle_deg) or not 6. <= angle_deg <= 10.:
        raise ValueError('mezzanine access angle must be between 6 and 10 degrees')
    angle = math.radians(angle_deg)
    height = 4.*math.tan(math.radians(10.))
    run = height/math.tan(angle)
    landing, east = 1.+run, 7.+run
    for name, pos, size in (
        ('north_wall', ((east-3.)/2., 6.2, 1.2), ((east+3.)/2., .15, 1.2)),
        ('south_wall', ((east-3.)/2., -2.2, 1.2), ((east+3.)/2., .15, 1.2)),
        ('east_wall', (east, 2., 1.2), (.15, 4.2, 1.2)),
        ('west_wall', (-3., 2., 1.2), (.15, 4.2, 1.2)),
    ):
        box(body, name, pos, size)
    box(body, 'mezzanine_slab', (landing+2., 2., height-.08), (2., 3.2, .08),
        '.35 .43 .5 1', user='3.5')
    for i, y in enumerate((0., 4.)):
        thickness = .04
        box(body, f'access_ramp_{i}', (1.+run/2.+thickness*math.sin(angle)/2, y,
            height/2-thickness*math.cos(angle)/2), (run/2./math.cos(angle), 1.2, thickness/2),
            '.5 .57 .6 1', quat=f'{math.cos(angle/2)} 0 {-math.sin(angle/2)} 0')
        # Thin continuous rails leave the centre-line path and sensor view open.
        for side in (-1, 1):
            box(body, f'ramp_rail_{i}_{side}', (1.+run/2., y+side*1.18, height/2+.65),
                (run/2./math.cos(angle), .025, .04), '.95 .7 .1 1',
                quat=f'{math.cos(angle/2)} 0 {-math.sin(angle/2)} 0')
    for name, pos, size in (
        ('gallery_east_rail', (landing+4., 2., height+.65), (.025, 3.2, .04)),
        ('gallery_north_rail', (landing+2., 5.18, height+.65), (2., .025, .04)),
        ('gallery_south_rail', (landing+2., -1.18, height+.65), (2., .025, .04)),
        ('gallery_west_rail', (landing, 2., height+.65), (.025, .8, .04)),
    ):
        box(body, name, pos, size, '.95 .7 .1 1')
    box(body, 'lower_shield', (-1., 3., .65), (.2, .65, .65), user='2.0')
    return height


def source_description(angle_deg=10.):
    if not math.isfinite(angle_deg) or not 6. <= angle_deg <= 10.:
        raise ValueError('mezzanine access angle must be between 6 and 10 degrees')
    shift = 4.*math.tan(math.radians(10.))/math.tan(math.radians(angle_deg))-4.
    return dict(background_usv_h=.2, sources=[
        dict(id='gallery_hotspot', kind='hotspot', position=[7.5+shift, 0., 1.],
             strength_usv_h_m2=35., core_radius_m=.1),
        dict(id='lower_leak', kind='leak', position=[-1.8, 4.4, .2],
             strength_usv_h_m2=10., core_radius_m=.1),
    ], shields=[])
