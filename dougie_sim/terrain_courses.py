"""Parameterized qualification fixtures, NOT planner inputs.

The evaluator knows course geometry. Velocity guidance receives a centre-line
goal; observed guidance receives range returns and the allowed test corridor.
Neither guidance mode may read surface_height() or fixture contact identities.
"""
from dataclasses import asdict, dataclass, replace
import math
import xml.etree.ElementTree as ET

import numpy as np

from .reactor import box, stairs
from .simulation import scene_xml


@dataclass(frozen=True)
class CourseSpec:
    name: str
    kind: str = 'stairs'
    direction: str = 'up'
    rise: float = .06
    tread: float = .4
    steps: int = 6
    width: float = 1.6
    length: float = 4.
    amplitude: float = .04
    tile_size: float = .25
    slope_deg: float = 10.
    friction: float = .4
    payload_kg: float = 0.
    speed: float = .2
    observation_delay_ms: int = 0
    timestep: float = .005
    guidance: str = 'velocity'
    timeout_s: float = 180.
    pause_on_course: bool = False

    def __post_init__(self):
        numbers = [self.rise, self.tread, self.width, self.length, self.amplitude,
                   self.slope_deg, self.friction, self.payload_kg, self.speed, self.timestep, self.timeout_s]
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError('nonfinite course specification')
        if (self.kind not in ('stairs', 'legacy', 'tiles', 'ridges', 'ramp', 'flat') or
                self.direction not in ('up', 'down', 'both') or self.guidance not in ('velocity', 'observed') or
                self.rise <= 0 or self.tread <= 0 or self.width < .65 or self.steps < 1 or
                self.friction <= 0 or self.payload_kg < 0 or self.speed <= 0 or self.length <= 0 or
                self.tile_size <= 0 or self.amplitude < 0 or not 0 < self.slope_deg < 40 or
                self.observation_delay_ms < 0 or self.observation_delay_ms % 20 or
                self.timestep <= 0 or self.timeout_s <= 0):
            raise ValueError('invalid course specification')

    @property
    def start_x(self):
        return 1.

    @property
    def top(self):
        return self.steps*self.rise if self.kind in ('stairs', 'legacy') else self.length*math.tan(math.radians(self.slope_deg)) if self.kind == 'ramp' else 0.

    @property
    def end_x(self):
        if self.kind in ('stairs', 'legacy'):
            return 1.+(2*self.steps*self.tread+1.6 if self.direction == 'both' or self.kind == 'legacy' else self.steps*self.tread)
        return 1.+self.length

    @property
    def finish_x(self):
        return self.end_x+.7

    @property
    def initial_height(self):
        return self.top if self.direction == 'down' and self.kind in ('stairs', 'ramp') else 0.

    @property
    def final_height(self):
        return self.top if self.direction == 'up' and self.kind in ('stairs', 'ramp') else 0.


class Course:
    def __init__(self, spec, seed):
        self.spec, self.seed = spec, seed
        self.tiles = []
        root = ET.fromstring(scene_xml('flat'))
        body = root.find('worldbody')
        root.find('option').set('timestep', str(spec.timestep))
        for geom in root.iter('geom'):
            if geom.get('name') in ('FR', 'FL', 'RR', 'RL', 'floor'):
                geom.set('friction', f'{spec.friction} .02 .01')
        self.required_top = None
        if spec.kind == 'legacy':
            stairs(body, rise=spec.rise, tread=spec.tread, count=spec.steps, width=spec.width)
            self.required_top = (1.+spec.steps*spec.tread, 1.+spec.steps*spec.tread+1.6, spec.top)
        elif spec.kind == 'stairs':
            if spec.direction == 'down':
                self._box(body, 'start_platform', -2., 1., spec.top)
                for i in range(spec.steps-1):
                    self._box(body, f'down_{i}', 1.+i*spec.tread, 1.+(i+1)*spec.tread, (spec.steps-i-1)*spec.rise)
            else:
                for i in range(spec.steps):
                    self._box(body, f'up_{i}', 1.+i*spec.tread, 1.+(i+1)*spec.tread, (i+1)*spec.rise)
                top_start = 1.+spec.steps*spec.tread
                self._box(body, 'top_platform', top_start, spec.finish_x+2., spec.top)
                self.required_top = (top_start, spec.finish_x+2., spec.top)
                if spec.direction == 'both':
                    # Replace the long upper landing with a finite landing.
                    body.remove(body.find("geom[@name='qual_top_platform']"))
                    self._box(body, 'top_platform', top_start, top_start+1.6, spec.top)
                    self.required_top = (top_start, top_start+1.6, spec.top)
                    for i in range(spec.steps-1):
                        self._box(body, f'down_{i}', top_start+1.6+i*spec.tread,
                                  top_start+1.6+(i+1)*spec.tread, (spec.steps-i-1)*spec.rise)
        elif spec.kind == 'tiles':
            rng = np.random.default_rng(seed)
            nx, ny = math.ceil(spec.length/spec.tile_size), math.ceil(spec.width/spec.tile_size)
            dx, dy = spec.length/nx, spec.width/ny
            for i in range(nx):
                for j in range(ny):
                    height = float(rng.uniform(.002, max(.0021, spec.amplitude)))
                    x0, y0 = 1.+i*dx, -spec.width/2+j*dy
                    box(body, f'qual_tile_{i}_{j}', (x0+dx/2, y0+dy/2, height/2),
                        (dx/2, dy/2, height/2), '.4 .45 .5 1')
                    self.tiles.append((x0, x0+dx, y0, y0+dy, height))
        elif spec.kind == 'ridges':
            for i, x in enumerate(np.arange(1., spec.end_x, .6)):
                self._box(body, f'ridge_{i}', float(x), float(x)+.2, spec.amplitude)
        elif spec.kind == 'ramp':
            theta = math.radians(spec.slope_deg)*(1 if spec.direction == 'up' else -1)
            thickness = .08
            box(body, 'qual_ramp', (1.+spec.length/2+thickness*math.sin(theta), 0., spec.top/2-thickness*math.cos(theta)),
                (spec.length/(2*math.cos(theta)), spec.width/2, thickness), '.5 .55 .6 1', euler=f'0 {-theta} 0')
            if spec.direction == 'down':
                self._box(body, 'start_platform', -2., 1., spec.top)
            else:
                self._box(body, 'top_platform', spec.end_x, spec.finish_x+2., spec.top)
                self.required_top = (spec.end_x, spec.finish_x+2., spec.top)
        # Apply the declared traction to generated terrain as well as the floor
        # and feet. New boxes otherwise retain MuJoCo's default friction of 1.
        # The pinned feet have higher contact priority, so this parameter repair
        # does not by itself imply that historical foot traction was incorrect.
        for geom in body.findall('geom'):
            geom.set('friction', f'{spec.friction} .02 .01')
        if spec.payload_kg:
            payload = ET.SubElement(body.find("body[@name='base_link']"), 'body', name='qualification_payload', pos='0 0 .13')
            box(payload, 'qualification_payload_geom', (0, 0, 0), (.12, .09, .04), '.8 .6 .1 1', mass=str(spec.payload_kg))
        self.xml = ET.tostring(root, encoding='unicode')

    def _box(self, body, name, start, end, height):
        if height > 0:
            box(body, 'qual_'+name, ((start+end)/2, 0., height/2), ((end-start)/2, self.spec.width/2, height/2), '.55 .57 .6 1')

    def surface_height(self, x, y):
        """Evaluator-only reference. Never called by a controller/planner."""
        s = self.spec
        if abs(y) > s.width/2:
            return 0.
        if s.kind == 'tiles':
            return max((h for x0, x1, y0, y1, h in self.tiles if x0 <= x <= x1 and y0 <= y <= y1), default=0.)
        if s.kind == 'ridges':
            return s.amplitude if 1. <= x < s.end_x and (x-1.) % .6 <= .2 else 0.
        if s.kind == 'ramp':
            fraction = np.clip((x-1.)/s.length, 0., 1.)
            return float(s.top*(fraction if s.direction == 'up' else 1.-fraction))
        if s.kind in ('stairs', 'legacy'):
            if s.direction == 'down':
                return s.top if x < 1. else max(0., s.top-(math.floor((x-1.)/s.tread)+1)*s.rise)
            if x < 1.:
                return 0.
            up_end = 1.+s.steps*s.tread
            if x < up_end:
                return (math.floor((x-1.)/s.tread)+1)*s.rise
            if s.direction == 'up' and s.kind != 'legacy':
                return s.top
            if x < up_end+1.6:
                return s.top
            step = math.floor((x-up_end-1.6)/s.tread)
            return max(0., s.top-(step+(0 if s.kind == 'legacy' else 1))*s.rise)
        return 0.


def catalogue():
    cases = [CourseSpec('flat', kind='flat'), CourseSpec('legacy_06', kind='legacy', direction='both')]
    for rise, tread in ((.03, .4), (.06, .4), (.075, .4), (.10, .35), (.12, .30), (.15, .30), (.18, .28), (.20, .25)):
        for direction in ('up', 'down'):
            cases.append(CourseSpec(f'stairs_{direction}_{round(rise*1000):03d}', rise=rise, tread=tread, direction=direction))
    base = CourseSpec('stairs_up_060')
    for tag, changes in (
        ('narrow', dict(width=.8)), ('low_friction', dict(friction=.2)),
        ('high_friction', dict(friction=.8)), ('payload_05', dict(payload_kg=.5)),
        ('payload_20', dict(payload_kg=2.)), ('fast', dict(speed=.35)),
        ('delay_20', dict(observation_delay_ms=20)), ('delay_40', dict(observation_delay_ms=40)),
        ('half_dt', dict(timestep=.0025)), ('pause', dict(pause_on_course=True)),
    ):
        cases.append(replace(base, name=f'stairs_up_060_{tag}', **changes))
    for amplitude in (.02, .04, .06, .08, .10):
        cases.append(CourseSpec(f'tiles_{round(amplitude*1000):03d}', kind='tiles', amplitude=amplitude))
    for amplitude in (.02, .04, .06, .10):
        cases.append(CourseSpec(f'ridges_{round(amplitude*1000):03d}', kind='ridges', amplitude=amplitude))
    for angle in (5., 10., 15., 20.):
        for direction in ('up', 'down'):
            cases.append(CourseSpec(f'ramp_{direction}_{int(angle):02d}', kind='ramp', slope_deg=angle, direction=direction, length=3.))
    for direction in ('up', 'down'):
        for suffix, changes in (('long', dict(length=8., width=2.4)),
                                ('pause', dict(length=6., width=2.4, pause_on_course=True)),
                                ('delay', dict(length=6., width=2.4, observation_delay_ms=20))):
            cases.append(CourseSpec(f'ramp_{direction}_10_{suffix}', kind='ramp', slope_deg=10., direction=direction, **changes))
    for name in ('flat', 'stairs_up_060', 'stairs_down_060', 'tiles_040'):
        original = next(c for c in cases if c.name == name)
        cases.append(replace(original, name=name+'_observed', guidance='observed'))
    for tag, changes in (('low_friction', dict(friction=.2)), ('payload_20', dict(payload_kg=2.)), ('half_dt', dict(timestep=.0025))):
        cases.append(CourseSpec('tiles_040_'+tag, kind='tiles', amplitude=.04, **changes))
    return {case.name: case for case in cases}
