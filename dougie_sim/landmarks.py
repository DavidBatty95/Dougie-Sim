"""Seeded simulator targets and an ideal, occlusion-aware semantic sensor.

Only LandmarkObservation packets cross into autonomy. Placement, total count and
per-floor completion are evaluator information, not a supplied navigation map.
"""
from dataclasses import asdict, dataclass
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


PREFIX = 'discovery_target_'
RAY_GROUPS = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)


def add_facility_landmarks(root, count, seed):
    if count < 2 or count > 64 or seed < 0:
        raise ValueError('landmarks require count 2..64 and a nonnegative seed')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    model.geom_group[model.geom_bodyid != 0] = 4
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rng, positions = np.random.default_rng(seed), []
    geom = np.empty(1, dtype=np.int32)
    for index in range(count):
        height = (index % 2)*2.4
        for _ in range(10000):
            xy = rng.uniform([-2., -3.], [14.8, 4.])
            point = np.r_[xy, height+.55]
            if any(np.linalg.norm(point-p) < .8 for p in positions):
                continue
            clear = True
            for dx, dy in ((0., 0.), (.35, 0.), (-.35, 0.), (0., .35), (0., -.35)):
                origin = np.r_[xy+[dx, dy], height+2.]
                distance = mujoco.mj_ray(model, data, origin, np.array([0., 0., -1.]), RAY_GROUPS, 1, -1, geom)
                if abs(distance-2.) > .015:
                    clear = False
                    break
            if clear:
                positions.append(point)
                ET.SubElement(root.find('worldbody'), 'site', name=f'{PREFIX}{index:03d}', type='box',
                    pos=' '.join(map(str, point)), size='.08 .08 .08', rgba='.15 .9 .95 1', group='1')
                break
        else:
            raise ValueError('could not place separated landmarks on clear facility supports')


@dataclass(frozen=True)
class LandmarkObservation:
    identifier: str
    position: tuple
    stamp_s: float
    inspected: bool


class LandmarkSensor:
    """Ideal semantic IDs/positions, 4 m range, 120° horizontal / 80° vertical FOV.

    Identification and world positions are ideal simulation measurements; this
    is not a camera detector or a sensor-only localization implementation.
    """
    def __init__(self, sim):
        self.sites = [i for i in range(sim.model.nsite) if sim.model.site(i).name.startswith(PREFIX)]
        self.observed = {}

    def scan(self, sim):
        origin = sim.data.qpos[:3]+[0., 0., .12]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, sim.data.qpos[3:7])
        rotation = rotation.reshape(3, 3)
        hit = np.empty(1, dtype=np.int32)
        packets = []
        for site in self.sites:
            point = sim.data.site_xpos[site]
            delta = point-origin
            distance = float(np.linalg.norm(delta))
            local = delta@rotation
            if (not .05 < distance <= 4. or abs(math.atan2(local[1], local[0])) > math.pi/3
                    or abs(math.atan2(local[2], np.linalg.norm(local[:2]))) > math.radians(40.)):
                continue
            obstruction = mujoco.mj_ray(sim.model, sim.data, origin, delta/distance, RAY_GROUPS, 1, sim.base, hit)
            if obstruction >= 0 and obstruction < distance-.02:
                continue
            identifier = sim.model.site(site).name.removeprefix(PREFIX)
            old = self.observed.get(identifier)
            observation = LandmarkObservation(identifier, tuple(map(float, point)), float(sim.data.time),
                bool(distance <= 1.5 or (old is not None and old.inspected)))
            self.observed[identifier] = observation
            packets.append(observation)
        return tuple(packets)

    def evaluation(self, sim):
        targets = []
        for site in self.sites:
            identifier = sim.model.site(site).name.removeprefix(PREFIX)
            observation = self.observed.get(identifier)
            targets.append(dict(identifier=identifier, position=sim.data.site_xpos[site].tolist(),
                floor='upper' if sim.data.site_xpos[site, 2] > 2. else 'lower',
                discovered=observation is not None, inspected=bool(observation and observation.inspected)))
        return dict(scope='evaluator-only target catalogue; ideal occluded semantic detections', targets=targets,
            total=len(targets), discovered=len(self.observed), inspected=sum(o.inspected for o in self.observed.values()),
            per_floor={floor: dict(total=sum(t['floor'] == floor for t in targets),
                discovered=sum(t['floor'] == floor and t['discovered'] for t in targets),
                inspected=sum(t['floor'] == floor and t['inspected'] for t in targets)) for floor in ('lower', 'upper')})

    def packets(self):
        return [asdict(o) for o in self.observed.values()]
