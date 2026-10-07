#!/usr/bin/env python3
"""Measured ascent/descent trial; no pose resets after initialization."""
import json
import math
import argparse
from pathlib import Path

import mujoco
import numpy as np

from dougie_sim.simulation import Simulation, rotation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--terrain', choices=('stairs', 'uneven'), default='stairs')
parser.add_argument('--output', type=Path)
args = parser.parse_args()
sim = Simulation('stairs' if args.terrain == 'stairs' else 'reactor')
target_x = 9. if args.terrain == 'stairs' else -4.
target_y = 0. if args.terrain == 'stairs' else -2.
if args.terrain == 'uneven':
    sim.data.qpos[:2] = [-1., -2.]
    sim.data.qpos[3:7] = [0., 0., 0., 1.]
    mujoco.mj_forward(sim.model, sim.data)
contacts = set()
max_z = 0.
error = None
try:
    for step in range(30000):
        p = sim.data.qpos[:3]
        matrix = rotation(sim.data.qpos[3:7])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        heading = math.atan2(target_y-p[1], target_x-p[0])
        heading_error = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
        sim.step((.2*max(0., math.cos(heading_error)), 0., np.clip(1.8*heading_error, -.6, .6)))
        max_z = max(max_z, float(sim.data.qpos[2]))
        for contact in sim.data.contact:
            for geom in (contact.geom1, contact.geom2):
                name = sim.model.geom(geom).name
                if name.startswith(('step_', 'floor_plate_')):
                    contacts.add(name)
        if (args.terrain == 'stairs' and sim.data.qpos[0] > 8.) or (args.terrain == 'uneven' and sim.data.qpos[0] < -3.8):
            break
except RuntimeError as exc:
    error = str(exc)
traversed = (sim.data.qpos[0]>8. and max_z>.58 and sim.data.qpos[2]<.45) if args.terrain == 'stairs' else (sim.data.qpos[0]<-3.8 and len(contacts)>=2)
result = dict(**sim.metrics(), maximum_base_height_m=max_z, error=error, terrain=args.terrain,
              terrain_contacts=sorted(contacts), traversed=bool(traversed and error is None))
if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['traversed'] else 1)
