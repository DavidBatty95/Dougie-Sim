#!/usr/bin/env python3
"""Evaluator-only check of foot/terrain friction mixing in the pinned robot model."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco

from dougie_sim.simulation import Simulation
from dougie_sim.terrain_courses import Course, CourseSpec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = []
    for terrain_friction in (.25, 1.):
        course = Course(CourseSpec(name='contact_mixing', kind='ramp', slope_deg=8.53, friction=.25, length=8.), 0)
        sim = Simulation(controller='stand', model_xml=course.xml)
        sim.data.qpos[:3] = [4., 0., course.surface_height(4., 0.)+.34]
        ramp = sim.model.geom('qual_ramp').id
        sim.model.geom_friction[ramp, 0] = terrain_friction
        mujoco.mj_forward(sim.model, sim.data)
        feet = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
        contacts = set()
        for _ in range(400):
            sim.step()
            for contact in sim.data.contact:
                pair = {int(contact.geom1), int(contact.geom2)}
                if ramp in pair and pair & feet:
                    foot = next(iter(pair & feet))
                    contacts.add((int(sim.model.geom_priority[foot]), int(sim.model.geom_priority[ramp]), float(contact.friction[0])))
        if not contacts:
            raise ValueError('probe did not observe foot/ramp contact')
        results.append(dict(foot_friction=.25, terrain_friction=terrain_friction,
            observed_priority_and_effective_friction=sorted(contacts)))
    report = dict(scope=__doc__, mujoco_version=mujoco.__version__,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        policy=sim.policy_contract.identity(), results=results)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
