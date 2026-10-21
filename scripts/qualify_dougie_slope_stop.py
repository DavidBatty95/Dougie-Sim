#!/usr/bin/env python3
"""Controlled zero-command slope dwell, with an experimental encoder-hold comparator.

Fixture pose is used only for initialization and evaluation. Commands are timed;
policy attitude and joints come from noisy proprioception. This is a screening
experiment, not deployment qualification of the encoder-hold comparator.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import torch

from dougie_sim.odometry import FusedOdometry
from dougie_sim.onboard_sensors import OnboardSensors
from dougie_sim.simulation import Simulation
from dougie_sim.terrain_courses import Course, CourseSpec
from dougie_sim.pose_uncertainty import core_fingerprint


def trial(angle, mode, seed, duration):
    course = Course(CourseSpec(name='stop_screen', kind='flat' if angle == 0 else 'ramp',
                              slope_deg=angle or 8., length=8., width=3., friction=.4), seed)
    sim = Simulation(model_xml=course.xml)
    # Place on the middle of the fixture once, before integration starts.
    sim.data.qpos[:3] = [4., 0., course.surface_height(4., 0.)+.34]
    mujoco.mj_forward(sim.model, sim.data)
    sensors, odometry = OnboardSensors(seed), FusedOdometry()
    robot = {i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] != 0}
    feet = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    start, trace, held_at, error = None, [], None, None
    nonfoot_impulse = 0.
    max_drift = 0.
    wrench = np.zeros(6)
    while sim.data.time < duration+5.:
        measured = sensors.proprioception(sim)
        odometry.update(measured)
        sim.onboard_proprioception = measured, odometry.rotation
        if sim.data.time >= 5. and start is None:
            start = sim.data.qpos[:3].copy()
        if start is not None and mode != 'zero_policy' and held_at is None and np.all(measured.foot_force > 5.):
            target_action = ((measured.joints-sim.default)/sim.policy_contract.action_scale
                             if mode == 'encoder_hold' else sim.action.copy())
            action = torch.from_numpy(target_action.astype(np.float32))[None, :]
            sim.policy = lambda observation: action
            held_at = float(sim.data.time)
        try:
            sim.step((0., 0., 0.))
        except RuntimeError as exc:
            error = str(exc)
            break
        if start is not None:
            max_drift = max(max_drift, float(np.linalg.norm(sim.data.qpos[:2]-start[:2])))
            for contact_index in range(sim.data.ncon):
                contact = sim.data.contact[contact_index]
                pair = {int(contact.geom1), int(contact.geom2)}
                if pair & robot and pair-robot and not pair & feet:
                    mujoco.mj_contactForce(sim.model, sim.data, contact_index, wrench)
                    nonfoot_impulse += abs(float(wrench[0]))*sim.model.opt.timestep
        if sim.counter % 200 == 0:
            trace.append(dict(time_s=float(sim.data.time), evaluator_position=sim.data.qpos[:3].tolist()))
    return dict(angle_deg=angle, mode=mode, sensor_seed=seed, hold_started_s=held_at, error=error,
        max_horizontal_drift_m=max_drift, nonfoot_impulse_ns=nonfoot_impulse,
        scene_sha256=hashlib.sha256(course.xml.encode()).hexdigest(), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--modes', nargs='+', choices=('zero_policy', 'encoder_hold', 'target_hold'),
                        default=['zero_policy', 'encoder_hold'])
    args = parser.parse_args()
    if args.output.exists() or not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('output must be new and duration positive')
    cases = [(angle, mode, seed) for angle in (0., 8.53, 10.)
             for mode in args.modes for seed in (0, 1)]
    report = dict(scope=__doc__, duration_s=args.duration, cases=cases, source_sha256=core_fingerprint(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), results=[])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for angle, mode, seed in cases:
        result = trial(angle, mode, seed, args.duration)
        report['results'].append(result)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'trace'}), flush=True)


if __name__ == '__main__':
    main()
