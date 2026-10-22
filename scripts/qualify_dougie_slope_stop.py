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


def hold_eligibility(measured, stamp, stopped_at, quiet_since):
    """Require fresh, quiet four-foot support after a zero-command settling delay."""
    quiet = (0 <= stamp-measured.stamp_s <= .05 and stamp-stopped_at >= .5
             and np.all(measured.foot_force > 5.) and np.linalg.norm(measured.gyro) <= .15
             and np.max(np.abs(measured.joint_velocity)) <= .5)
    if not quiet:
        return False, None
    quiet_since = stamp if quiet_since is None else quiet_since
    return stamp-quiet_since >= .2 and stamp-stopped_at <= 3., quiet_since


def trial(angle, mode, seed, duration, approach_s=0., direction='up'):
    course = Course(CourseSpec(name='stop_screen', kind='flat' if angle == 0 else 'ramp',
                              slope_deg=angle or 8., direction=direction, length=8., width=3., friction=.4), seed)
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
    peak_force, peak_torque, quiet_since = 0., 0., None
    approach_start = None
    stopped_at = 5.+approach_s
    wrench = np.zeros(6)
    while sim.data.time < duration+stopped_at:
        measured = sensors.proprioception(sim)
        odometry.update(measured)
        sim.onboard_proprioception = measured, odometry.rotation
        if sim.data.time >= 5. and approach_start is None:
            approach_start = sim.data.qpos[:3].copy()
        if sim.data.time >= stopped_at and start is None:
            start = sim.data.qpos[:3].copy()
        eligible = np.all(measured.foot_force > 5.)
        if mode == 'guarded_target_hold':
            eligible, quiet_since = hold_eligibility(measured, float(sim.data.time), stopped_at, quiet_since)
        if start is not None and mode != 'zero_policy' and held_at is None and eligible:
            target_action = ((measured.joints-sim.default)/sim.policy_contract.action_scale
                             if mode == 'encoder_hold' else sim.action.copy())
            action = torch.from_numpy(target_action.astype(np.float32))[None, :]
            sim.policy = lambda observation: action
            held_at = float(sim.data.time)
        try:
            sim.step((.15 if 5. <= sim.data.time < stopped_at else 0., 0., 0.))
        except RuntimeError as exc:
            error = str(exc)
            break
        if start is not None:
            max_drift = max(max_drift, float(np.linalg.norm(sim.data.qpos[:2]-start[:2])))
        if approach_start is not None:
            peak_torque = max(peak_torque, float(np.max(np.abs(sim.data.ctrl[sim.actuators]))))
            for contact_index in range(sim.data.ncon):
                contact = sim.data.contact[contact_index]
                pair = {int(contact.geom1), int(contact.geom2)}
                if pair & robot and pair-robot and not pair & feet:
                    mujoco.mj_contactForce(sim.model, sim.data, contact_index, wrench)
                    nonfoot_impulse += abs(float(wrench[0]))*sim.model.opt.timestep
                    peak_force = max(peak_force, abs(float(wrench[0])))
        if sim.counter % 200 == 0:
            trace.append(dict(time_s=float(sim.data.time), evaluator_position=sim.data.qpos[:3].tolist(),
                              command=sim.last_command.tolist(), held=held_at is not None))
    return dict(angle_deg=angle, direction=direction, mode=mode, sensor_seed=seed, hold_started_s=held_at, error=error,
        stop_requested_s=stopped_at, approach_s=approach_s,
        evaluator_approach_distance_m=None if start is None or approach_start is None else float(np.linalg.norm(start[:2]-approach_start[:2])),
        hold_entry_failed=mode != 'zero_policy' and held_at is None,
        nonfoot_peak_force_n=peak_force, peak_motor_torque_nm=peak_torque,
        policy=sim.policy_contract.identity(),
        max_horizontal_drift_m=max_drift, nonfoot_impulse_ns=nonfoot_impulse,
        scene_sha256=hashlib.sha256(course.xml.encode()).hexdigest(), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--approach-seconds', type=float, default=0.)
    parser.add_argument('--directions', nargs='+', choices=('up', 'down'), default=['up'])
    parser.add_argument('--modes', nargs='+', choices=('zero_policy', 'encoder_hold', 'target_hold', 'guarded_target_hold'),
                        default=['zero_policy', 'encoder_hold'])
    args = parser.parse_args()
    if (args.output.exists() or not np.isfinite([args.duration, args.approach_seconds]).all()
            or args.duration <= 0 or not 0 <= args.approach_seconds <= 10.):
        parser.error('output must be new, duration positive, and approach between 0 and 10 seconds')
    cases = [(angle, mode, seed, direction) for direction in args.directions for angle in (0., 8.53, 10.)
             if angle != 0 or direction == args.directions[0] for mode in args.modes for seed in (0, 1)]
    report = dict(scope=__doc__, duration_s=args.duration, approach_s=args.approach_seconds,
                  hold_entry=dict(zero_command_delay_s=.5, quiet_dwell_s=.2, maximum_entry_delay_s=3.,
                      minimum_each_foot_force_n=5., maximum_gyro_norm_rad_s=.15, maximum_joint_speed_rad_s=.5),
                  cases=cases, source_sha256=core_fingerprint(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), results=[])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for angle, mode, seed, direction in cases:
        result = trial(angle, mode, seed, args.duration, args.approach_seconds, direction)
        report['results'].append(result)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'trace'}), flush=True)


if __name__ == '__main__':
    main()
