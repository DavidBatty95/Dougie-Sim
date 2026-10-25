#!/usr/bin/env python3
"""Screen production terminal stance under reduced traction and timed lateral pushes.

Fixture placement, applied forces and physical scoring are evaluator-only.
The controller receives noisy proprioception and estimated attitude only.
This is a finite simulation screen, not hardware or mission qualification.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import platform

import mujoco
import numpy as np
import torch
from threadpoolctl import threadpool_limits

from dougie_sim.odometry import FusedOdometry
from dougie_sim.onboard_sensors import OnboardSensors
from dougie_sim.pose_uncertainty import core_fingerprint
from dougie_sim.simulation import Simulation
from dougie_sim.terrain_courses import Course, CourseSpec


def trial(angle, direction, friction, force, seed, duration):
    course = Course(CourseSpec(name='stance_disturbance', kind='flat' if angle == 0 else 'ramp',
        slope_deg=angle or 8., direction=direction, length=8., width=3., friction=friction), seed)
    sim = Simulation(model_xml=course.xml)
    sim.data.qpos[:3] = [4., 0., course.surface_height(4., 0.)+.34]
    mujoco.mj_forward(sim.model, sim.data)
    sensors, odometry = OnboardSensors(seed), FusedOdometry()
    stop, push_start, push_duration = 13., 23., .5
    robot = {i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] != 0}
    feet = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    start = push_origin = None
    drift = post_push_drift = impulse = peak_force = applied_impulse = peak_torque = 0.
    trace, error = [], None
    contact_friction = []
    wrench = np.zeros(6)
    while sim.data.time < stop+duration:
        stamp = float(sim.data.time)
        measured = sensors.proprioception(sim)
        odometry.update(measured)
        sim.onboard_proprioception = measured, odometry.rotation
        if stamp >= stop and start is None:
            start = sim.data.qpos[:3].copy()
            sim.request_terminal_stance()
        if stamp >= push_start and push_origin is None:
            push_origin = sim.data.qpos[:3].copy()
        # Constant world-lateral force is prescribed by the fixture protocol,
        # never selected from true pose or used by the stance controller.
        applied = force if push_start <= stamp < push_start+push_duration else 0.
        sim.data.xfrc_applied[sim.base, :] = 0.
        sim.data.xfrc_applied[sim.base, 1] = applied
        try:
            sim.step((.15 if 5. <= stamp < stop else 0., 0., 0.))
        except RuntimeError as exc:
            error = str(exc)
            break
        applied_impulse += applied*sim.model.opt.timestep
        if start is not None:
            drift = max(drift, float(np.linalg.norm(sim.data.qpos[:2]-start[:2])))
        if push_origin is not None:
            post_push_drift = max(post_push_drift, float(np.linalg.norm(sim.data.qpos[:2]-push_origin[:2])))
        if stamp >= 5.:
            peak_torque = max(peak_torque, float(np.max(np.abs(sim.data.ctrl[sim.actuators]))))
            for index in range(sim.data.ncon):
                contact = sim.data.contact[index]
                pair = {int(contact.geom1), int(contact.geom2)}
                if pair & feet and pair-robot:
                    contact_friction.append(float(contact.friction[0]))
                if pair & robot and pair-robot and not pair & feet:
                    mujoco.mj_contactForce(sim.model, sim.data, index, wrench)
                    impulse += abs(float(wrench[0]))*sim.model.opt.timestep
                    peak_force = max(peak_force, abs(float(wrench[0])))
        if sim.counter % 200 == 0:
            trace.append(dict(time_s=float(sim.data.time), evaluator_position=sim.data.qpos[:3].tolist(),
                force_n=applied, stance=None if sim.terminal_stance is None else sim.terminal_stance.metrics()))
    stance = None if sim.terminal_stance is None else sim.terminal_stance.metrics()
    failures = []
    if error:
        failures.append('simulation_error')
    if stance is None or stance['phase'] != 'holding':
        failures.append('stance_not_retained')
    if impulse > .1 or peak_force > 20.:
        failures.append('nonfoot_contact')
    if drift > .05:
        failures.append('drift_above_5cm')
    if abs(applied_impulse-force*push_duration) > abs(force)*sim.model.opt.timestep+1e-8:
        failures.append('incomplete_push')
    if sim.data.time < stop+duration:
        failures.append('incomplete_duration')
    friction_range = None if not contact_friction else [min(contact_friction), max(contact_friction)]
    if friction_range is None or not np.allclose(friction_range, friction):
        failures.append('contact_friction_mismatch')
    return dict(angle_deg=angle, direction=direction, friction=friction, force_n=force, sensor_seed=seed,
        simulated_time_s=float(sim.data.time), applied_impulse_ns=applied_impulse, error=error,
        max_horizontal_drift_m=drift, max_post_push_drift_m=post_push_drift,
        nonfoot_impulse_ns=impulse, nonfoot_peak_force_n=peak_force, peak_motor_torque_nm=peak_torque,
        stance=stance, contact_friction_range=friction_range, failures=failures, passed=not failures, policy=sim.policy_contract.identity(),
        scene_sha256=hashlib.sha256(course.xml.encode()).hexdigest(), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=45.)
    args = parser.parse_args()
    if args.output.exists() or not np.isfinite(args.duration) or args.duration < 20.:
        parser.error('output must be new and post-stop duration at least 20 seconds')
    cases = list(itertools.product(((0., 'up'), (8.53, 'up'), (8.53, 'down')), (.4, .25), (0., 15.), (0, 1)))
    report = dict(scope=__doc__, protocol=dict(stop_s=13., approach_s=8., push_start_s=23.,
        push_duration_s=.5, post_stop_duration_s=args.duration, drift_limit_m=.05,
        nonfoot_impulse_limit_ns=.1, nonfoot_peak_force_limit_n=20., force_axis='world +Y',
        controller='production GravityStanceHold; no LiDAR'), expected_cases=len(cases),
        source_sha256=core_fingerprint(),
        fixture_sha256=hashlib.sha256(Path(__file__).resolve().parents[1].joinpath('dougie_sim/terrain_courses.py').read_bytes()).hexdigest(),
        software_versions=dict(python=platform.python_version(), numpy=np.__version__, mujoco=mujoco.__version__, torch=torch.__version__),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), results=[])
    for (angle, direction), friction, force, seed in cases:
        with threadpool_limits(limits=1):
            result = trial(angle, direction, friction, force, seed, args.duration)
        report['results'].append(result)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: result[key] for key in ('angle_deg', 'direction', 'friction', 'force_n',
            'sensor_seed', 'max_horizontal_drift_m', 'failures')}), flush=True)


if __name__ == '__main__':
    main()
