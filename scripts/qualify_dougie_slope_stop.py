#!/usr/bin/env python3
"""Controlled stop screening with frozen-target and active-feedback comparators.

Fixture pose is used only for initialization and evaluation. Approach commands
are timed; corrective feedback uses onboard estimates. This is a screening
experiment, not deployment qualification of any comparator.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch
from threadpoolctl import threadpool_limits

from dougie_sim.odometry import FusedOdometry
from dougie_sim.onboard_sensors import OnboardSensors
from dougie_sim.simulation import Simulation
from dougie_sim.terrain_courses import Course, CourseSpec
from dougie_sim.pose_uncertainty import core_fingerprint
from dougie_sim.recovery import healthy_local_return


class HoldAnchor:
    """A stamped estimated target in the same revision as the local pose graph."""
    def __init__(self, position, stamp, revision):
        self.position, self.stamp, self.revision = np.asarray(position).copy(), stamp, revision

    def correct(self, events):
        for event in events:
            if event.revision <= self.revision:
                continue
            if event.revision != self.revision+1:
                raise ValueError('hold anchor missed a map revision')
            self.position = event.point(self.position, self.stamp)
            self.revision = event.revision


def observed_stop_scene(course, enabled):
    if not enabled:
        return course.xml
    from dougie_sim.reactor import box
    root = ET.fromstring(course.xml)
    body = root.find('worldbody')
    # Simulator-only walls supply independent surface normals to noisy LiDAR.
    # Their poses/identities are never exposed to registration or feedback.
    for name, position, size in (
        ('north', (4.5, 2.2, 1.5), (5., .05, 1.5)),
        ('south', (4.5, -2.2, 1.5), (5., .05, 1.5)),
        ('west', (-.5, 0., 1.5), (.05, 2.2, 1.5)),
        ('east', (9.5, 0., 1.5), (.05, 2.2, 1.5))):
        box(body, 'stop_wall_'+name, position, size, '.5 .55 .6 1')
    return ET.tostring(root, encoding='unicode')


def hold_eligibility(measured, stamp, stopped_at, quiet_since):
    """Require fresh, quiet four-foot support after a zero-command settling delay."""
    quiet = (0 <= stamp-measured.stamp_s <= .05 and stamp-stopped_at >= .5
             and np.all(measured.foot_force > 5.) and np.linalg.norm(measured.gyro) <= .15
             and np.max(np.abs(measured.joint_velocity)) <= .5)
    if not quiet:
        return False, None
    quiet_since = stamp if quiet_since is None else quiet_since
    return stamp-quiet_since >= .2 and stamp-stopped_at <= 3., quiet_since


def position_hold_command(anchor, position, velocity, yaw, previous, dt):
    """Experimental planar PD feedback; inputs are estimates, never evaluator pose."""
    values = np.r_[anchor, position, velocity, yaw, previous, dt]
    if not np.isfinite(values).all() or not 0 < dt <= .05:
        raise ValueError('invalid position-hold observation or interval')
    world = .6*(np.asarray(anchor)[:2]-np.asarray(position)[:2])-.25*np.asarray(velocity)[:2]
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.array([[c, s], [-s, c]])@world
    body *= min(1., .08/max(float(np.linalg.norm(body)), 1e-12))
    change = body-np.asarray(previous)[:2]
    change *= min(1., .2*dt/max(float(np.linalg.norm(change)), 1e-12))
    return np.r_[np.asarray(previous)[:2]+change, 0.]


def trial(angle, mode, seed, duration, approach_s=0., direction='up', range_observations=False, dropout_after=None):
    course = Course(CourseSpec(name='stop_screen', kind='flat' if angle == 0 else 'ramp',
                              slope_deg=angle or 8., direction=direction, length=8., width=3., friction=.4), seed)
    scene = observed_stop_scene(course, range_observations)
    sim = Simulation(model_xml=scene)
    # Place on the middle of the fixture once, before integration starts.
    sim.data.qpos[:3] = [4., 0., course.surface_height(4., 0.)+.34]
    mujoco.mj_forward(sim.model, sim.data)
    sensors = OnboardSensors(seed, lidar_dropout_after=dropout_after)
    odometry = FusedOdometry(slam_backend='pose-graph' if range_observations else 'distributed')
    robot = {i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] != 0}
    feet = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    start, trace, held_at, error = None, [], None, None
    nonfoot_impulse = 0.
    max_drift = 0.
    peak_force, peak_torque, quiet_since = 0., 0., None
    approach_start = None
    estimated_anchor = None
    stamped_anchor = None
    next_scan = 0.
    range_ready = False
    range_inhibited_ticks = 0
    maximum_inhibited_command = 0.
    anchor_corrections = 0
    feedback_command = np.zeros(3)
    max_estimated_drift, feedback_inhibited_ticks = 0., 0
    stopped_at = 5.+approach_s
    wrench = np.zeros(6)
    while sim.data.time < duration+stopped_at:
        measured = sensors.proprioception(sim)
        odometry.update(measured)
        if range_observations and sim.data.time >= next_scan-1e-9:
            revision = odometry.map_revision
            odometry.observe_scan(sensors.scan(sim))
            if stamped_anchor is not None:
                stamped_anchor.correct(odometry.corrections[revision:])
                estimated_anchor = stamped_anchor.position.copy()
                anchor_corrections += odometry.map_revision-revision
            next_scan = float(sim.data.time)+.2
        if range_observations:
            range_ready = healthy_local_return(odometry.metrics(), float(sim.data.time))
        sim.onboard_proprioception = measured, odometry.rotation
        if sim.data.time >= 5. and approach_start is None:
            approach_start = sim.data.qpos[:3].copy()
        if sim.data.time >= stopped_at and start is None:
            start = sim.data.qpos[:3].copy()
            estimated_anchor = odometry.position.copy()
            stamped_anchor = HoldAnchor(estimated_anchor, float(sim.data.time), odometry.map_revision)
        eligible = np.all(measured.foot_force > 5.)
        if mode in ('guarded_target_hold', 'position_hold', 'range_position_hold'):
            eligible, quiet_since = hold_eligibility(measured, float(sim.data.time), stopped_at, quiet_since)
            if mode == 'range_position_hold':
                eligible = eligible and range_ready
        if start is not None and mode != 'zero_policy' and held_at is None and eligible:
            if mode not in ('position_hold', 'range_position_hold'):
                target_action = ((measured.joints-sim.default)/sim.policy_contract.action_scale
                                 if mode == 'encoder_hold' else sim.action.copy())
                action = torch.from_numpy(target_action.astype(np.float32))[None, :]
                sim.policy = lambda observation: action
            held_at = float(sim.data.time)
        command = (.15 if 5. <= sim.data.time < stopped_at else 0., 0., 0.)
        if held_at is not None and mode in ('position_hold', 'range_position_hold'):
            permitted = (odometry.initialized and 0 <= sim.data.time-measured.stamp_s <= .05
                         and (mode != 'range_position_hold' or range_ready))
            if permitted:
                feedback_command = position_hold_command(estimated_anchor, odometry.position, odometry.velocity,
                                                        odometry.yaw, feedback_command, sim.model.opt.timestep)
                command = feedback_command
            else:
                feedback_command = np.zeros(3)
                feedback_inhibited_ticks += 1
            if mode == 'range_position_hold' and not range_ready:
                range_inhibited_ticks += 1
                maximum_inhibited_command = max(maximum_inhibited_command, float(np.linalg.norm(command[:2])))
        try:
            sim.step(command)
        except RuntimeError as exc:
            error = str(exc)
            break
        if start is not None:
            max_drift = max(max_drift, float(np.linalg.norm(sim.data.qpos[:2]-start[:2])))
            max_estimated_drift = max(max_estimated_drift, float(np.linalg.norm(odometry.position[:2]-estimated_anchor[:2])))
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
                              estimated_position=odometry.position.tolist(),
                              range_feedback_ready=range_ready, map_revision=odometry.map_revision,
                              anchor_revision=None if stamped_anchor is None else stamped_anchor.revision,
                              command=sim.last_command.tolist(), held=held_at is not None))
    return dict(angle_deg=angle, direction=direction, mode=mode, sensor_seed=seed, hold_started_s=held_at, error=error,
        stop_requested_s=stopped_at, approach_s=approach_s,
        evaluator_approach_distance_m=None if start is None or approach_start is None else float(np.linalg.norm(start[:2]-approach_start[:2])),
        hold_entry_failed=mode != 'zero_policy' and held_at is None,
        max_estimated_horizontal_drift_m=max_estimated_drift, feedback_inhibited_ticks=feedback_inhibited_ticks,
        range_observations=range_observations, lidar_dropout_after_s=dropout_after,
        range_inhibited_ticks=range_inhibited_ticks, max_inhibited_requested_command_m_s=maximum_inhibited_command,
        anchor_corrections=anchor_corrections, localization=odometry.metrics(),
        nonfoot_peak_force_n=peak_force, peak_motor_torque_nm=peak_torque,
        policy=sim.policy_contract.identity(),
        max_horizontal_drift_m=max_drift, nonfoot_impulse_ns=nonfoot_impulse,
        scene_sha256=hashlib.sha256(scene.encode()).hexdigest(), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--approach-seconds', type=float, default=0.)
    parser.add_argument('--directions', nargs='+', choices=('up', 'down'), default=['up'])
    parser.add_argument('--angles', nargs='+', type=float, default=[0., 8.53, 10.])
    parser.add_argument('--sensor-seeds', nargs='+', type=int, default=[0, 1])
    parser.add_argument('--range-observations', action='store_true')
    parser.add_argument('--lidar-dropout-after', type=float)
    parser.add_argument('--modes', nargs='+', choices=('zero_policy', 'encoder_hold', 'target_hold', 'guarded_target_hold', 'position_hold', 'range_position_hold'),
                        default=['zero_policy', 'encoder_hold'])
    args = parser.parse_args()
    if (args.output.exists() or not np.isfinite([args.duration, args.approach_seconds]).all()
            or args.duration <= 0 or not 0 <= args.approach_seconds <= 10.):
        parser.error('output must be new, duration positive, and approach between 0 and 10 seconds')
    if (any(not np.isfinite(angle) or not 0 <= angle <= 10 for angle in args.angles)
            or any(seed < 0 for seed in args.sensor_seeds)
            or any(len(values) != len(set(values)) for values in (args.angles, args.sensor_seeds, args.directions, args.modes))):
        parser.error('angles/seeds must be valid and case selections unique')
    if (('range_position_hold' in args.modes or args.lidar_dropout_after is not None) and not args.range_observations):
        parser.error('range hold and dropout require --range-observations')
    if args.lidar_dropout_after is not None and (not np.isfinite(args.lidar_dropout_after) or args.lidar_dropout_after < 0):
        parser.error('dropout time must be finite and nonnegative')
    cases = [(angle, mode, seed, direction) for direction in args.directions for angle in args.angles
             if angle != 0 or direction == args.directions[0] for mode in args.modes for seed in args.sensor_seeds]
    report = dict(scope=__doc__, duration_s=args.duration, approach_s=args.approach_seconds,
                  hold_entry=dict(zero_command_delay_s=.5, quiet_dwell_s=.2, maximum_entry_delay_s=3.,
                      minimum_each_foot_force_n=5., maximum_gyro_norm_rad_s=.15, maximum_joint_speed_rad_s=.5),
                  position_hold=dict(kp=.6, kd=.25, command_norm_limit_m_s=.08, command_slew_limit_m_s2=.2,
                      estimator='IMU/leg/LiDAR pose graph' if args.range_observations else 'IMU/leg only; no LiDAR corrections', enabled_in_missions=False),
                  range_observations=args.range_observations, lidar_dropout_after_s=args.lidar_dropout_after,
                  range_scan_period_s=.2 if args.range_observations else None,
                  cases=cases, source_sha256=core_fingerprint(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), results=[])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for angle, mode, seed, direction in cases:
        with threadpool_limits(limits=1):
            result = trial(angle, mode, seed, args.duration, args.approach_seconds, direction,
                           args.range_observations, args.lidar_dropout_after)
        report['results'].append(result)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'trace'}), flush=True)


if __name__ == '__main__':
    main()
