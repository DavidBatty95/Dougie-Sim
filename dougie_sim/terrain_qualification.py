"""Independent, source-attributed terrain qualification with retained failures."""
import argparse
from collections import Counter, deque
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import platform
import time
import traceback

import mujoco
import numpy as np
import torch
from scipy.stats import beta
from threadpoolctl import threadpool_limits

from .assets import ROOT, ensure_assets
from .simulation import JOINTS, Simulation, rotation
from .terrain_courses import Course, CourseSpec, catalogue

PROTOCOL_PATH = ROOT/'config/dougie_terrain_protocol.json'


def source_identity():
    paths = [*sorted((ROOT/'dougie_sim').glob('*.py')), PROTOCOL_PATH, ROOT/'config/dougie_assets.json']
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def qualification_failures(metrics, protocol):
    """Fail closed. Reaching the goal is necessary, not sufficient."""
    reasons = []
    if not metrics['data_valid']:
        reasons.append('invalid_simulation_data')
    if metrics['terminal_reason'] != 'completed':
        reasons.append(metrics['terminal_reason'])
    if not metrics['upper_landing_witness']:
        reasons.append('missing_upper_landing_witness')
    if metrics['nonfoot_impulse_ns'] > protocol['nonfoot_contact_impulse_limit_ns'] or metrics['nonfoot_peak_force_n'] > protocol['nonfoot_contact_peak_limit_n']:
        reasons.append('nonfoot_impact')
    if metrics['max_penetration_m'] > protocol['penetration_limit_m']:
        reasons.append('excessive_penetration')
    if metrics['max_joint_limit_violation_rad'] > protocol['joint_limit_tolerance_rad']:
        reasons.append('joint_limit_violation')
    if metrics['terminal_reason'] == 'completed' and metrics['settled_speed_m_s'] > protocol['settled_planar_speed_max_m_s']:
        reasons.append('failed_to_settle')
    return sorted(set(reasons))


class DelayedPolicy:
    def __init__(self, policy, ticks):
        self.policy = policy
        self.history = deque(maxlen=ticks+1)

    def __call__(self, observation):
        self.history.append(observation.clone())
        return self.policy(self.history[0])


class ObservedGuidance:
    """Same sensed support graph and route follower used by radiation mode.

    The goal XY and allowable qualification corridor are given, but support
    heights, terrain geometry and collision identities are not supplied.
    Sensing/planning is synchronous in simulation time; this is not a latency test.
    """
    def __init__(self, spec):
        from .terrain import SurfaceMap
        self.map = SurfaceMap()
        self.spec = spec
        self.path = []
        self.last_plan = -1.
        self.plans = 0
        self.blocked_plans = 0
        self.planning_ms = []

    def command(self, sim):
        from .radiation_run import follow, support_pose
        position = support_pose(sim)
        if sim.data.time-self.last_plan >= .5:
            started = time.perf_counter()
            points, supports = sim.lidar3d()
            self.map.update(points, supports, sim.data.qpos[:3]+[0, 0, .12])
            self.map.build_graph()
            allowed = {key for key, p in self.map.nodes.items()
                       if abs(p[1]) <= self.spec.width/2-.25 and -.5 <= p[0] <= self.spec.finish_x+.5}
            self.map.nodes = {key: p for key, p in self.map.nodes.items() if key in allowed}
            self.map.graph = {key: [other for other in neighbours if other in allowed]
                              for key, neighbours in self.map.graph.items() if key in allowed}
            root = self.map.nearest(position)
            self.path = []
            if root is not None:
                _, _, _, parent = self.map.routes(root)
                # Aim beyond the evaluator's crossing plane, as velocity guidance
                # does. Targeting the plane itself stops one grid-cell tolerance
                # short and falsely reports a stall even on a flat floor.
                target = observed_target(self.map.nodes, parent, self.spec.finish_x)
                self.path = [self.map.nodes[key].copy() for key in self.map.path(parent, target)[1:]]
            self.plans += 1
            self.blocked_plans += int(not self.path)
            self.last_plan = sim.data.time
            self.planning_ms.append((time.perf_counter()-started)*1000.)
        matrix = rotation(sim.data.qpos[3:7])
        command = follow(self.path, position, math.atan2(matrix[1, 0], matrix[0, 0]))
        command[0] *= self.spec.speed/.22
        return command


def observed_target(nodes, reachable, finish_x):
    return min(reachable, key=lambda key: math.hypot(nodes[key][0]-(finish_x+.5), nodes[key][1]))


def velocity_command(sim, spec, stopped=False):
    point = sim.data.qpos[:3]
    matrix = rotation(sim.data.qpos[3:7])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    heading = 0. if stopped else math.atan2(-point[1], max(.5, spec.finish_x+.5-point[0]))
    error = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
    speed = 0. if stopped else spec.speed*max(0., math.cos(error))
    return np.array([speed, 0., np.clip(1.8*error, -.6, .6)])


def run_trial(job):
    spec_data, seed, protocol, directory, identity, visualize = job
    spec = CourseSpec(**spec_data)
    output = Path(directory)/f'{spec.name}__seed_{seed:03d}'
    output.mkdir(parents=True, exist_ok=False)
    (output/'request.json').write_text(json.dumps(dict(spec=spec_data, seed=seed, protocol=protocol, source_sha256=identity), indent=2)+'\n')
    try:
        with threadpool_limits(limits=1):
            result = _trial(spec, seed, protocol, output, visualize)
    except Exception as exc:
        result = dict(case=spec.name, spec=spec_data, seed=seed, data_valid=False,
                      qualified=False, traversed=False, reasons=['harness_error'], error=str(exc))
        (output/'error.txt').write_text(traceback.format_exc())
    result.update(source_sha256=identity, request_sha256=hashlib.sha256((output/'request.json').read_bytes()).hexdigest())
    (output/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def _trial(spec, seed, protocol, output, visualize):
    start_time = time.perf_counter()
    course = Course(spec, seed)
    (output/'scene.xml').write_text(course.xml)
    sim = Simulation(model_xml=course.xml)
    model, data = sim.model, sim.data
    rng = np.random.default_rng(seed)
    initial_y = 0. if seed == 0 else rng.uniform(-protocol['initial_lateral_jitter_m'], protocol['initial_lateral_jitter_m'])
    yaw = 0. if seed == 0 else math.radians(rng.uniform(-protocol['initial_yaw_jitter_deg'], protocol['initial_yaw_jitter_deg']))
    jitter = np.zeros(12) if seed == 0 else rng.uniform(-protocol['initial_joint_jitter_rad'], protocol['initial_joint_jitter_rad'], 12)
    height_jitter = 0. if seed == 0 else rng.uniform(-protocol['initial_height_jitter_m'], protocol['initial_height_jitter_m'])
    # Initialization only. No root pose/velocity assignments occur in the trial loop.
    data.qpos[:7] = [0., initial_y, course.spec.initial_height+.32+height_jitter, math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
    data.qpos[sim.qadr] += jitter
    mujoco.mj_forward(model, data)
    initial_qpos = data.qpos.copy()
    if spec.observation_delay_ms:
        sim.policy = DelayedPolicy(sim.policy, spec.observation_delay_ms//20)
    robot_geoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] != 0}
    feet = {model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    foot_bodies = [model.body(name+'_foot').id for name in ('FR', 'FL', 'RR', 'RL')]
    assert len(feet) == 4 and model.nu == 12 and model.neq == 0 and model.nmocap == 0
    guidance = ObservedGuidance(spec) if spec.guidance == 'observed' else None
    viewer = None
    if visualize:
        from mujoco import viewer as viewer_module
        viewer = viewer_module.launch_passive(model, data)
        viewer.cam.distance, viewer.cam.elevation = 4., -25.
    metrics = dict(terminal_reason='timeout', data_valid=True, upper_landing_witness=course.required_top is None,
        nonfoot_impulse_ns=0., nonfoot_peak_force_n=0., max_penetration_m=0., max_joint_limit_violation_rad=0.,
        max_tilt_deg=0., max_abs_y_m=abs(initial_y), max_base_z_m=float(data.qpos[2]), min_clearance_m=10.,
        settled_speed_m_s=0., mechanical_work_j=0., motor_saturation_fraction=0.)
    friction_values, foot_forces, slip_speeds, final_speeds = [], [], [], []
    terrain_contacts = Counter()
    nonfoot_contacts = Counter()
    contact_events = []
    trace = []
    progress = deque()
    wrench, velocity = np.empty(6), np.empty(6)
    tilt_time = low_time = upper_time = 0.
    goal_stamp = None
    pause_stamp = None
    pause_done = False
    saturated, motor_samples = 0, 0
    dt = float(model.opt.timestep)
    sample_interval = max(1, round(1./protocol['trace_rate_hz']/dt))
    try:
        while data.time < spec.timeout_s:
            if viewer and not viewer.is_running():
                metrics['terminal_reason'] = 'viewer_closed'
                break
            if spec.pause_on_course and not pause_done and pause_stamp is None and data.qpos[0] > 1.+spec.steps*spec.tread/2:
                pause_stamp = float(data.time)
            paused = pause_stamp is not None and data.time-pause_stamp < 2.
            if pause_stamp is not None and not paused:
                pause_done = True
            stopped = goal_stamp is not None or paused
            command = velocity_command(sim, spec, stopped) if guidance is None or stopped else guidance.command(sim)
            try:
                sim.step(command)
            except RuntimeError as exc:
                metrics['terminal_reason'] = 'fall' if 'fell' in str(exc) else 'controller_error'
                metrics['runtime_error'] = str(exc)
                break
            point = data.qpos[:3]
            matrix = rotation(data.qpos[3:7])
            tilt = math.degrees(math.acos(float(np.clip(matrix[2, 2], -1., 1.))))
            clearance = float(point[2]-course.surface_height(float(point[0]), float(point[1])))
            metrics['max_tilt_deg'] = max(metrics['max_tilt_deg'], tilt)
            metrics['min_clearance_m'] = min(metrics['min_clearance_m'], clearance)
            metrics['max_abs_y_m'] = max(metrics['max_abs_y_m'], abs(float(point[1])))
            metrics['max_base_z_m'] = max(metrics['max_base_z_m'], float(point[2]))
            metrics['mechanical_work_j'] += float(np.abs(data.ctrl[sim.actuators]*data.qvel[sim.vadr]).sum()*dt)
            saturated += int(np.sum(np.abs(data.ctrl[sim.actuators]) >= 23.49))
            motor_samples += 12
            q = data.qpos[sim.qadr]
            violation = np.maximum(model.jnt_range[sim.joint_ids, 0]-q, q-model.jnt_range[sim.joint_ids, 1])
            metrics['max_joint_limit_violation_rad'] = max(metrics['max_joint_limit_violation_rad'], float(np.max(violation)))
            for i, contact in enumerate(data.contact):
                g1, g2 = int(contact.geom1), int(contact.geom2)
                if (g1 in robot_geoms) == (g2 in robot_geoms):
                    continue
                robot, environment = (g1, g2) if g1 in robot_geoms else (g2, g1)
                mujoco.mj_contactForce(model, data, i, wrench)
                force = max(0., float(wrench[0]))
                metrics['max_penetration_m'] = max(metrics['max_penetration_m'], max(0., -float(contact.dist)))
                if force < protocol['nonfoot_contact_force_min_n']:
                    continue
                if robot in feet:
                    terrain_contacts[model.geom(environment).name or f'geom_{environment}'] += 1
                    friction_values.append(float(contact.friction[0]))
                    foot_forces.append(force)
                    if force > 5.:
                        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_GEOM, robot, velocity, 0)
                        linear = velocity[3:]+np.cross(velocity[:3], contact.pos-data.geom_xpos[robot])
                        normal = np.asarray(contact.frame[:3])
                        slip_speeds.append(float(np.linalg.norm(linear-normal*np.dot(linear, normal))))
                else:
                    label = model.body(int(model.geom_bodyid[robot])).name
                    nonfoot_contacts[label] += 1
                    metrics['nonfoot_impulse_ns'] += force*dt
                    metrics['nonfoot_peak_force_n'] = max(metrics['nonfoot_peak_force_n'], force)
                    if len(contact_events) < 64:
                        contact_events.append(dict(time_s=float(data.time), body=label,
                            terrain=model.geom(environment).name, normal_force_n=force))
            if course.required_top:
                lo, hi, height = course.required_top
                witnessed = lo+.3 <= point[0] <= hi-.25 and np.min(data.xpos[foot_bodies, 2]) >= height-.03
                upper_time = upper_time+dt if witnessed else 0.
                metrics['upper_landing_witness'] |= upper_time >= protocol['upper_landing_witness_s']
            tilt_time = tilt_time+dt if tilt > protocol['tilt_limit_deg'] else 0.
            low_time = low_time+dt if clearance < protocol['ground_clearance_min_m'] else 0.
            if tilt_time >= protocol['tilt_limit_duration_s'] or low_time >= protocol['low_clearance_duration_s']:
                metrics['terminal_reason'] = 'unstable_attitude_or_clearance'
                break
            if abs(point[1]) > spec.width/2-protocol['corridor_body_margin_m']:
                metrics['terminal_reason'] = 'left_test_corridor'
                break
            if (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or
                    any(w.number for w in data.warning) or np.any(data.xfrc_applied)):
                metrics['data_valid'] = False
                metrics['terminal_reason'] = 'invalid_simulation_data'
                break
            if point[0] >= spec.finish_x and goal_stamp is None:
                goal_stamp = float(data.time)
            if goal_stamp is not None:
                final_speeds.append(float(np.linalg.norm(data.qvel[:2])))
                if data.time-goal_stamp >= protocol['settle_time_s']:
                    final_clearance = float(point[2]-spec.final_height)
                    metrics['terminal_reason'] = 'completed' if .18 < final_clearance < .45 else 'wrong_finish_height'
                    break
            if not paused and goal_stamp is None:
                progress.append((float(data.time), float(point[0])))
                while len(progress) > 1 and data.time-progress[1][0] > protocol['stall_window_s']:
                    progress.popleft()
                if data.time-progress[0][0] >= protocol['stall_window_s'] and point[0]-progress[0][1] < protocol['stall_progress_m']:
                    metrics['terminal_reason'] = 'observed_route_stall' if guidance else 'locomotion_stall'
                    break
            if sim.counter % sample_interval == 0:
                trace.append(np.r_[data.time, data.qpos, data.qvel, data.ctrl, command])
            if viewer:
                viewer.cam.lookat[:] = point
                if sim.counter % sim.control_decimation == 0:
                    viewer.sync()
                time.sleep(max(0., start_time+data.time-time.perf_counter()))
    finally:
        if viewer:
            viewer.close()
    # Always retain terminal state, including a fall that interrupted stepping.
    if np.isfinite(data.qpos).all():
        final_matrix = rotation(data.qpos[3:7])
        metrics['max_tilt_deg'] = max(metrics['max_tilt_deg'], math.degrees(math.acos(float(np.clip(final_matrix[2, 2], -1., 1.)))))
        metrics['max_abs_y_m'] = max(metrics['max_abs_y_m'], abs(float(data.qpos[1])))
        metrics['max_base_z_m'] = max(metrics['max_base_z_m'], float(data.qpos[2]))
    trace.append(np.r_[data.time, data.qpos, data.qvel, data.ctrl, sim.last_command])
    np.savez_compressed(output/'trace.npz', rows=np.asarray(trace),
        columns=np.asarray(['time']+[f'qpos_{i}' for i in range(model.nq)]+[f'qvel_{i}' for i in range(model.nv)]+
                           [f'ctrl_{i}' for i in range(model.nu)]+['cmd_vx', 'cmd_vy', 'cmd_yaw']))
    metrics['data_valid'] &= bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all() and not any(w.number for w in data.warning))
    metrics['motor_saturation_fraction'] = saturated/max(1, motor_samples)
    metrics['settled_speed_m_s'] = float(np.mean(final_speeds[-max(1, round(.5/dt)):])) if final_speeds else 0.
    metrics['qualified'] = not qualification_failures(metrics, protocol)
    metrics['reasons'] = qualification_failures(metrics, protocol)
    metrics['traversed'] = bool(metrics['terminal_reason'] == 'completed' and metrics['upper_landing_witness'] and metrics['data_valid'])
    metrics.update(case=spec.name, spec=asdict(spec), seed=seed, simulation_time_s=float(data.time),
        wall_time_s=time.perf_counter()-start_time, initial_qpos=initial_qpos.tolist(), final_qpos=data.qpos.tolist(),
        foot_contacts=dict(terrain_contacts), nonfoot_contacts=dict(nonfoot_contacts), first_nonfoot_events=contact_events,
        effective_foot_friction_range=[float(min(friction_values)), float(max(friction_values))] if friction_values else None,
        peak_foot_force_n=max(foot_forces, default=0.), stance_contact_slip_p95_m_s=float(np.percentile(slip_speeds, 95)) if slip_speeds else None,
        scene_sha256=hashlib.sha256(course.xml.encode()).hexdigest(),
        trace_sha256=hashlib.sha256((output/'trace.npz').read_bytes()).hexdigest(),
        observation_contract='ideal root pose for guidance; policy proprioception only; no terrain height observations in walking policy',
        guidance_statistics=None if guidance is None else dict(plans=guidance.plans, blocked_plans=guidance.blocked_plans,
            plan_p95_ms=float(np.percentile(guidance.planning_ms, 95)) if guidance.planning_ms else None))
    return metrics


def aggregate(records, protocol):
    groups = {}
    for record in records:
        groups.setdefault(record['case'], []).append(record)
    rows = []
    for name, runs in sorted(groups.items()):
        n = len(runs)
        qualified = sum(bool(r['qualified']) for r in runs)
        traversed = sum(bool(r['traversed']) for r in runs)
        lower = float(beta.ppf(1.-protocol['confidence'], qualified, n-qualified+1)) if qualified else 0.
        rows.append(dict(case=name, trials=n, traversed=traversed, qualified=qualified,
            qualified_probability_lower_one_sided_95=lower,
            meets_repeatability_target=bool(lower >= protocol['repeatability_target']),
            reasons=dict(Counter(reason for r in runs for reason in r['reasons'])),
            median_traversal_s=float(np.median([r['simulation_time_s'] for r in runs if r['traversed']])) if traversed else None,
            max_nonfoot_impulse_ns=max((r.get('nonfoot_impulse_ns', 0.) for r in runs), default=0.)))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=('smoke', 'matrix', 'repeatability'), default='matrix')
    parser.add_argument('--cases', nargs='+', help='exact case names; overrides suite')
    parser.add_argument('--seeds', nargs='+', type=int)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--require-qualified', action='store_true', help='nonzero exit for any qualification failure')
    args = parser.parse_args()
    cases = catalogue()
    if args.list:
        print(json.dumps([asdict(c) for c in cases.values()], indent=2))
        return
    selected = args.cases or (['flat', 'legacy_06', 'stairs_down_060', 'tiles_040'] if args.suite == 'smoke' else
        ['flat', 'stairs_up_030', 'stairs_down_030', 'tiles_020', 'tiles_040', 'ramp_up_10', 'ramp_down_10'] if args.suite == 'repeatability' else list(cases))
    if any(name not in cases for name in selected) or len(set(selected)) != len(selected):
        parser.error('unknown or repeated case; use --list')
    seeds = args.seeds if args.seeds is not None else list(range(101, 161)) if args.suite == 'repeatability' else [0] if args.suite == 'smoke' else [1, 2, 3, 4, 5]
    if args.workers < 1 or any(seed < 0 for seed in seeds) or len(set(seeds)) != len(seeds):
        parser.error('workers must be positive; seeds must be unique and nonnegative')
    if args.view and (len(selected)*len(seeds) != 1 or args.workers != 1):
        parser.error('--view requires one case, one seed, and --workers 1')
    ensure_assets()
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('terrain_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    protocol = json.loads(PROTOCOL_PATH.read_text())
    identity = source_identity()
    header = dict(schema_version=1, created_at=datetime.now(timezone.utc).isoformat(),
        protocol=protocol, source_sha256=identity,
        environment=dict(python=platform.python_version(), mujoco=mujoco.__version__, torch=torch.__version__, numpy=np.__version__, machine=platform.machine()),
        cases=[asdict(cases[name]) for name in selected], seeds=seeds,
        expected_trials=len(selected)*len(seeds), scope='bounded seeded simulation qualification, not hardware certification')
    (output/'manifest.json').write_text(json.dumps(header, indent=2)+'\n')
    jobs = [(asdict(cases[name]), seed, protocol, str(output), identity, args.view) for name in selected for seed in seeds]
    results = []
    def record(result):
        results.append(result)
        print(f"{len(results)}/{len(jobs)} {result['case']} seed={result['seed']}: traversed={result['traversed']} qualified={result['qualified']} {result['reasons']}", flush=True)
        (output/'summary.json').write_text(json.dumps(dict(**header, completed_trials=len(results),
            cases_summary=aggregate(results, protocol), complete=len(results)==len(jobs)), indent=2)+'\n')
    if args.workers == 1:
        for job in jobs:
            record(run_trial(job))
    else:
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context('spawn')) as pool:
            for future in as_completed([pool.submit(run_trial, job) for job in jobs]):
                record(future.result())
    print(f'Qualification evidence: {output}', flush=True)
    if not all(r['data_valid'] for r in results) or (args.require_qualified and not all(r['qualified'] for r in results)):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
