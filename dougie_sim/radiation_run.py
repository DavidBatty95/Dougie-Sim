"""Dose-constrained 3-D radiation mapping mission in native Go2 physics."""
import argparse
from collections import Counter
import hashlib
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import multiprocessing
import subprocess
from pathlib import Path
import time

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

from .assets import ROOT, ensure_assets
from . import planning_worker
from .demo_overlay import DemoControls, update_viewer
from .radiation import Detector, RadiationField, available_samples
from .reactor import radiation_description
from .source_config import DEFAULT_SOURCE_CONFIG
from .simulation import Simulation, rotation
from .graphics import rviz_command


def support_pose(sim):
    feet = [sim.data.body(name+'_foot').xpos[2]-.022 for name in ('FR', 'FL', 'RR', 'RL')]
    return np.array([*sim.data.qpos[:2], np.mean(sorted(feet)[:2])])


def follow(path, position, yaw, waypoint_tolerance=.22):
    while path and math.dist(position, path[0]) < waypoint_tolerance:
        path.pop(0)
    if not path:
        return np.zeros(3)
    # Follow adjacent graph edges; do not shortcut across an unknown corner.
    target = path[0]
    error = math.atan2(target[1]-position[1], target[0]-position[0])-yaw
    error = math.atan2(math.sin(error), math.cos(error))
    speed = .22*max(0., math.cos(error)) if abs(error) < .8 else 0.
    if abs(target[2]-position[2]) > .025:
        speed = min(speed, .16)
    return np.array([speed, 0., np.clip(1.8*error, -.5, .5)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--dose-budget', type=float, default=2., help='mission budget in micro-sieverts')
    parser.add_argument('--rate-limit', type=float, default=100., help='planning threshold in micro-sieverts/hour')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--localization', choices=('fused', 'ground-truth'), default='fused')
    parser.add_argument('--sensor-seed', type=int, default=0)
    parser.add_argument('--navigation-profile', choices=('robust', 'legacy'), default='robust', help='quality-gated submaps and consistent history, or pre-recovery comparator')
    parser.add_argument('--slam-backend', choices=('distributed', 'pose-graph'), help='pose-graph for robust fusion; distributed retains the previous correction model')
    parser.add_argument('--lidar-dropout-after', type=float, help='fault experiment: all LiDAR beams missing after this simulated time')
    parser.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='reactor')
    parser.add_argument('--layout-seed', type=int, default=0, help='facility doorway/equipment layout seed, separate from detector noise')
    parser.add_argument('--landmark-count', type=int, default=0, help='optional seeded facility discovery targets (2..64)')
    parser.add_argument('--landmark-seed', type=int, default=0)
    parser.add_argument('--mission-objective', choices=('survey', 'inspect', 'circuit'), default='survey')
    parser.add_argument('--utility-mode', choices=('legacy', 'information'), help='default legacy utility; information is an experimental cost/whole-task scorer')
    parser.add_argument('--circuit-radius', type=float, default=1.5, help='radius of four goals selected from initially observed support')
    parser.add_argument('--require-circuit', action='store_true', help='require all four physical circuit witnesses and clean home return')
    parser.add_argument('--region-scheduling', choices=('local', 'hierarchical'), default='hierarchical')
    parser.add_argument('--policy-config', type=Path, help='versioned TorchScript deployment manifest')
    parser.add_argument('--gp-capacity', type=int, default=128)
    parser.add_argument('--gp-mode', choices=('global', 'spatial', 'geometry'), default='geometry')
    parser.add_argument('--detector-likelihood', choices=('window', 'point'), default='window')
    parser.add_argument('--gp-expert-capacity', type=int, default=32, help='inducing points per spatial expert')
    parser.add_argument('--gp-max-experts', type=int, default=128, help='bounded spatial model count')
    parser.add_argument('--gp-cell-size', type=float, nargs=3, default=(4., 4., 1.5), metavar=('X', 'Y', 'Z'))
    parser.add_argument('--path-mode', choices=('grid', 'smooth'), default='smooth')
    parser.add_argument('--gp-likelihood', choices=('poisson', 'gaussian'), default='poisson')
    parser.add_argument('--planner', choices=('baseline', 'clustered', 'clustered-no-commit'), default='clustered')
    parser.add_argument('--terrain-profile', choices=('conservative', 'experimental', 'layered'), default=None,
                        help='conservative: 0.4 m clearance and 2 cm edge steps; experimental: original 0.3 m / 7.5 cm')
    parser.add_argument('--camera', choices=('follow', 'overview'), default='follow')
    parser.add_argument('--map-layer', choices=('mean', 'uncertainty'), default='mean')
    parser.add_argument('--field-display', choices=('detected', 'all'), default='detected')
    parser.add_argument('--display-background', type=float, help='operator background calibration in uSv/h; defaults to scenario background')
    parser.add_argument('--fixed-plan-delay', type=float, help='headless fast benchmark: fixed simulated planner latency in seconds')
    parser.add_argument('--require-clean', action='store_true', help='independent evaluator aborts on non-foot impact and fails incomplete/unclean returns')
    parser.add_argument('--require-round-trip', action='store_true', help='require clean upstairs/downstairs/home witnesses; evaluator only')
    parser.add_argument('--exit-on-complete', action='store_true', help='close the viewer immediately rather than inspect the paused final map')
    parser.add_argument('--record-demo', action='store_true', help='headless offscreen GIF of the actual run; use MUJOCO_GL=egl')
    parser.add_argument('--record-view', choices=('field', 'routes'), default='field', help='recording-only overlay and camera style')
    parser.add_argument('--source-profile', help='defaults to the selected world scenario')
    parser.add_argument('--source-config', type=Path, help='point-source JSON; defaults to config/dougie_radiation_sources.json')
    parser.add_argument('--show-sources', action='store_true', help='diagnostic viewer markers: red hotspots, amber leaks')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--visualizer', choices=('rviz', 'mujoco'), default='rviz',
                        help='interactive renderer (RViz runs independently of physics)')
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    args.slam_backend = args.slam_backend or ('pose-graph' if args.navigation_profile == 'robust' else 'distributed')
    if args.slam_backend == 'pose-graph' and args.navigation_profile != 'robust':
        parser.error('--slam-backend pose-graph requires --navigation-profile robust')
    args.utility_mode = args.utility_mode or 'legacy'
    if not math.isfinite(args.circuit_radius) or not 1. <= args.circuit_radius <= 3.:
        parser.error('--circuit-radius must be between 1 and 3 metres')
    if args.require_circuit:
        if args.mission_objective != 'circuit':
            parser.error('--require-circuit requires --mission-objective circuit')
        args.require_clean = True
    if args.mission_objective == 'circuit' and args.planner == 'baseline':
        parser.error('observed circuit requires a clustered planner')
    if args.lidar_dropout_after is not None and (args.localization != 'fused' or not math.isfinite(args.lidar_dropout_after) or args.lidar_dropout_after < 0):
        parser.error('--lidar-dropout-after requires fused localization and a nonnegative finite time')
    if args.gp_mode == 'geometry' and args.gp_likelihood != 'poisson':
        parser.error('--gp-mode geometry requires --gp-likelihood poisson')
    if args.localization == 'fused' and args.landmark_count:
        parser.error('onboard fused missions exclude ideal semantic landmarks; use --localization ground-truth for that diagnostic comparator')
    if args.mission_objective == 'inspect' and (args.landmark_count == 0 or args.planner == 'baseline'):
        parser.error('--mission-objective inspect requires discovery landmarks and a clustered planner')
    if args.landmark_seed < 0 or (args.landmark_count != 0 and
            (args.world != 'facility' or not 2 <= args.landmark_count <= 64 or args.planner == 'baseline')):
        parser.error('landmarks require facility, a clustered planner, count 2..64 and a nonnegative seed')
    if args.require_round_trip:
        if args.world not in ('mezzanine', 'facility'):
            parser.error('--require-round-trip requires a multi-floor world')
        args.require_clean = True
    args.terrain_profile = args.terrain_profile or ('layered' if args.world in ('mezzanine', 'facility') else 'conservative')
    args.source_profile = args.source_profile or (args.world if args.world in ('mezzanine', 'facility') else 'hotspots-and-leaks')
    if not math.isfinite(args.duration) or args.duration <= 0 or not 8 <= args.gp_capacity <= 512:
        parser.error('invalid duration or GP capacity (8..512)')
    if (not 8 <= args.gp_expert_capacity <= 128 or not 1 <= args.gp_max_experts <= 512 or
            not all(math.isfinite(v) and v > 0 for v in args.gp_cell_size)):
        parser.error('invalid spatial GP capacity, expert count or cell dimensions')
    if args.fast and (not args.headless or args.ros):
        parser.error('--fast requires --headless without --ros')
    if args.record_demo and not args.headless:
        parser.error('--record-demo requires --headless')
    if args.display_background is not None and (not math.isfinite(args.display_background) or args.display_background < 0):
        parser.error('--display-background must be a finite nonnegative rate')
    if args.fixed_plan_delay is not None and (not args.fast or not math.isfinite(args.fixed_plan_delay) or not 0. <= args.fixed_plan_delay <= 1.):
        parser.error('--fixed-plan-delay requires --fast and a finite value in [0, 1]')
    config_path = args.source_config or DEFAULT_SOURCE_CONFIG
    if not args.headless and args.visualizer == 'rviz':
        args.ros = True
    try:
        if args.source_profile in ('mezzanine', 'facility'):
            if args.source_config:
                parser.error('use a named JSON profile with --source-config')
            if args.source_profile == 'facility':
                from .facility import source_description
            else:
                from .multilevel import source_description
            description = source_description()
        else:
            description = radiation_description(args.source_profile, config_path)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    ensure_assets()
    sim = Simulation(args.world, radiation_sources=description, policy_config=args.policy_config, layout_seed=args.layout_seed,
                     landmark_count=args.landmark_count, landmark_seed=args.landmark_seed)
    odometry = sensors = None
    if args.localization == 'fused':
        if sim.policy_contract.perceptive:
            parser.error('fused missions currently require the proprioceptive policy contract')
        from .odometry import FusedOdometry
        from .onboard_sensors import OnboardSensors
        odometry, sensors = FusedOdometry(robust=args.navigation_profile == 'robust', slam_backend=args.slam_backend), OnboardSensors(args.sensor_seed, args.lidar_dropout_after)
    true_home = sim.data.qpos[:3].copy()  # evaluator only, never a planning goal
    pose_errors = []
    local_packet = None
    mapped_packet = None
    last_onboard_scan = -1.
    from .landmarks import LandmarkSensor
    landmark_sensor = LandmarkSensor(sim)
    field = RadiationField.from_mujoco(sim.model, sim.data)
    from .radiation_display import RadiationHighlights
    highlights = RadiationHighlights(field.background if args.display_background is None else args.display_background)
    # Archive the resolved runtime shielding, not the reactor's nominal list.
    description['shields'] = field.shields
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('radiation_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted((ROOT/'dougie_sim').glob('*.py'))}
    source_hashes['config/dougie_assets.json'] = hashlib.sha256((ROOT/'config/dougie_assets.json').read_bytes()).hexdigest()
    source_hashes['radiation_source_config'] = hashlib.sha256(config_path.read_bytes()).hexdigest()
    resolved_args = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    (output/'source_scenario.json').write_text(json.dumps(dict(profile=args.source_profile, **description), indent=2)+'\n')
    (output/'run_config.json').write_text(json.dumps({**resolved_args, 'output': str(output),
        'source_sha256': source_hashes, 'policy': sim.policy_contract.identity()}, indent=2)+'\n')
    (output/'scene.xml').write_text(sim.scene_xml)
    (output/'policy_contract.json').write_text(json.dumps(sim.policy_contract.config, indent=2)+'\n')
    detector = Detector(args.seed)
    parameters = (args.dose_budget, args.rate_limit, args.gp_capacity, args.duration, args.gp_likelihood)
    planning_options = {name: getattr(args, name) for name in
                        ('gp_mode', 'gp_expert_capacity', 'gp_max_experts', 'gp_cell_size', 'path_mode', 'mission_objective', 'region_scheduling', 'detector_likelihood', 'navigation_profile', 'utility_mode', 'circuit_radius')}
    planning_options['smooth_return_overlay'] = not args.headless or args.record_demo or args.ros
    planning_options['display_prediction'] = not args.headless or args.record_demo or args.ros
    planning_options['consistent_mapping'] = bool(odometry and odometry.robust)
    planning_options['recovery_enabled'] = args.navigation_profile == 'robust'
    explorer = planning_worker.create_explorer(parameters, args.planner, args.terrain_profile, planning_options)
    controls = DemoControls(args.camera, args.map_layer, args.show_sources)
    controls.highlights, controls.field_display = highlights, args.field_display
    for i in range(sim.model.nsite):
        if sim.model.site(i).name.startswith('radiation_source_'):
            point = sim.data.site_xpos[i].copy()
            height = float(point[2]+.85)
            for geom in range(sim.model.ngeom):
                if sim.model.geom_bodyid[geom] == 0 and sim.model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_PLANE:
                    size = sim.model.geom_size[geom].copy()
                    kind = sim.model.geom_type[geom]
                    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                        size[:] = size[0]
                    elif kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                        size = np.array([size[0], size[0], size[1]+(size[0] if kind == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.)])
                    extent = np.abs(sim.data.geom_xmat[geom].reshape(3, 3))@size
                    centre = sim.data.geom_xpos[geom]
                    if np.all(np.abs(point-centre) <= extent+.01):
                        height = max(height, float(centre[2]+extent[2]+.5))
            controls.sources.append((sim.model.site(i).name.removeprefix('radiation_source_'), point,
                                     sim.model.site_rgba[i].copy(), height))
    # Limit BLAS oversubscription so GP work cannot consume every control core.
    limits = threadpool_limits(limits=1)
    isolated_planning = args.terrain_profile == 'layered' or args.path_mode == 'smooth' or args.gp_mode == 'spatial'
    worker_args = dict(max_workers=1, initializer=planning_worker.initialize,
                       initargs=(parameters, args.planner, args.terrain_profile, isolated_planning, planning_options))
    if isolated_planning:
        # Python-heavy support fitting must not compete for the control-loop GIL.
        pool = ProcessPoolExecutor(mp_context=multiprocessing.get_context('spawn'), **worker_args)
    else:
        pool = ThreadPoolExecutor(thread_name_prefix='radiation-gp', **worker_args)
    future = None
    samples = []
    path, state, map_view = [], 'initial_survey', None
    from .path_motion import RouteFollower
    follower = RouteFollower()
    previous_command = np.zeros(3)
    command_variation = np.zeros(3)
    turning_time = 0.
    latest_sample = None
    true_dose = 0.
    error = None
    live_metrics = dict(state='initial_survey', planner=args.planner)
    trail = []
    trail_stamps = []
    display_history = []
    distance_travelled = 0.
    previous_position = sim.data.qpos[:3].copy()
    last_target = None
    target_switches = 0
    foot_geoms = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    robot_geoms = {i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] != 0}
    nonfoot_impulse, nonfoot_peak = 0., 0.
    nonfoot_contacts = Counter()
    contact_force = np.empty(6)
    from .mission_evaluation import MissionWitness
    witness = MissionWitness(sim, args.world)
    from .circuit_evaluation import CircuitWitness
    circuit_witness = CircuitWitness()
    evaluator_capture_frames = {}
    recorder = None
    if args.record_demo:
        from .demo_recording import DemoRecorder
        recorder = DemoRecorder(sim.model, output, args.duration, args.record_view)
    bridge = viewer = radiation_pub = cloud_pub = status_pub = None
    visualization = rviz = rviz_log = None
    if args.ros:
        from .ros_bridge import Bridge
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        # This mission uses 3-D LiDAR, not the navigation stack's extra 2-D scan.
        bridge = Bridge(sim, publish_scan=False)
        from .rviz_visualization import Visualization
        visualization = Visualization(bridge, args.show_sources, args.map_layer, cutaway=args.world == 'facility',
                                      highlights=highlights, field_display=args.field_display)
        radiation_pub = bridge.node.create_publisher(String, '/radiation/measurement', 10)
        status_pub = bridge.node.create_publisher(String, '/radiation/status', 10)
        cloud_pub = bridge.node.create_publisher(PointCloud2, '/radiation/gp_map', 1)
    if not args.headless and args.visualizer == 'mujoco':
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(sim.model, sim.data, key_callback=controls.key)
        viewer.cam.distance, viewer.cam.elevation = 6., -35.
        viewer.opt.sitegroup[5] = int(args.show_sources)
    started = time.monotonic()
    loop_times, bridge_times, viewer_times, lidar_times = [], [], [], []
    last_plan = -10.
    accepted_plan_stamp = -10.
    accepted_map_revision = 0
    observed_plan_delay = 0.
    minimum_sensor_z = math.inf
    maximum_sensor_z = -math.inf
    last_path_position = np.zeros(3) if odometry else support_pose(sim)
    last_progress_stamp = 0.
    next_report = 30.
    trace = (output/'trace.jsonl').open('w')
    measurements = (output/'measurements.jsonl').open('w')
    localization_log = (output/'localization_evaluation.jsonl').open('w')
    correction_log = (output/'map_corrections.jsonl').open('w')
    last_pose_log = -1.
    terminal_stop = None
    terminal_reported = False
    control_wait_start = None
    control_wait_dose = 0.

    print(f'Dougie radiation mission: {output}\nSource profile: {args.source_profile} ({len(field.sources)} point sources)', flush=True)
    try:
        if not args.headless and args.visualizer == 'rviz':
            rviz_log = (output/'rviz.log').open('w')
            rviz_config = 'dougie_facility.rviz' if args.world == 'facility' else 'dougie_radiation.rviz'
            rviz = subprocess.Popen(rviz_command(ROOT/'config'/rviz_config, output/'rviz_graphics.json'),
                                    stdout=rviz_log, stderr=subprocess.STDOUT)
        while sim.data.time < args.duration and (viewer is None or viewer.is_running()):
            loop_started = time.perf_counter()
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError(f'RViz exited with {rviz.returncode}; see {output}/rviz.log')
                state = 'interrupted'
                break
            stamp = float(sim.data.time)
            distance_travelled += float(np.linalg.norm(sim.data.qpos[:2]-previous_position[:2]))
            previous_position = sim.data.qpos[:3].copy()
            if odometry is not None:
                proprioception = sensors.proprioception(sim)
                odometry.update(proprioception)
                if stamp-last_onboard_scan >= .2-1e-9:
                    lidar_started = time.perf_counter()
                    local_packet = sensors.scan(sim)
                    revision_before = odometry.map_revision
                    odometry.observe_scan(local_packet)
                    for event in odometry.corrections[revision_before:]:
                        detector.correct_map(event)
                        samples = [event.sample(s) for s in samples]
                        if latest_sample is not None:
                            latest_sample = event.sample(latest_sample)
                        display_history = [event.sample(s) for s in display_history]
                        trail = [event.point(p, t).tolist() for p, t in zip(trail, trail_stamps)]
                        last_path_position = event.point(last_path_position, last_progress_stamp)
                        highlights.cells.clear()
                        highlights.last_stamp = -math.inf
                        highlights._tree = highlights._confirmed = None
                        for observation in display_history:
                            highlights.observe(observation)
                        path, map_view = [], None
                        follower.set_route([], [], odometry.support_position)
                        correction_log.write(json.dumps(asdict(event))+'\n')
                        correction_log.flush()
                    if odometry.initialized and (not odometry.robust or odometry.rejection_streak == 0):
                        mapped_packet = odometry.world_packet(local_packet)
                        if args.mission_objective == 'circuit':
                            # This transform only reaches the independent witness.
                            R_score = rotation(sim.data.qpos[3:7])@odometry.rotation.T
                            evaluator_capture_frames[local_packet.sequence] = (R_score, sim.data.qpos[:3].copy()-R_score@odometry.position)
                            if len(evaluator_capture_frames) > 64:
                                del evaluator_capture_frames[next(iter(evaluator_capture_frames))]
                    last_onboard_scan = stamp
                    lidar_times.append(time.perf_counter()-lidar_started)
                sim.onboard_proprioception = proprioception, odometry.rotation.copy()
                position = odometry.support_position
                if stamp-last_pose_log >= .1 and odometry.initialized:
                    pose_errors.append(float(np.linalg.norm(odometry.position-sim.data.qpos[:3])))
                    localization_log.write(json.dumps(dict(stamp_s=stamp, **odometry.metrics(),
                        evaluator_true_position=sim.data.qpos[:3].tolist(), evaluator_error_m=pose_errors[-1]))+'\n')
                    last_pose_log = stamp
            else:
                position = support_pose(sim)
            if stamp >= next_report:
                print(f't={stamp:.1f}s {state}: support_z={position[2]:.2f}m '
                      f'measured_dose={detector.dose:.3f}/{args.dose_budget:g}uSv', flush=True)
                next_report = stamp+30.
            if not trail or math.dist(position, trail[-1]) > .08:
                trail.append(position.tolist())
                trail_stamps.append(stamp)
                if len(trail) > 2000:
                    trail = trail[::2]
                    trail_stamps = trail_stamps[::2]
            sensor = sim.data.qpos[:3]+(rotation(sim.data.qpos[3:7])@np.array([0., 0., .12]) if odometry else [0., 0., .12])
            minimum_sensor_z = min(minimum_sensor_z, float(sensor[2]))
            maximum_sensor_z = max(maximum_sensor_z, float(sensor[2]))
            rate = field.rate(sensor)
            true_dose += rate*sim.model.opt.timestep/3600.
            sample = detector.integrate(odometry.sensor_position if odometry else sensor,
                rate, sim.model.opt.timestep, stamp+sim.model.opt.timestep,
                position_valid=not odometry or not odometry.robust or (odometry.initialized
                    and odometry.rejection_streak < 5 and odometry.position_std.max() <= .3))
            if sample is not None:
                latest_sample = sample
                display_history = (display_history+[sample])[-8192:]
                highlights.observe(sample)
                samples.append(sample)
                measurements.write(json.dumps(asdict(sample))+'\n')
                if radiation_pub:
                    radiation_pub.publish(String(data=json.dumps(asdict(sample))))
                if visualization:
                    visualization.update_dashboard(live_metrics, sample, args.dose_budget, args.rate_limit)
            ready = future is not None and (stamp-last_plan >= args.fixed_plan_delay if args.fixed_plan_delay is not None
                else future.done() or (args.fast and stamp-last_plan >= .5))
            if ready:
                path, state, metrics, map_view = future.result()
                accepted_map_revision = metrics.get('map_revision', 0)
                if odometry and accepted_map_revision != odometry.map_revision:
                    path, map_view, state = [], None, 'waiting_for_map_revision'
                    metrics.update(state=state, planned_path=[], path_spans=[])
                if metrics.get('recovery', {}).get('terminal') and terminal_stop is None:
                    terminal_stop = dict(time_s=stamp, reason=metrics['recovery'].get('reason'),
                        measured_dose_usv=detector.dose, evaluator_true_dose_usv=true_dose)
                if terminal_stop is not None:
                    path, state = [], 'stopped_unrecoverable'
                    metrics.update(state=state, planned_path=[], path_spans=[])
                observed_plan_delay = max(0., stamp-last_plan)
                metrics['plan_age_sim_s'] = observed_plan_delay
                if odometry:
                    metrics['localization'] = odometry.metrics()
                    if metrics.get('map_consistency', {}).get('retained_scans', 0) >= 992:
                        odometry.loop_closure_enabled = False
                if args.path_mode == 'smooth':
                    metrics['route_accepted'] = follower.set_route(path, metrics.get('path_spans', []), position)
                    metrics['route_replan_rejections'] = follower.replan_rejections
                    metrics['follower_status'] = follower.status
                live_metrics = metrics
                if args.mission_objective == 'circuit':
                    task = metrics.get('circuit', {})
                    circuit_witness.set_goals(task, evaluator_capture_frames.get(task.get('capture_sequence')))
                target = metrics.get('selected_target')
                if target is not None and (last_target is None or math.dist(target, last_target) > .35):
                    target_switches += 1
                    last_target = target
                accepted_plan_stamp = last_plan
                future = None
                trace.write(json.dumps(dict(time_s=stamp, **{k:v for k,v in metrics.items() if k != 'events'}))+'\n')
                trace.flush()
                if status_pub:
                    status_pub.publish(String(data=json.dumps({k:v for k,v in metrics.items() if k != 'events'})))
                if cloud_pub and map_view:
                    from sensor_msgs.msg import PointField
                    from sensor_msgs_py import point_cloud2
                    from std_msgs.msg import Header
                    query, mean, std, upper = map_view
                    fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z', 'dose_rate_usv_h', 'std_log_rate', 'upper_rate_usv_h'))]
                    cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=bridge.stamp(), frame_id='odom'), fields,
                        np.column_stack((query, mean, std, upper)).astype(np.float32)))
                if visualization:
                    visualization.update(map_view, metrics, trail)
                    visualization.update_dashboard(metrics, latest_sample, args.dose_budget, args.rate_limit)
                if state == 'returned':
                    break
            if math.dist(position, last_path_position) > .15:
                last_path_position, last_progress_stamp = position.copy(), stamp
            stalled = stamp-last_progress_stamp > 15.
            ready_samples, future_samples = [], samples
            if terminal_stop is None and future is None and stamp-last_plan >= 2.:
                ready_samples, future_samples = available_samples(samples, stamp) if args.navigation_profile == 'robust' else (samples, [])
            if terminal_stop is None and future is None and ready_samples and stamp-last_plan >= 2. and (odometry is None or (odometry.initialized and mapped_packet is not None)):
                lidar_started = time.perf_counter()
                packet = mapped_packet if odometry else sim.lidar_packet()
                if args.mission_objective == 'circuit' and odometry is None:
                    evaluator_capture_frames[packet.sequence] = (np.eye(3), np.zeros(3))
                    if len(evaluator_capture_frames) > 64:
                        del evaluator_capture_frames[next(iter(evaluator_capture_frames))]
                lidar_times.append(time.perf_counter()-lidar_started)
                landmark_observations = () if odometry else landmark_sensor.scan(sim)
                if visualization:
                    visualization.update_landmarks(landmark_sensor.packets())
                future = pool.submit(planning_worker.plan, ready_samples, packet, position.copy(), stamp, stalled,
                                     landmark_observations, observed_plan_delay,
                                     odometry.yaw if odometry else math.atan2(rotation(sim.data.qpos[3:7])[1, 0], rotation(sim.data.qpos[3:7])[0, 0]),
                                     tuple(odometry.corrections) if odometry else (), odometry.metrics() if odometry else None,
                                     follower.status if args.path_mode == 'smooth' else None)
                samples = future_samples
                last_plan = stamp
                if stalled:
                    last_progress_stamp = stamp
            R = rotation(sim.data.qpos[3:7])
            yaw = odometry.yaw if odometry else math.atan2(R[1, 0], R[0, 0])
            command = (follower.tick(position, yaw, sim.model.opt.timestep) if args.path_mode == 'smooth' else
                       follow(path, position, yaw, .14 if args.terrain_profile == 'layered' else .22))
            control_reason = None
            if latest_sample is None or stamp-latest_sample.stamp_s > 1.5 or stamp-accepted_plan_stamp > 4.:
                command[:] = 0.
                follower.stop()
                control_reason = 'stale_radiation' if latest_sample is None or stamp-latest_sample.stamp_s > 1.5 else 'stale_plan'
            if odometry and (not odometry.initialized or stamp-last_onboard_scan > .6
                    or odometry.position_std.max() > .3 or accepted_map_revision != odometry.map_revision
                    or (odometry.robust and odometry.rejection_streak >= 5)):
                command[:] = 0.
                follower.stop()
                control_reason = 'map_revision_wait' if accepted_map_revision != odometry.map_revision else 'localization_unreliable'
            if latest_sample is not None and (latest_sample.rate_usv_h > args.rate_limit or latest_sample.dose_upper_usv > args.dose_budget*.8) and not state.startswith('return'):
                command[:] = 0.
                follower.stop()
                control_reason = 'measured_radiation_requires_retreat'
            if args.navigation_profile == 'robust' and stamp > 3. and terminal_stop is None:
                if control_reason:
                    if control_wait_start is None:
                        control_wait_start, control_wait_dose = stamp, detector.dose
                    if stamp-control_wait_start >= 12. or detector.dose-control_wait_dose >= .02*args.dose_budget:
                        terminal_stop = dict(time_s=stamp, reason=control_reason,
                            measured_dose_usv=detector.dose, evaluator_true_dose_usv=true_dose)
                else:
                    control_wait_start = None
            if terminal_stop is not None:
                state, path = 'stopped_unrecoverable', []
                command[:] = 0.
                follower.stop()
                terminal_status = {k: v for k, v in terminal_stop.items() if not k.startswith('evaluator_')}
                live_metrics.update(state=state, terminal_stop=terminal_status, planned_path=[], selected_target=None)
                if not terminal_reported:
                    record = dict(time_s=stamp, **{k: v for k, v in live_metrics.items() if k != 'events'})
                    trace.write(json.dumps(record)+'\n')
                    trace.flush()
                    if status_pub:
                        status_pub.publish(String(data=json.dumps(record)))
                    terminal_reported = True
                # Keep integrating physics/detector exposure to the original
                # deadline. Ending planning must not manufacture a dose saving.
            live_metrics['control_inhibit_reason'] = control_reason
            command_variation += np.abs(command-previous_command)
            previous_command = command.copy()
            turning_time += float(abs(command[2]) > .1 and abs(command[0]) < .03)*sim.model.opt.timestep
            if bridge:
                bridge_started = time.perf_counter()
                bridge.tick()  # Publish sensing; the radiation executive owns motion.
                visualization.tick()
                bridge_times.append(time.perf_counter()-bridge_started)
            sim.step(command)
            witness.tick(sim)
            if args.mission_objective == 'circuit':
                circuit_witness.tick(support_pose(sim), float(sim.data.time), sim.model.opt.timestep)
            for i, contact in enumerate(sim.data.contact):
                a, b = int(contact.geom1), int(contact.geom2)
                if (a in robot_geoms) == (b in robot_geoms):
                    continue
                robot = a if a in robot_geoms else b
                if robot not in foot_geoms:
                    mujoco.mj_contactForce(sim.model, sim.data, i, contact_force)
                    force = max(0., float(contact_force[0]))
                    if force > 1.:
                        nonfoot_impulse += force*sim.model.opt.timestep
                        nonfoot_peak = max(nonfoot_peak, force)
                        environment = b if robot == a else a
                        nonfoot_contacts[sim.model.body(int(sim.model.geom_bodyid[robot])).name+' / '+sim.model.geom(environment).name] += 1
            if args.require_clean and (nonfoot_impulse > .1 or nonfoot_peak > 25.):
                raise RuntimeError('independent contact qualification failed; aborting trial')
            if recorder:
                recorder.capture(sim, map_view, live_metrics, trail, controls, latest_sample, args.dose_budget, args.rate_limit)
            if viewer and sim.counter % 4 == 0:
                viewer_started = time.perf_counter()
                update_viewer(viewer, sim.data.qpos[:3], map_view, live_metrics, trail, controls,
                              latest_sample, stamp, args.dose_budget, args.rate_limit)
                viewer.sync()
                viewer_times.append(time.perf_counter()-viewer_started)
            loop_times.append(time.perf_counter()-loop_started)
            if not args.fast:
                time.sleep(max(0., started+sim.data.time-time.monotonic()))
    except KeyboardInterrupt:
        state = 'interrupted'
    except Exception as exc:
        error = str(exc)
    finally:
        loop_wall_time = time.monotonic()-started
        try:
            if future is not None:
                _, _, _, map_view = future.result()
            explorer = pool.submit(planning_worker.finalize, samples, tuple(odometry.corrections) if odometry else ()).result()
        except Exception as exc:
            error = error or str(exc)
        finally:
            pool.shutdown(wait=True, cancel_futures=True)
        if recorder:
            recorder.capture(sim, map_view, {**live_metrics, 'state': state if error is None else 'FAILED: '+error},
                             trail, controls, latest_sample, args.dose_budget, args.rate_limit, force=True)
            recorder.close()
        trace.close()
        measurements.close()
        localization_log.close()
        correction_log.close()
        if odometry and odometry.pose_graph is not None:
            (output/'pose_graph.json').write_text(json.dumps(odometry.pose_graph.archive(), indent=2, allow_nan=False)+'\n')
        witness.save(output/'motion_trace.npz')
        if map_view is None and explorer.terrain.nodes:
            nodes = np.asarray(list(explorer.terrain.nodes.values()))
            nodes = nodes[::max(1, math.ceil(len(nodes)/100))]
            query = np.concatenate([nodes+[0., 0., z] for z in (.25, .75, 1.5)])
            map_view = (query, *explorer.gp.predict(query))
        if map_view:
            query = map_view[0]
            mean, std, upper = explorer.gp.predict(query)
            np.savez_compressed(output/'radiation_map_3d.npz', xyz=query, mean_usv_h=mean, std_log_rate=std, upper_usv_h=upper)
        gp = explorer.gp
        gp.save(output/'gp_posterior.npz')
        np.savez_compressed(output/'observed_geometry.npz',
            points=np.asarray(list(explorer.terrain.hits.values())).reshape(-1, 3),
            free_voxels=np.asarray(list(explorer.terrain.free_voxels)).reshape(-1, 3), voxel_size_m=.2)
        if hasattr(explorer.terrain, 'occupancy'):
            terrain = explorer.terrain
            np.savez_compressed(output/'layered_geometry.npz',
                schema_version=2,
                support_returns=np.asarray(list(terrain.support_returns.values())).reshape(-1, 3),
                occupancy_keys=np.asarray(list(terrain.occupancy.evidence), dtype=np.int32).reshape(-1, 3),
                occupancy_evidence=np.asarray(list(terrain.occupancy.evidence.values()), dtype=np.int8),
                coarse_free_keys=np.asarray(list(terrain.occupancy.coarse_free), dtype=np.int32).reshape(-1, 3),
                coarse_free_evidence=np.asarray(list(terrain.occupancy.coarse_free.values()), dtype=np.int8),
                coarse_free_voxel_size_m=2*terrain.occupancy.resolution,
                voxel_size_m=terrain.occupancy.resolution)
        metrics = dict(**explorer.metrics(), simulation=sim.metrics(), end_state=state, error=error,
            source_profile=args.source_profile, radiation_source_count=len(field.sources),
            source_sha256=source_hashes,
            true_dose_usv=true_dose, measured_dose_usv=detector.dose,
            sampled_height_range_m=[minimum_sensor_z, maximum_sensor_z],
            mission_completed=bool(state == 'returned'),
            actual_budget_exceeded=bool(true_dose > args.dose_budget),
            wall_time_s=time.monotonic()-started, seed=args.seed,
            information_contract=('planner receives body-frame noisy LiDAR transformed by IMU/leg/LiDAR estimated pose and counting-detector observations only' if odometry else 'planner receives range returns, ideal pose, counting-detector samples'
                                  +(', and visible landmark observations only' if args.landmark_count else ' only')))
        metrics['localization'] = odometry.metrics() if odometry else dict(source='mujoco_ground_truth')
        metrics['state'] = state
        metrics['terminal_stop'] = terminal_stop
        if terminal_stop:
            recovery_metrics = metrics.setdefault('recovery', {})
            source = 'planning_recovery' if recovery_metrics.get('terminal') else 'execution_watchdog'
            recovery_metrics.update(last_worker_phase=recovery_metrics.get('phase'), phase='stopped_unrecoverable',
                terminal=True, reason=terminal_stop['reason'], termination_source=source)
        metrics['post_stop_true_dose_usv'] = max(0., true_dose-terminal_stop['evaluator_true_dose_usv']) if terminal_stop else 0.
        metrics['localization_evaluation'] = dict(position_rmse_m=float(np.sqrt(np.mean(np.square(pose_errors)))) if pose_errors else None,
            position_p95_m=float(np.percentile(pose_errors, 95)) if pose_errors else None,
            true_home_distance_m=float(np.linalg.norm(sim.data.qpos[:3]-true_home)))
        metrics.update(planner=args.planner, path_length_m=distance_travelled, target_switches=target_switches,
                       world=args.world, exploration_complete=False,
                       gp_mode=args.gp_mode, path_mode=args.path_mode,
                       command_total_variation=command_variation.tolist(), turning_in_place_s=turning_time,
                       physical_witness=witness.metrics(),
                       completion_scope='return outcome only; complete unknown-space coverage is not established',
                       terrain_profile=args.terrain_profile, nonfoot_contacts=dict(nonfoot_contacts),
                       nonfoot_impulse_ns=nonfoot_impulse, nonfoot_peak_force_n=nonfoot_peak,
                        clean_return=bool(state == 'returned' and error is None and true_dose <= args.dose_budget
                                           and nonfoot_impulse <= .1 and nonfoot_peak <= 25.
                                           and metrics['localization_evaluation']['true_home_distance_m'] <= .4))
        if hasattr(explorer.terrain, 'metrics'):
            metrics['terrain'] = explorer.terrain.metrics()
        from .mission_evaluation import round_trip_failures
        metrics['round_trip_failures'] = round_trip_failures(metrics)
        metrics['round_trip_completed'] = not metrics['round_trip_failures']
        if args.mission_objective == 'circuit':
            evaluated_circuit = circuit_witness.metrics()
            metrics['circuit_evaluation'] = evaluated_circuit
            metrics['circuit_mission_completed'] = bool(metrics['clean_return'] and evaluated_circuit['all_goals_reached']
                and metrics.get('circuit', {}).get('completed'))
            (output/'circuit_evaluation.json').write_text(json.dumps(evaluated_circuit, indent=2)+'\n')
        metrics['route_replan_rejections'] = follower.replan_rejections
        metrics['worker_phase_p95_ms'] = {name: float(np.percentile(values, 95))
            for name, values in getattr(explorer, 'worker_phase_history', {}).items()}
        landmark_evaluation = landmark_sensor.evaluation(sim)
        metrics['landmarks'] = {k: v for k, v in landmark_evaluation.items() if k != 'targets'}
        metrics['landmark_mission_completed'] = bool(args.landmark_count and metrics['clean_return'] and
                                                   landmark_evaluation['inspected'] == args.landmark_count)
        (output/'landmark_evaluation.json').write_text(json.dumps(landmark_evaluation, indent=2)+'\n')
        (output/'landmark_observations.json').write_text(json.dumps(landmark_sensor.packets(), indent=2)+'\n')
        def timing(values):
            return dict(count=len(values), total_s=float(sum(values)),
                        p50_ms=float(np.percentile(values, 50)*1000) if values else 0.,
                        p95_ms=float(np.percentile(values, 95)*1000) if values else 0.,
                        p99_ms=float(np.percentile(values, 99)*1000) if values else 0.)
        metrics['performance'] = dict(
            planner_execution='spawned_process' if isolated_planning else 'thread',
            loop_wall_time_s=loop_wall_time,
            real_time_factor=float(sim.data.time/loop_wall_time),
            physics_steps_per_wall_second=sim.counter/loop_wall_time,
            physics_budget_ms=sim.model.opt.timestep*1000,
            steps_over_budget=sum(t > sim.model.opt.timestep for t in loop_times),
            active_loop=timing(loop_times), ros_and_visualization=timing(bridge_times),
            mujoco_viewer=timing(viewer_times), lidar3d=timing(lidar_times))
        (output/'radiation_summary.json').write_text(json.dumps(metrics, indent=2)+'\n')
        # Truth is exported only for independent post-run evaluation.
        (output/'evaluator_field.json').write_text(json.dumps(dict(background_usv_h=field.background,
            sources=field.sources, shields=field.shields), indent=2)+'\n')
        limits.restore_original_limits()
        console_metrics = {k:v for k,v in metrics.items() if k not in ('events', 'source_sha256', 'clusters', 'planned_path', 'return_path')}
        if 'exploration_agenda' in console_metrics:
            console_metrics['exploration_agenda'] = {k:v for k,v in console_metrics['exploration_agenda'].items() if k != 'tasks'}
        print(json.dumps(console_metrics, indent=2))
        if visualization:
            final_state = 'failed: '+error if error else state if state in ('returned', 'interrupted', 'stopped_unrecoverable') else 'time limit reached'
            visualization.update(map_view, {**live_metrics, 'planned_path': [], 'selected_target': None}, trail)
            visualization.update_dashboard({**live_metrics, 'state': final_state+' / PAUSED'},
                                           latest_sample, args.dose_budget, args.rate_limit)
        if rviz is not None:
            try:
                if not args.exit_on_complete and error is None and state != 'interrupted':
                    print('Mission ended. RViz shows the final map; close it or press Ctrl+C to exit.', flush=True)
                    while rviz.poll() is None:
                        import rclpy
                        rclpy.spin_once(bridge.node, timeout_sec=.1)
            except KeyboardInterrupt:
                pass
            finally:
                if rviz.poll() is None:
                    rviz.terminate()
                    try:
                        rviz.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        rviz.kill()
                        rviz.wait()
        if rviz_log:
            rviz_log.close()
        if bridge:
            bridge.close()
        if viewer:
            try:
                if not args.exit_on_complete:
                    print('Mission ended. Viewer is a paused inspection of the final state; close it or press Ctrl+C to exit.', flush=True)
                    inspection = {**live_metrics, 'state': state+' / PAUSED', 'decision_reason': 'post-run inspection; physics/time stopped'}
                    while viewer.is_running():
                        update_viewer(viewer, sim.data.qpos[:3], map_view, inspection, trail, controls,
                                      latest_sample, sim.data.time, args.dose_budget, args.rate_limit)
                        viewer.sync()
                        time.sleep(.05)
            except KeyboardInterrupt:
                pass
            finally:
                viewer.close()
    if (error or true_dose > args.dose_budget or (args.require_clean and not metrics['clean_return'])
            or (args.require_round_trip and not metrics['round_trip_completed'])
            or (args.require_circuit and not metrics.get('circuit_mission_completed'))):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
