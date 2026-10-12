"""Dose-constrained 3-D radiation mapping mission in native Go2 physics."""
import argparse
from collections import Counter
import hashlib
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import subprocess
from pathlib import Path
import time

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

from .assets import ROOT, ensure_assets
from .dose_planner import DoseExplorer
from .clustered_planner import ClusteredDoseExplorer
from .demo_overlay import DemoControls, update_viewer
from .radiation import Detector, RadiationField
from .reactor import radiation_description
from .source_config import DEFAULT_SOURCE_CONFIG
from .simulation import Simulation, rotation
from .terrain import SurfaceMap


def support_pose(sim):
    feet = [sim.data.body(name+'_foot').xpos[2]-.022 for name in ('FR', 'FL', 'RR', 'RL')]
    return np.array([*sim.data.qpos[:2], np.mean(sorted(feet)[:2])])


def follow(path, position, yaw):
    while path and math.dist(position, path[0]) < .22:
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
    parser.add_argument('--gp-capacity', type=int, default=128)
    parser.add_argument('--gp-likelihood', choices=('poisson', 'gaussian'), default='poisson')
    parser.add_argument('--planner', choices=('baseline', 'clustered', 'clustered-no-commit'), default='clustered')
    parser.add_argument('--terrain-profile', choices=('conservative', 'experimental'), default='conservative',
                        help='conservative: 0.4 m clearance and 2 cm edge steps; experimental: original 0.3 m / 7.5 cm')
    parser.add_argument('--camera', choices=('follow', 'overview'), default='follow')
    parser.add_argument('--map-layer', choices=('mean', 'uncertainty'), default='mean')
    parser.add_argument('--fixed-plan-delay', type=float, help='headless fast benchmark: fixed simulated planner latency in seconds')
    parser.add_argument('--exit-on-complete', action='store_true', help='close the viewer immediately rather than inspect the paused final map')
    parser.add_argument('--record-demo', action='store_true', help='headless offscreen GIF of the actual run; use MUJOCO_GL=egl')
    parser.add_argument('--source-profile', default='hotspots-and-leaks', help='baseline, hotspots, leaks, or hotspots-and-leaks')
    parser.add_argument('--source-config', type=Path, help='point-source JSON; defaults to config/dougie_radiation_sources.json')
    parser.add_argument('--show-sources', action='store_true', help='diagnostic viewer markers: red hotspots, amber leaks')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--visualizer', choices=('rviz', 'mujoco'), default='rviz',
                        help='interactive renderer (RViz runs independently of physics)')
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0 or not 8 <= args.gp_capacity <= 512:
        parser.error('invalid duration or GP capacity (8..512)')
    if args.fast and (not args.headless or args.ros):
        parser.error('--fast requires --headless without --ros')
    if args.record_demo and not args.headless:
        parser.error('--record-demo requires --headless')
    if args.fixed_plan_delay is not None and (not args.fast or not math.isfinite(args.fixed_plan_delay) or not 0. <= args.fixed_plan_delay <= 1.):
        parser.error('--fixed-plan-delay requires --fast and a finite value in [0, 1]')
    config_path = args.source_config or DEFAULT_SOURCE_CONFIG
    if not args.headless and args.visualizer == 'rviz':
        args.ros = True
    try:
        description = radiation_description(args.source_profile, config_path)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    ensure_assets()
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('radiation_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=True)
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted((ROOT/'dougie_sim').glob('*.py'))}
    source_hashes['config/dougie_assets.json'] = hashlib.sha256((ROOT/'config/dougie_assets.json').read_bytes()).hexdigest()
    source_hashes['radiation_source_config'] = hashlib.sha256(config_path.read_bytes()).hexdigest()
    resolved_args = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    (output/'source_scenario.json').write_text(json.dumps(dict(profile=args.source_profile, **description), indent=2)+'\n')
    (output/'run_config.json').write_text(json.dumps({**resolved_args, 'output': str(output),
        'source_sha256': source_hashes}, indent=2)+'\n')
    sim = Simulation('reactor', radiation_sources=description)
    field = RadiationField.from_mujoco(sim.model, sim.data)
    detector = Detector(args.seed)
    parameters = (args.dose_budget, args.rate_limit, args.gp_capacity, args.duration, args.gp_likelihood)
    explorer = DoseExplorer(*parameters) if args.planner == 'baseline' else ClusteredDoseExplorer(*parameters, commitment=args.planner == 'clustered')
    if args.terrain_profile == 'conservative':
        explorer.terrain = SurfaceMap(radius=.4, max_step=.02, max_local_relief=.02)
    controls = DemoControls(args.camera, args.map_layer, args.show_sources)
    # Limit BLAS oversubscription so GP work cannot consume every control core.
    limits = threadpool_limits(limits=1)
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='radiation-gp')
    future = None
    samples = []
    path, state, map_view = [], 'initial_survey', None
    latest_sample = None
    true_dose = 0.
    error = None
    live_metrics = dict(state='initial_survey', planner=args.planner)
    trail = []
    distance_travelled = 0.
    previous_position = sim.data.qpos[:3].copy()
    last_target = None
    target_switches = 0
    foot_geoms = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    robot_geoms = {i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] != 0}
    nonfoot_impulse, nonfoot_peak = 0., 0.
    nonfoot_contacts = Counter()
    contact_force = np.empty(6)
    recorder = None
    if args.record_demo:
        from .demo_recording import DemoRecorder
        recorder = DemoRecorder(sim.model, output, args.duration)
    bridge = viewer = radiation_pub = cloud_pub = status_pub = None
    visualization = rviz = rviz_log = None
    if args.ros:
        from .ros_bridge import Bridge
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        # This mission uses 3-D LiDAR, not the navigation stack's extra 2-D scan.
        bridge = Bridge(sim, publish_scan=False)
        from .rviz_visualization import Visualization
        visualization = Visualization(bridge, args.show_sources, args.map_layer)
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
    minimum_sensor_z = math.inf
    maximum_sensor_z = -math.inf
    last_path_position = support_pose(sim)
    last_progress_stamp = 0.
    trace = (output/'trace.jsonl').open('w')
    measurements = (output/'measurements.jsonl').open('w')

    def plan(batch, cloud, supports, origin, position, stamp, stalled):
        for sample in batch:
            explorer.observe_radiation(sample)
        explorer.observe_geometry(cloud, supports, origin)
        if stalled and explorer.target is not None:
            if hasattr(explorer, 'report_stall'):
                explorer.report_stall(position, stamp)
            else:
                explorer.failed.append((explorer.target.copy(), stamp))
        explorer.replan(position, stamp)
        # Show/export a real XYZ volume around observed surface locations.
        nodes = np.asarray(list(explorer.terrain.nodes.values())).reshape(-1, 3)
        if len(nodes):
            nodes = nodes[::max(1, math.ceil(len(nodes)/150))]
            query = np.concatenate([nodes+[0., 0., z] for z in (.25, .75, 1.5)])
            mean, std, upper = explorer.gp.predict(query)
            volume = (query, mean, std, upper)
        else:
            volume = None
        metrics = explorer.metrics()
        metrics.update(planner=args.planner, selected_target=None if explorer.target is None else explorer.target.tolist(),
                       planned_path=[p.tolist() for p in explorer.path])
        return [p.copy() for p in explorer.path], explorer.state, metrics, volume

    print(f'Dougie radiation mission: {output}\nSource profile: {args.source_profile} ({len(field.sources)} point sources)', flush=True)
    try:
        if not args.headless and args.visualizer == 'rviz':
            rviz_log = (output/'rviz.log').open('w')
            rviz = subprocess.Popen(['rviz2', '-d', str(ROOT/'config/dougie_radiation.rviz'),
                                     '--ros-args', '-p', 'use_sim_time:=true'],
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
            position = support_pose(sim)
            if not trail or math.dist(position, trail[-1]) > .08:
                trail.append(position.tolist())
                if len(trail) > 2000:
                    trail = trail[::2]
            sensor = sim.data.qpos[:3]+[0., 0., .12]
            minimum_sensor_z = min(minimum_sensor_z, float(sensor[2]))
            maximum_sensor_z = max(maximum_sensor_z, float(sensor[2]))
            rate = field.rate(sensor)
            true_dose += rate*sim.model.opt.timestep/3600.
            sample = detector.integrate(sensor, rate, sim.model.opt.timestep, stamp+sim.model.opt.timestep)
            if sample is not None:
                latest_sample = sample
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
                live_metrics = metrics
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
            if future is None and samples and stamp-last_plan >= 2.:
                lidar_started = time.perf_counter()
                cloud, supports = sim.lidar3d()
                lidar_times.append(time.perf_counter()-lidar_started)
                future = pool.submit(plan, samples, cloud, supports, sensor.copy(), position.copy(), stamp, stalled)
                samples = []
                last_plan = stamp
                if stalled:
                    last_progress_stamp = stamp
            R = rotation(sim.data.qpos[3:7])
            yaw = math.atan2(R[1, 0], R[0, 0])
            command = follow(path, position, yaw)
            if latest_sample is None or stamp-latest_sample.stamp_s > 1.5 or stamp-accepted_plan_stamp > 4.:
                command[:] = 0.
            if latest_sample is not None and (latest_sample.rate_usv_h > args.rate_limit or latest_sample.dose_upper_usv > args.dose_budget*.8) and not state.startswith('return'):
                command[:] = 0.
            if bridge:
                bridge_started = time.perf_counter()
                bridge.tick()  # Publish sensing; the radiation executive owns motion.
                visualization.tick()
                bridge_times.append(time.perf_counter()-bridge_started)
            sim.step(command)
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
        pool.shutdown(wait=True, cancel_futures=True)
        if future is not None and future.done() and not future.cancelled():
            try:
                _, _, _, map_view = future.result()
            except Exception as exc:
                error = error or str(exc)
        for sample in samples:
            explorer.observe_radiation(sample)
        if recorder:
            recorder.capture(sim, map_view, {**live_metrics, 'state': state if error is None else 'FAILED: '+error},
                             trail, controls, latest_sample, args.dose_budget, args.rate_limit, force=True)
            recorder.close()
        trace.close()
        measurements.close()
        if map_view:
            query = map_view[0]
            mean, std, upper = explorer.gp.predict(query)
            np.savez_compressed(output/'radiation_map_3d.npz', xyz=query, mean_usv_h=mean, std_log_rate=std, upper_usv_h=upper)
        gp = explorer.gp
        np.savez_compressed(output/'gp_posterior.npz', Z=gp.Z, mean=gp.mean, covariance=gp.cov,
                            cholesky=gp.L, lengthscales=gp.lengthscales, variance=gp.variance,
                            prior_log_rate=gp.prior, likelihood=args.gp_likelihood)
        np.savez_compressed(output/'observed_geometry.npz',
            points=np.asarray(list(explorer.terrain.hits.values())).reshape(-1, 3),
            free_voxels=np.asarray(list(explorer.terrain.free_voxels)).reshape(-1, 3), voxel_size_m=.2)
        metrics = dict(**explorer.metrics(), simulation=sim.metrics(), end_state=state, error=error,
            source_profile=args.source_profile, radiation_source_count=len(field.sources),
            source_sha256=source_hashes,
            true_dose_usv=true_dose, measured_dose_usv=detector.dose,
            sampled_height_range_m=[minimum_sensor_z, maximum_sensor_z],
            mission_completed=bool(state == 'returned'),
            actual_budget_exceeded=bool(true_dose > args.dose_budget),
            wall_time_s=time.monotonic()-started, seed=args.seed,
            information_contract='planner receives range returns, ideal pose, and counting-detector samples only')
        metrics.update(planner=args.planner, path_length_m=distance_travelled, target_switches=target_switches,
                       terrain_profile=args.terrain_profile, nonfoot_contacts=dict(nonfoot_contacts),
                       nonfoot_impulse_ns=nonfoot_impulse, nonfoot_peak_force_n=nonfoot_peak,
                       clean_return=bool(state == 'returned' and error is None and true_dose <= args.dose_budget
                                         and nonfoot_impulse <= .1 and nonfoot_peak <= 25.))
        def timing(values):
            return dict(count=len(values), total_s=float(sum(values)),
                        p50_ms=float(np.percentile(values, 50)*1000) if values else 0.,
                        p95_ms=float(np.percentile(values, 95)*1000) if values else 0.,
                        p99_ms=float(np.percentile(values, 99)*1000) if values else 0.)
        metrics['performance'] = dict(
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
        print(json.dumps({k:v for k,v in metrics.items() if k not in ('events', 'source_sha256', 'clusters', 'planned_path', 'return_path')}, indent=2))
        if visualization:
            final_state = 'failed: '+error if error else state if state in ('returned', 'interrupted') else 'time limit reached'
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
    if error or true_dose > args.dose_budget:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
