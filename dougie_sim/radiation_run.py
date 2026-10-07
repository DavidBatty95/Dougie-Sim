"""Dose-constrained 3-D radiation mapping mission in native Go2 physics."""
import argparse
import hashlib
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

from .assets import ROOT, ensure_assets
from .dose_planner import DoseExplorer
from .radiation import Detector, RadiationField
from .reactor import radiation_description
from .simulation import Simulation, rotation


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
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0 or not 8 <= args.gp_capacity <= 512:
        parser.error('invalid duration or GP capacity (8..512)')
    if args.fast and (not args.headless or args.ros):
        parser.error('--fast requires --headless without --ros')
    ensure_assets()
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('radiation_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=True)
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted((ROOT/'dougie_sim').glob('*.py'))}
    source_hashes['config/dougie_assets.json'] = hashlib.sha256((ROOT/'config/dougie_assets.json').read_bytes()).hexdigest()
    (output/'run_config.json').write_text(json.dumps({**vars(args), 'output': str(output),
        'source_sha256': source_hashes}, indent=2)+'\n')
    sim = Simulation('reactor')
    field = RadiationField.from_mujoco(sim.model, sim.data)
    detector = Detector(args.seed)
    explorer = DoseExplorer(args.dose_budget, args.rate_limit, args.gp_capacity, args.duration, args.gp_likelihood)
    # Limit BLAS oversubscription so GP work cannot consume every control core.
    limits = threadpool_limits(limits=1)
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='radiation-gp')
    future = None
    samples = []
    path, state, map_view = [], 'initial_survey', None
    latest_sample = None
    true_dose = 0.
    error = None
    bridge = viewer = radiation_pub = cloud_pub = status_pub = None
    if args.ros:
        from .ros_bridge import Bridge
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        bridge = Bridge(sim)
        radiation_pub = bridge.node.create_publisher(String, '/radiation/measurement', 10)
        status_pub = bridge.node.create_publisher(String, '/radiation/status', 10)
        cloud_pub = bridge.node.create_publisher(PointCloud2, '/radiation/gp_map', 1)
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        viewer.cam.distance, viewer.cam.elevation = 6., -35.
    started = time.monotonic()
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
            explorer.failed.append((explorer.target.copy(), stamp))
        explorer.replan(position, stamp)
        # Show/export a real XYZ volume around observed surface locations.
        nodes = np.asarray(list(explorer.terrain.nodes.values())).reshape(-1, 3)
        if len(nodes):
            nodes = nodes[::max(1, len(nodes)//150)]
            query = np.concatenate([nodes+[0., 0., z] for z in (.25, .75, 1.5)])
            mean, std, upper = explorer.gp.predict(query)
            volume = (query, mean, std, upper)
        else:
            volume = None
        return [p.copy() for p in explorer.path], explorer.state, explorer.metrics(), volume

    print(f'Dougie radiation mission: {output}', flush=True)
    try:
        while sim.data.time < args.duration and (viewer is None or viewer.is_running()):
            stamp = float(sim.data.time)
            position = support_pose(sim)
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
            if future is not None and (future.done() or (args.fast and stamp-last_plan >= .5)):
                path, state, metrics, map_view = future.result()
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
                if state == 'returned':
                    break
            if math.dist(position, last_path_position) > .15:
                last_path_position, last_progress_stamp = position.copy(), stamp
            stalled = stamp-last_progress_stamp > 15.
            if future is None and samples and stamp-last_plan >= 2.:
                cloud, supports = sim.lidar3d()
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
                bridge.tick()  # Publish sensing; the radiation executive owns motion.
            sim.step(command)
            if viewer and sim.counter % 4 == 0:
                viewer.cam.lookat[:] = sim.data.qpos[:3]
                if map_view:
                    query, mean, _, _ = map_view
                    count = min(len(query), viewer.user_scn.maxgeom)
                    viewer.user_scn.ngeom = count
                    for i in range(count):
                        intensity = min(1., math.log1p(mean[i])/math.log1p(args.rate_limit))
                        color = np.array([intensity, 1.-intensity, .2, .3], dtype=np.float32)
                        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([.045]*3), query[i], np.eye(3).reshape(-1), color)
                viewer.sync()
            if not args.fast:
                time.sleep(max(0., started+sim.data.time-time.monotonic()))
    except KeyboardInterrupt:
        state = 'interrupted'
    except Exception as exc:
        error = str(exc)
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
        if future is not None and future.done() and not future.cancelled():
            try:
                _, _, _, map_view = future.result()
            except Exception as exc:
                error = error or str(exc)
        for sample in samples:
            explorer.observe_radiation(sample)
        if bridge:
            bridge.close()
        if viewer:
            viewer.close()
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
            source_sha256=source_hashes,
            true_dose_usv=true_dose, measured_dose_usv=detector.dose,
            sampled_height_range_m=[minimum_sensor_z, maximum_sensor_z],
            mission_completed=bool(state == 'returned'),
            actual_budget_exceeded=bool(true_dose > args.dose_budget),
            wall_time_s=time.monotonic()-started, seed=args.seed,
            information_contract='planner receives range returns, ideal pose, and counting-detector samples only')
        (output/'radiation_summary.json').write_text(json.dumps(metrics, indent=2)+'\n')
        # Truth is exported only for independent post-run evaluation.
        (output/'evaluator_field.json').write_text(json.dumps(dict(background_usv_h=field.background,
            sources=field.sources, shields=field.shields), indent=2)+'\n')
        limits.restore_original_limits()
        print(json.dumps({k:v for k,v in metrics.items() if k != 'events'}, indent=2))
    if error or true_dose > args.dose_budget:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
