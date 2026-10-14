"""Own the simulator, SLAM, Nav2, explorer and viewer processes as one session."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from .assets import ROOT, ensure_assets
from .graphics import rviz_command


def navigation_config(output):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('nav2_bringup'))
    config = yaml.safe_load((share/'params/nav2_params.yaml').read_text())
    def replace(item):
        if isinstance(item, dict):
            return {k: True if k == 'use_sim_time' else replace(v) for k, v in item.items()}
        if isinstance(item, list):
            return [replace(v) for v in item]
        return 'base_footprint' if item == 'base_link' else item
    config = replace(config)
    control = config['controller_server']['ros__parameters']
    control['controller_frequency'] = 20.
    control['min_y_velocity_threshold'] = .001
    control['FollowPath'] = dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=.3, lookahead_dist=.6, min_lookahead_dist=.3, max_lookahead_dist=.9,
        use_velocity_scaled_lookahead_dist=True, lookahead_time=1.5,
        rotate_to_heading_angular_vel=.4, rotate_to_heading_min_angle=.6,
        max_angular_accel=.6, use_rotate_to_heading=True,
        use_collision_detection=True, max_allowed_time_to_collision_up_to_carrot=1.,
        regulated_linear_scaling_min_speed=.08, transform_tolerance=.3)
    control['general_goal_checker']['xy_goal_tolerance'] = .3
    control['general_goal_checker']['yaw_goal_tolerance'] = .35
    control['progress_checker']['movement_time_allowance'] = 20.
    control['progress_checker']['required_movement_radius'] = .2
    config['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        params['robot_radius'] = .38
        params['resolution'] = .05
        params['inflation_layer']['inflation_radius'] = .6
        params['inflation_layer']['cost_scaling_factor'] = 4.
        params['plugins'] = ['obstacle_layer', 'inflation_layer'] if name == 'local_costmap' else ['static_layer', 'obstacle_layer', 'inflation_layer']
        params['obstacle_layer'] = dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True,
            observation_sources='scan', scan=dict(topic='/scan', data_type='LaserScan',
            clearing=True, marking=True, inf_is_valid=True, obstacle_max_range=7.5, raytrace_max_range=8.))
    smoother = config['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.35, .0, .5], min_velocity=[-.2, .0, -.5],
                    max_accel=[.5, .0, .6], max_decel=[-.5, .0, -.6])
    output.write_text(yaml.safe_dump(config))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('flat', 'room', 'office'), default='room')
    parser.add_argument('--people', type=int, choices=range(9), default=0)
    parser.add_argument('--duration', type=float, default=300.)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--explore', action='store_true')
    parser.add_argument('--vx', type=float, default=0.)
    parser.add_argument('--vy', type=float, default=0.)
    parser.add_argument('--yaw-rate', type=float, default=0.)
    args = parser.parse_args()
    ensure_assets()
    output = ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True)
    (output/'session.json').write_text(json.dumps(vars(args), indent=2)+'\n')
    children = []
    logs = []
    def start(name, command):
        stream = (output/(name+'.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
        return process
    status = 0
    try:
        start('interfaces', [sys.executable, '-m', 'dougie_sim.record', str(output)])
        sim_cmd = [sys.executable, '-m', 'dougie_sim.run', '--ros', '--world', args.world,
                   '--people', str(args.people), '--duration', str(args.duration), '--output', str(output/'simulation.json'),
                   '--vx', str(args.vx), '--vy', str(args.vy), '--yaw-rate', str(args.yaw_rate)]
        # RViz owns rendering; the physics process never creates a GL viewer.
        sim_cmd.append('--headless')
        sim = start('simulation', sim_cmd)
        if args.explore:
            nav_config = output/'nav2.yaml'
            navigation_config(nav_config)
            start('slam', ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=true',
                          'slam_params_file:='+str(ROOT/'config/dougie_slam.yaml')])
            start('navigation', ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                  'use_sim_time:=true', 'autostart:=true', 'use_composition:=False', 'params_file:='+str(nav_config)])
            start('exploration', [sys.executable, '-m', 'dougie_sim.explore', '--ros-args', '-p', 'use_sim_time:=true'])
        if not args.headless:
            rviz_config = 'dougie.rviz' if args.explore else 'dougie_robot.rviz'
            start('rviz', rviz_command(ROOT/'config'/rviz_config, output/'rviz_graphics.json'))
        print(f'Dougie running; logs and metrics: {output}', flush=True)
        deadline = time.monotonic()+args.duration*2+60.
        while sim.poll() is None:
            failed = [(name, p.returncode) for name, p in children
                      if p.poll() is not None and (name != 'rviz' or p.returncode)]
            if failed:
                raise RuntimeError(f'Critical process exited: {failed}; inspect {output}')
            if time.monotonic() > deadline:
                raise RuntimeError('session wall-clock deadline exceeded')
            time.sleep(.2)
        status = sim.returncode
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        status = 1
    finally:
        for _, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in reversed(children):
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in logs:
            stream.close()
        print(f'Dougie session ended (exit {status}); evidence: {output}', flush=True)
    raise SystemExit(status)


if __name__ == '__main__':
    main()
