#!/usr/bin/env python3
"""Single entry point for Dougie's isolated Docker simulation."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ['docker', 'compose', '--project-directory', str(ROOT), '-f', str(ROOT/'docker-compose.yml')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('setup', help='download verified Go2 assets and build Docker image')
    sub.add_parser('worlds', help='list Go2 scenes')
    sub.add_parser('test', help='run policy, physics, and interface tests')
    sub.add_parser('terrain-test', help='legacy 6 cm stair completion demonstration; use terrain-qualify for qualification')
    qualification = sub.add_parser('terrain-qualify', help='repeatable terrain qualification with contact and stability checks')
    qualification.add_argument('--suite', choices=('smoke', 'matrix', 'repeatability'), default='matrix')
    qualification.add_argument('--cases', nargs='+')
    qualification.add_argument('--seeds', nargs='+', type=int)
    qualification.add_argument('--workers', type=int, default=4)
    qualification.add_argument('--output')
    qualification.add_argument('--list', action='store_true')
    qualification.add_argument('--view', action='store_true')
    qualification.add_argument('--require-qualified', action='store_true')
    sub.add_parser('benchmark-gp', help='measure bounded 3-D GP accuracy and latency')
    radiation = sub.add_parser('radiation', help='3-D GP radiation mapping under a dose budget')
    radiation.add_argument('--duration', type=float, default=600.)
    radiation.add_argument('--dose-budget', type=float, default=2., help='micro-sieverts')
    radiation.add_argument('--rate-limit', type=float, default=100., help='micro-sieverts/hour')
    radiation.add_argument('--seed', type=int, default=1)
    radiation.add_argument('--gp-capacity', type=int, default=128)
    radiation.add_argument('--gp-likelihood', choices=('poisson', 'gaussian'), default='poisson')
    radiation.add_argument('--planner', choices=('baseline', 'clustered', 'clustered-no-commit'), default='clustered')
    radiation.add_argument('--terrain-profile', choices=('conservative', 'experimental'), default='conservative')
    radiation.add_argument('--camera', choices=('follow', 'overview'), default='follow')
    radiation.add_argument('--map-layer', choices=('mean', 'uncertainty'), default='mean')
    radiation.add_argument('--exit-on-complete', action='store_true')
    radiation.add_argument('--record-demo', action='store_true')
    radiation.add_argument('--source-profile', default='hotspots-and-leaks', help='baseline, hotspots, leaks, or hotspots-and-leaks')
    radiation.add_argument('--source-config', help='JSON source configuration inside this checkout')
    radiation.add_argument('--show-sources', action='store_true', help='show diagnostic hotspot/leak markers')
    radiation.add_argument('--headless', action='store_true')
    radiation.add_argument('--visualizer', choices=('rviz', 'mujoco'), default='rviz')
    radiation.add_argument('--fast', action='store_true', help='headless unpaced physics, without ROS')
    for name in ('demo', 'explore'):
        p = sub.add_parser(name)
        p.add_argument('--world', choices=('flat', 'room', 'office'), default='room' if name == 'demo' else 'office')
        p.add_argument('--people', type=int, choices=range(9), default=0)
        p.add_argument('--duration', type=float, default=300.)
        p.add_argument('--headless', action='store_true')
        if name == 'demo':
            p.add_argument('--vx', type=float, default=0.)
            p.add_argument('--vy', type=float, default=0.)
            p.add_argument('--yaw-rate', type=float, default=0.)
    sub.add_parser('shell', help='ROS-enabled shell for topic inspection and teleoperation')
    args = parser.parse_args()
    os.environ['DOUGIE_UID'] = str(os.getuid())
    os.environ['DOUGIE_GID'] = str(os.getgid())
    if args.command == 'worlds':
        print('flat     Open plane\nroom     12 x 12 m room\noffice   Partitions and furniture\nreactor  Shielded rooms, vessel, pipes, uneven floors, and inspection stairs\nstairs   Physical ascent/descent qualification course')
        return
    if args.command == 'setup':
        subprocess.run([sys.executable, '-m', 'dougie_sim.assets'], cwd=ROOT, check=True)
        subprocess.run([*COMPOSE, 'build'], cwd=ROOT, check=True)
        return
    if args.command == 'test':
        command = ['python3', '-m', 'pytest', '-q', 'tests/dougie']
    elif args.command == 'terrain-test':
        command = ['python3', 'scripts/probe_dougie_terrain.py']
    elif args.command == 'terrain-qualify':
        command = ['python3', '-m', 'dougie_sim.terrain_qualification', '--suite', args.suite, '--workers', str(args.workers)]
        for flag in ('cases', 'seeds'):
            value = getattr(args, flag)
            if value is not None:
                command += ['--'+flag, *map(str, value)]
        if args.output:
            command += ['--output', args.output]
        for flag in ('list', 'view', 'require_qualified'):
            if getattr(args, flag):
                command.append('--'+flag.replace('_', '-'))
    elif args.command == 'benchmark-gp':
        command = ['python3', 'scripts/benchmark_dougie_gp.py']
    elif args.command == 'shell':
        command = ['bash']
    elif args.command == 'radiation':
        command = ['python3', '-m', 'dougie_sim.radiation_run', '--duration', str(args.duration),
            '--dose-budget', str(args.dose_budget), '--rate-limit', str(args.rate_limit),
            '--seed', str(args.seed), '--gp-capacity', str(args.gp_capacity), '--gp-likelihood', args.gp_likelihood,
            '--source-profile', args.source_profile, '--planner', args.planner,
            '--camera', args.camera, '--map-layer', args.map_layer, '--terrain-profile', args.terrain_profile,
            '--visualizer', args.visualizer]
        if args.source_config:
            path = Path(args.source_config).expanduser()
            path = (path if path.is_absolute() else ROOT/path).resolve()
            try:
                relative = path.relative_to(ROOT)
            except ValueError:
                parser.error('--source-config must be inside the checkout mounted at /workspace')
            if not path.is_file():
                parser.error(f'missing source configuration: {path}')
            command += ['--source-config', str(Path('/workspace')/relative)]
        if args.show_sources:
            command.append('--show-sources')
        if args.exit_on_complete:
            command.append('--exit-on-complete')
        if args.record_demo:
            if not args.headless:
                parser.error('--record-demo requires --headless')
            command.append('--record-demo')
        if args.fast:
            if not args.headless:
                parser.error('--fast requires --headless')
            command.append('--fast')
        else:
            command.append('--ros')
        if args.headless:
            command.append('--headless')
        elif not os.environ.get('DISPLAY'):
            parser.error('DISPLAY is unset; use --headless')
    else:
        command = ['python3', '-m', 'dougie_sim.session', '--world', args.world,
                   '--people', str(args.people), '--duration', str(args.duration)]
        if args.headless:
            command.append('--headless')
        if args.command == 'explore':
            command.append('--explore')
        else:
            command += ['--vx', str(args.vx), '--vy', str(args.vy), '--yaw-rate', str(args.yaw_rate)]
        if not args.headless and not os.environ.get('DISPLAY'):
            parser.error('DISPLAY is unset; use --headless for a non-graphical session')
    flags = [] if args.command == 'shell' else ['-T']
    if args.command == 'radiation' and args.record_demo:
        flags += ['-e', 'MUJOCO_GL=egl']
    raise SystemExit(subprocess.call([*COMPOSE, 'run', '--rm', *flags, 'dougie-sim', *command], cwd=ROOT))


if __name__ == '__main__':
    main()
