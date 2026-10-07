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
    sub.add_parser('terrain-test', help='physical 6 cm stair ascent and descent qualification')
    sub.add_parser('benchmark-gp', help='measure bounded 3-D GP accuracy and latency')
    radiation = sub.add_parser('radiation', help='3-D GP radiation mapping under a dose budget')
    radiation.add_argument('--duration', type=float, default=600.)
    radiation.add_argument('--dose-budget', type=float, default=2., help='micro-sieverts')
    radiation.add_argument('--rate-limit', type=float, default=100., help='micro-sieverts/hour')
    radiation.add_argument('--seed', type=int, default=1)
    radiation.add_argument('--gp-capacity', type=int, default=128)
    radiation.add_argument('--gp-likelihood', choices=('poisson', 'gaussian'), default='poisson')
    radiation.add_argument('--headless', action='store_true')
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
    elif args.command == 'benchmark-gp':
        command = ['python3', 'scripts/benchmark_dougie_gp.py']
    elif args.command == 'shell':
        command = ['bash']
    elif args.command == 'radiation':
        command = ['python3', '-m', 'dougie_sim.radiation_run', '--duration', str(args.duration),
            '--dose-budget', str(args.dose_budget), '--rate-limit', str(args.rate_limit),
            '--seed', str(args.seed), '--gp-capacity', str(args.gp_capacity), '--gp-likelihood', args.gp_likelihood]
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
    raise SystemExit(subprocess.call([*COMPOSE, 'run', '--rm', *flags, 'dougie-sim', *command], cwd=ROOT))


if __name__ == '__main__':
    main()
