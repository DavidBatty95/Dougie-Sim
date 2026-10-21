#!/usr/bin/env python3
"""Single entry point for Dougie's isolated Docker simulation."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dougie_sim.graphics import MODES, compose_graphics, select_graphics
COMPOSE = ['docker', 'compose', '--project-directory', str(ROOT), '-f', str(ROOT/'docker-compose.yml')]


def checkout_file(parser, value):
    path = Path(value).expanduser()
    path = (path if path.is_absolute() else ROOT/path).resolve()
    try:
        relative = path.relative_to(ROOT)
    except ValueError:
        parser.error('configuration must be inside the checkout mounted at /workspace')
    if not path.is_file():
        parser.error(f'missing configuration: {path}')
    return str(Path('/workspace')/relative)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('setup', help='download verified Go2 assets and build Docker image')
    sub.add_parser('training-setup', help='install isolated optional CUDA Go2 training environment')
    training = sub.add_parser('train', help='GPU PPO foundation task; does not replace the deployed policy')
    training.add_argument('--num-envs', type=int, default=64)
    training.add_argument('--iterations', type=int, default=10)
    training.add_argument('--terrain', choices=('flat', 'small-stairs', 'stairs', 'inclines'), default='flat')
    training.add_argument('--perceptive', action='store_true')
    training.add_argument('--seed', type=int, default=1)
    training.add_argument('--smoke', action='store_true')
    training.add_argument('--initialize-baseline', action='store_true')
    training.add_argument('--residual-scale', type=float, default=0.)
    training.add_argument('--quiet', action='store_true')
    training.add_argument('--resume')
    training.add_argument('--export-only', action='store_true')
    training.add_argument('--output')
    sub.add_parser('worlds', help='list Go2 scenes')
    graphics = sub.add_parser('graphics-check', help='report the actual container OpenGL renderer')
    graphics.add_argument('--graphics', choices=MODES, default=os.environ.get('DOUGIE_GRAPHICS', 'auto'))
    graphics.add_argument('--require-hardware', action='store_true')
    benchmark = sub.add_parser('benchmark-rviz', help='measure actual RViz buffer-swap FPS with the full Go2 model')
    benchmark.add_argument('--graphics', choices=MODES, default=os.environ.get('DOUGIE_GRAPHICS', 'auto'))
    benchmark.add_argument('--world', choices=('room', 'reactor', 'facility'), default='facility')
    benchmark.add_argument('--warmup', type=float, default=8.)
    benchmark.add_argument('--duration', type=float, default=20.)
    sub.add_parser('test', help='run policy, physics, and interface tests')
    sub.add_parser('terrain-test', help='legacy 6 cm stair completion demonstration; use terrain-qualify for qualification')
    qualification = sub.add_parser('terrain-qualify', help='repeatable terrain qualification with contact and stability checks')
    qualification.add_argument('--suite', choices=('smoke', 'matrix', 'repeatability'), default='matrix')
    qualification.add_argument('--cases', nargs='+')
    qualification.add_argument('--seeds', nargs='+', type=int)
    qualification.add_argument('--workers', type=int, default=4)
    qualification.add_argument('--output')
    qualification.add_argument('--policy-config')
    qualification.add_argument('--list', action='store_true')
    qualification.add_argument('--view', action='store_true')
    qualification.add_argument('--graphics', choices=MODES, default=os.environ.get('DOUGIE_GRAPHICS', 'auto'))
    qualification.add_argument('--require-qualified', action='store_true')
    sub.add_parser('benchmark-gp', help='measure bounded 3-D GP accuracy and latency')
    radiation = sub.add_parser('radiation', help='3-D GP radiation mapping under a dose budget')
    radiation.add_argument('--duration', type=float, default=600.)
    radiation.add_argument('--dose-budget', type=float, default=2., help='micro-sieverts')
    radiation.add_argument('--rate-limit', type=float, default=100., help='micro-sieverts/hour')
    radiation.add_argument('--seed', type=int, default=1)
    radiation.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='reactor')
    radiation.add_argument('--layout-seed', type=int, default=0)
    radiation.add_argument('--landmark-count', type=int, default=0)
    radiation.add_argument('--landmark-seed', type=int, default=0)
    radiation.add_argument('--mission-objective', choices=('survey', 'inspect'), default='survey')
    radiation.add_argument('--region-scheduling', choices=('local', 'hierarchical'), default='hierarchical')
    radiation.add_argument('--localization', choices=('fused', 'ground-truth'), default='fused')
    radiation.add_argument('--record-view', choices=('field', 'routes'), default='field')
    radiation.add_argument('--sensor-seed', type=int, default=0)
    radiation.add_argument('--navigation-profile', choices=('robust', 'legacy'), default='robust')
    radiation.add_argument('--policy-config', help='policy manifest inside this checkout')
    radiation.add_argument('--fixed-plan-delay', type=float, help='headless fast reproducible planning schedule')
    radiation.add_argument('--require-clean', action='store_true')
    radiation.add_argument('--require-round-trip', action='store_true')
    radiation.add_argument('--gp-capacity', type=int, default=128)
    radiation.add_argument('--gp-mode', choices=('global', 'spatial', 'geometry'), default='geometry')
    radiation.add_argument('--detector-likelihood', choices=('window', 'point'), default='window')
    radiation.add_argument('--gp-expert-capacity', type=int, default=32)
    radiation.add_argument('--gp-max-experts', type=int, default=128)
    radiation.add_argument('--gp-cell-size', nargs=3, type=float, default=(4., 4., 1.5), metavar=('X', 'Y', 'Z'))
    radiation.add_argument('--path-mode', choices=('grid', 'smooth'), default='smooth')
    radiation.add_argument('--gp-likelihood', choices=('poisson', 'gaussian'), default='poisson')
    radiation.add_argument('--planner', choices=('baseline', 'clustered', 'clustered-no-commit'), default='clustered')
    radiation.add_argument('--terrain-profile', choices=('conservative', 'experimental', 'layered'))
    radiation.add_argument('--camera', choices=('follow', 'overview'), default='follow')
    radiation.add_argument('--map-layer', choices=('mean', 'uncertainty'), default='mean')
    radiation.add_argument('--field-display', choices=('detected', 'all'), default='detected')
    radiation.add_argument('--display-background', type=float)
    radiation.add_argument('--exit-on-complete', action='store_true')
    radiation.add_argument('--record-demo', action='store_true')
    radiation.add_argument('--source-profile', help='defaults to the selected world scenario')
    radiation.add_argument('--source-config', help='JSON source configuration inside this checkout')
    radiation.add_argument('--show-sources', action='store_true', help='show diagnostic hotspot/leak markers')
    radiation.add_argument('--headless', action='store_true')
    radiation.add_argument('--visualizer', choices=('rviz', 'mujoco'), default='rviz')
    radiation.add_argument('--fast', action='store_true', help='headless unpaced physics, without ROS')
    radiation.add_argument('--graphics', choices=MODES, default=os.environ.get('DOUGIE_GRAPHICS', 'auto'))
    for name in ('demo', 'explore'):
        p = sub.add_parser(name)
        p.add_argument('--world', choices=('flat', 'room', 'office'), default='room' if name == 'demo' else 'office')
        p.add_argument('--people', type=int, choices=range(9), default=0)
        p.add_argument('--duration', type=float, default=300.)
        p.add_argument('--headless', action='store_true')
        p.add_argument('--graphics', choices=MODES, default=os.environ.get('DOUGIE_GRAPHICS', 'auto'))
        if name == 'demo':
            p.add_argument('--vx', type=float, default=0.)
            p.add_argument('--vy', type=float, default=0.)
            p.add_argument('--yaw-rate', type=float, default=0.)
    sub.add_parser('shell', help='ROS-enabled shell for topic inspection and teleoperation')
    args = parser.parse_args()
    os.environ['DOUGIE_UID'] = str(os.getuid())
    os.environ['DOUGIE_GID'] = str(os.getgid())
    if args.command == 'training-setup':
        raise SystemExit(subprocess.call([sys.executable, str(ROOT/'scripts/setup_dougie_training.py')], cwd=ROOT))
    if args.command == 'train':
        python = ROOT/'.dougie/training/bin/python'
        if not python.exists():
            parser.error('run ./dougie training-setup first')
        command = [str(python), '-u', '-m', 'dougie_sim.training']
        for flag in ('num_envs', 'iterations', 'terrain', 'seed', 'resume', 'output', 'residual_scale'):
            value = getattr(args, flag)
            if value is not None:
                command += ['--'+flag.replace('_', '-'), str(value)]
        for flag in ('smoke', 'initialize_baseline', 'quiet', 'perceptive', 'export_only'):
            if getattr(args, flag):
                command += ['--'+flag.replace('_', '-')]
        raise SystemExit(subprocess.call(command, cwd=ROOT))
    if args.command == 'worlds':
        print('flat     Open plane\nroom     12 x 12 m room\noffice   Partitions and furniture\nreactor  Shielded rooms, vessel, pipes, uneven floors, and inspection stairs\nstairs   Physical ascent/descent qualification course\nmezzanine Two ramps and an elevated gallery (radiation --world mezzanine)\nfacility Full-height two-storey rooms, switchback ramps and stair challenge')
        return
    if args.command == 'setup':
        subprocess.run([sys.executable, '-m', 'dougie_sim.assets'], cwd=ROOT, check=True)
        subprocess.run([*COMPOSE, 'build'], cwd=ROOT, check=True)
        return
    if args.command == 'graphics-check':
        command = ['python3', '-m', 'dougie_sim.graphics']
        if args.require_hardware:
            command.append('--require-hardware')
    elif args.command == 'benchmark-rviz':
        command = ['python3', 'scripts/benchmark_dougie_rviz.py', '--world', args.world,
                   '--warmup', str(args.warmup), '--duration', str(args.duration)]
    elif args.command == 'test':
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
        if args.policy_config:
            command += ['--policy-config', checkout_file(parser, args.policy_config)]
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
            '--world', args.world, '--layout-seed', str(args.layout_seed), '--planner', args.planner,
            '--landmark-count', str(args.landmark_count), '--landmark-seed', str(args.landmark_seed),
            '--mission-objective', args.mission_objective,
            '--region-scheduling', args.region_scheduling,
            '--localization', args.localization, '--sensor-seed', str(args.sensor_seed),
            '--navigation-profile', args.navigation_profile,
            '--detector-likelihood', args.detector_likelihood,
            '--record-view', args.record_view,
            '--camera', args.camera, '--map-layer', args.map_layer,
            '--field-display', args.field_display,
            '--visualizer', args.visualizer]
        command += ['--gp-mode', args.gp_mode, '--gp-expert-capacity', str(args.gp_expert_capacity),
                    '--gp-max-experts', str(args.gp_max_experts), '--gp-cell-size', *map(str, args.gp_cell_size),
                    '--path-mode', args.path_mode]
        for flag in ('source_profile', 'terrain_profile', 'fixed_plan_delay'):
            value = getattr(args, flag)
            if value is not None:
                command += ['--'+flag.replace('_', '-'), str(value)]
        if args.display_background is not None:
            command += ['--display-background', str(args.display_background)]
        if args.policy_config:
            command += ['--policy-config', checkout_file(parser, args.policy_config)]
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
        if args.require_clean:
            command.append('--require-clean')
        if args.require_round_trip:
            command.append('--require-round-trip')
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
    graphics_needed = (args.command in ('graphics-check', 'benchmark-rviz') or
        (args.command in ('demo', 'explore', 'radiation') and not args.headless) or
        (args.command == 'radiation' and args.record_demo) or
        (args.command == 'terrain-qualify' and args.view))
    try:
        mode = select_graphics(getattr(args, 'graphics', 'auto'), graphics_needed)
        overlays, graphics_flags = compose_graphics(ROOT, mode)
    except ValueError as exc:
        parser.error(str(exc))
    if graphics_needed:
        print(f'Dougie graphics: {mode}', flush=True)
    flags += graphics_flags
    if args.command == 'radiation' and args.record_demo:
        flags += ['-e', 'MUJOCO_GL=egl']
    raise SystemExit(subprocess.call([*COMPOSE, *overlays, 'run', '--rm', *flags, 'dougie-sim', *command], cwd=ROOT))


if __name__ == '__main__':
    main()
