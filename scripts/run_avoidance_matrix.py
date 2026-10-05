#!/usr/bin/env python3
"""Run a serial, reproducible multi-world onboard avoidance experiment."""
import argparse
import json
import signal
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from avoidance_worlds import ROOT, SCENES, WORLDS, WORLD_VERSION, generate
from demo_suite import case_catalog, file_sha256, git_output, stream_process, write_json

DEFAULT_OUTPUT = ROOT / 'ros2_ws/src/dynamic_obstacle_benchmarks/results/avoidance_matrix'


def publish_dataset(path, dataset):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(dataset, indent=2) + '\n')
    temporary.replace(path)


def command_for(directory, world, seed, people, goals, config, fast_start=True, synchronized_start=False):
    command = list(case_catalog(directory, seed, True, fast_start, goals, onboard_people=people)['onboard-waypoints'].command)
    for flag, value in {'--scene': f'scene_29dof_avoidance_{world}.xml',
                        '--environment-name': world, '--config': str(config),
                        '--result-dir': str(directory)}.items():
        command[command.index(flag)+1] = value
    if synchronized_start:
        command = ['mujoco_pedestrian_start_gate:=true' if value == 'mujoco_pedestrian_start_gate:=false'
                   else 'mujoco_pedestrian_start_delay:=0.0' if value == 'mujoco_pedestrian_start_delay:=10.0'
                   else value for value in command]
    return tuple(command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worlds', nargs='+', choices=WORLDS, default=list(WORLDS))
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 43, 44])
    parser.add_argument('--people', type=int, choices=range(1, 9), default=4)
    parser.add_argument('--goals', type=int, default=3)
    parser.add_argument('--trial-seconds', type=int, default=90)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    parser.add_argument('--publish', type=Path, default=ROOT / 'docs/results/avoidance_dataset.json')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--fast-start', action='store_true', help='Reuse an existing verification build instead of refreshing it for the first trial')
    parser.add_argument('--synchronized-start', action='store_true', help='Release the independent crowd on the first accepted navigation goal; require simulator acknowledgement')
    parser.add_argument('--label', default='working-tree', help='Human-readable controller revision label')
    args = parser.parse_args()
    def interrupted_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted_signal)
    if args.goals < 1 or args.trial_seconds < 30:
        parser.error('goals must be positive and trial-seconds at least 30')
    if len(set(args.worlds)) != len(args.worlds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('worlds and seeds must not contain duplicates')
    output = args.output.resolve()
    # The existing launch wrapper mounts only workspace result paths.
    try:
        output.relative_to(ROOT / 'ros2_ws')
    except ValueError:
        parser.error('--output must be inside ros2_ws so the container can access it')
    if output.exists():
        parser.error('output directory exists; use a fresh directory to preserve prior runs')
    generate()
    output.mkdir(parents=True)
    records, attempts = [], []
    config = output / 'benchmark.yaml'
    template = ROOT / 'ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_natural_crowd_benchmark.yaml'
    config.write_text(template.read_text().replace('total_timeout_s: 360.0', f'total_timeout_s: {args.trial_seconds}.0'))
    if args.synchronized_start:
        config.write_text(config.read_text().replace('synchronized_crowd_start: false', 'synchronized_crowd_start: true'))
    (output / 'person_scenarios.yaml').write_bytes((template.parent / 'person_scenarios.yaml').read_bytes())
    container_config = Path('/workspace') / config.relative_to(ROOT)
    image = subprocess.run(['docker', 'image', 'inspect', '--format', '{{.Id}}',
                            'unitree-g1-ros2-mujoco:humble'], capture_output=True, text=True)
    manifest = dict(schema_version=1, world_version=WORLD_VERSION, created=datetime.now(timezone.utc).isoformat(),
                    controller_label=args.label,
                    synchronized_start=args.synchronized_start,
                    docker_image_id=image.stdout.strip() if image.returncode == 0 else None,
                    source_files={str(path.relative_to(ROOT)): file_sha256(path) for directory in (
                        ROOT / 'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge',
                        ROOT / 'ros2_ws/src/unitree_g1_sim_bringup/config',
                        ROOT / 'ros2_ws/src/unitree_social_costmap/src')
                        for path in sorted(directory.iterdir()) if path.suffix in ('.py', '.yaml', '.cpp', '.xml')},
                    git_commit=git_output('rev-parse', 'HEAD'), git_status=git_output('status', '--short'),
                    source_patch_sha256=file_sha256(ROOT / 'patches/unitree_mujoco.patch'),
                    policy_sha256=file_sha256(ROOT / 'vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx'),
                    config_sha256=file_sha256(config), presets_sha256=file_sha256(output / 'person_scenarios.yaml'), people=args.people, goals=args.goals,
                    trial_seconds=args.trial_seconds, worlds={w: WORLDS[w] for w in args.worlds},
                    seeds=args.seeds, dry_run=args.dry_run, attempts=attempts)
    for world in args.worlds:
        for seed in args.seeds:
            run_id = f'{world}-people-{args.people}-seed-{seed}'
            directory = output / run_id
            directory.mkdir()
            command = command_for(directory, world, seed, args.people, args.goals, container_config,
                                  fast_start=args.fast_start or bool(attempts), synchronized_start=args.synchronized_start)
            attempt = dict(id=run_id, world=world, seed=seed, command=list(command),
                           scene_sha256=file_sha256(SCENES / f'scene_29dof_avoidance_{world}.xml'), status='planned')
            attempts.append(attempt)
            write_json(output / 'manifest.json', manifest)
            if args.dry_run:
                print(run_id)
                continue
            print(f'Running {run_id}', flush=True)
            attempt['status'] = 'running'
            write_json(output / 'manifest.json', manifest)
            interrupted = False
            try:
                with (directory / 'console.log').open('w') as log:
                    code, timeout = stream_process(command, log, args.trial_seconds + 180)
            except KeyboardInterrupt:
                code, timeout, interrupted = 130, False, True
            attempt.update(return_code=code, timed_out=timeout, status='interrupted' if interrupted else 'finished')
            artifacts = sorted(directory.glob('person_avoidance_*.json'))
            if len(artifacts) == 1:
                artifact = artifacts[0]
                result = json.loads(artifact.read_text())
                source, digest = str(artifact.relative_to(ROOT)), file_sha256(artifact)
            else:
                result = dict(seed=seed, backend='mujoco', onboard_only=True,
                              people_count=args.people, goals_requested=args.goals,
                              valid_trial=False, success=False, invalid_reason='missing_result',
                              failure_reason='timeout' if timeout else 'infrastructure_failure')
                source, digest = None, None
            if interrupted:
                result.update(valid_trial=False, success=False, invalid_reason='interrupted', failure_reason='interrupted')
            records.append(dict(id=run_id, role='scored', world=world,
                                world_size_m=WORLDS[world]['size'], complexity=WORLDS[world]['complexity'],
                                source_artifact=source, source_sha256=digest, result=result,
                                execution=attempt.copy()))
            dataset = dict(schema_version=1, scope='Multi-world onboard avoidance; invalid attempts retained',
                           provenance=manifest, records=records)
            write_json(output / 'dataset.json', dataset)
            publish_dataset(args.publish, dataset)
            write_json(output / 'manifest.json', manifest)
            if interrupted:
                raise SystemExit(130)
            if source is None:
                print('No benchmark artifact was produced; stopping after recording the infrastructure failure.')
                return 1
    print(f'Manifest: {output / "manifest.json"}')
    if not args.dry_run:
        print(f'Dataset: {args.publish} ({len(records)} attempted trials)')
        passed = sum(record['result'].get('success') is True and
                     record['result'].get('valid_trial') is True and
                     record['execution']['return_code'] == 0 for record in records)
        print(f'Strict acceptance: {passed}/{len(records)} trials passed')
        return 0 if passed == len(records) else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
