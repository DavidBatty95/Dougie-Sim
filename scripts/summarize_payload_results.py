#!/usr/bin/env python3
"""Archive payload-policy comparisons and every recorded native candidate trial."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read_evidence(path):
    raw = path.read_bytes()
    return dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(raw).hexdigest(),data=json.loads(raw))


def summarize(training):
    policy = training/'cycle_01/policy.onnx'
    policy_hash = hashlib.sha256(policy.read_bytes()).hexdigest()
    evaluations = {}
    for name,directory in (('original','baseline_original_eval71'),('warm_start','baseline_zero_eval71'),
                           ('trained','cycle_01_eval71_retry1')):
        record = read_evidence(training/directory/'evaluation.json')
        if {item['profile'] for item in record['data']} != {'unloaded','held_empty','loaded','heavy'}:
            raise ValueError(f'Incomplete four-profile evaluation: {directory}')
        evaluations[name] = record
    native = []
    other_attempts = 0
    results = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'
    for path in sorted(results.glob('*/result.json')):
        manifest_path = path.parent/'manifest.json'
        if not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text())
        if policy_hash not in manifest.get('source_sha256',{}).values():
            other_attempts += 1
            continue
        result = read_evidence(path)
        events = result['data'].get('events',[])
        completion = next((e['elapsed_s'] for e in reversed(events) if e['phase']=='complete'),None)
        native.append(dict(result=result,manifest=read_evidence(manifest_path),completion_seconds=completion))
    latest = native[-2:]
    repeated = (len(latest)==2 and all(item['result']['data']['success'] for item in latest)
                and latest[0]['manifest']['data']['source_sha256']==latest[1]['manifest']['data']['source_sha256'])
    return dict(role='experimental_payload_policy_evidence',training_manifest=read_evidence(training/'cycle_01/manifest.json'),
                policy_path=str(policy.relative_to(ROOT)),policy_sha256=policy_hash,evaluations=evaluations,
                native_candidate_attempts=native,native_candidate_successes=sum(item['result']['data']['success'] for item in native),
                other_recorded_transport_attempts=other_attempts,two_latest_unchanged_runs_passed=repeated,
                scope='Zero-person transport fixture; not crowded-navigation or comprehensive static-contact acceptance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training',type=Path,default=ROOT/'vendor/mjlab/logs/payload_training')
    parser.add_argument('--output',type=Path,default=ROOT/'docs/results/payload_training_cycle_01.json')
    parser.add_argument('--archive-container-logs',action='store_true')
    args = parser.parse_args()
    if args.archive_container_logs:
        for container,destination in (('geoff-payload-cycle-01',args.training/'cycle_01/console'),
                                      ('geoff-payload-baselines',args.training/'baseline_console')):
            completed = subprocess.run(['docker','logs',container],capture_output=True,text=True,check=True)
            destination.with_suffix('.stdout.log').write_text(completed.stdout)
            destination.with_suffix('.stderr.log').write_text(completed.stderr)
    report = summarize(args.training.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),policy_sha256=report['policy_sha256'],
        native_attempts=len(report['native_candidate_attempts']),native_successes=report['native_candidate_successes'],
        two_latest_unchanged_runs_passed=report['two_latest_unchanged_runs_passed'],
        latest=[dict(path=item['result']['path'],seconds=item['completion_seconds'])
                for item in report['native_candidate_attempts'][-2:]]),indent=2))


if __name__=='__main__':
    main()
