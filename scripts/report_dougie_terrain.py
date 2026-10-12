#!/usr/bin/env python3
"""Validate complete qualification cohorts and export portable evidence/report."""
import argparse
from collections import Counter
import hashlib
import html
import gzip
import io
import json
from pathlib import Path
import tarfile

import numpy as np

from dougie_sim.assets import ROOT
from dougie_sim.terrain_qualification import aggregate


def source_snapshot(directory, manifest):
    """Recover exact hashed sources, including the documented v1 harness.

    Reconstruction is accepted only if its bytes match the original manifest;
    historical manifests themselves are never modified.
    """
    path = directory/'source_snapshot.tar.gz'
    if path.exists():
        # Historical evidence remains reproducible when unrelated runtime code
        # evolves (for example, new radiation source scenarios).
        expected = manifest['source_sha256']
        with tarfile.open(path, 'r:gz') as archive:
            names = archive.getnames()
            if set(names) != set(expected) or len(names) != len(expected):
                raise ValueError(f'Source archive contents mismatch: {path}')
            for name, digest in expected.items():
                stream = archive.extractfile(name)
                if stream is None or hashlib.sha256(stream.read()).hexdigest() != digest:
                    raise ValueError(f'Source archive hash mismatch: {path}: {name}')
        return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    archive_reused=True, scope='existing source archive verified against the original manifest')
    recovered = []
    entries = {}
    for name, expected in manifest['source_sha256'].items():
        data = (ROOT/name).read_bytes()
        if hashlib.sha256(data).hexdigest() != expected and name == 'dougie_sim/terrain_qualification.py':
            text = data.decode()
            text = text.replace('''                # Aim beyond the evaluator's crossing plane, as velocity guidance
                # does. Targeting the plane itself stops one grid-cell tolerance
                # short and falsely reports a stall even on a flat floor.
                target = observed_target(self.map.nodes, parent, self.spec.finish_x)''',
                '''                target = min(parent, key=lambda key: math.hypot(self.map.nodes[key][0]-self.spec.finish_x, self.map.nodes[key][1]))''')
            text = text.replace('''\n\ndef observed_target(nodes, reachable, finish_x):
    return min(reachable, key=lambda key: math.hypot(nodes[key][0]-(finish_x+.5), nodes[key][1]))
''', '')
            text = text.replace("['flat', 'stairs_up_030', 'stairs_down_030', 'tiles_020', 'tiles_040', 'ramp_up_10', 'ramp_down_10'] if args.suite == 'repeatability'",
                                "['flat', 'legacy_06', 'stairs_up_060', 'stairs_down_060', 'tiles_040'] if args.suite == 'repeatability'")
            text = text.replace('list(range(101, 161))', 'list(range(1, 61))')
            data = text.encode()
            recovered.append(name)
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'Exact source snapshot unavailable: {directory}: {name}')
        entries[name] = data
    path = directory/'source_snapshot.tar.gz'
    with path.open('wb') as stream, gzip.GzipFile(fileobj=stream, mode='wb', mtime=0, filename='') as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as archive:
            for name, data in sorted(entries.items()):
                info = tarfile.TarInfo(name)
                info.size, info.mtime = len(data), 0
                archive.addfile(info, io.BytesIO(data))
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        reconstructed_and_hash_verified=recovered,
        scope='exact source bytes checked against original run manifest; archive assembled at report time')


def load_cohort(directory):
    manifest = json.loads((directory/'manifest.json').read_text())
    summary = json.loads((directory/'summary.json').read_text())
    expected = {(case['name'], seed) for case in manifest['cases'] for seed in manifest['seeds']}
    records = []
    for path in sorted(directory.glob('*__seed_*/result.json')):
        record = json.loads(path.read_text())
        if record['source_sha256'] != manifest['source_sha256']:
            raise ValueError(f'Mixed source identity: {path}')
        for name, key in [('request.json', 'request_sha256'), ('trace.npz', 'trace_sha256'), ('scene.xml', 'scene_sha256')]:
            artifact = path.parent/name
            if key in record and hashlib.sha256(artifact.read_bytes()).hexdigest() != record[key]:
                raise ValueError(f'Artifact mismatch: {artifact}')
        record['result_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        records.append(record)
    actual = [(record['case'], record['seed']) for record in records]
    if set(actual) != expected or len(actual) != len(expected) or not summary['complete']:
        raise ValueError(f'Incomplete/duplicate cohort: {directory}')
    return manifest, records


def table(rows, confidence=False):
    columns = ['Case', 'Traversed', 'Strict passes', 'Failures']
    if confidence:
        columns += ['One-sided 95% lower bound', 'Meets 95% target']
    lines = ['| '+' | '.join(columns)+' |', '|'+'---|'*len(columns)]
    for row in rows:
        cells = [f"`{row['case']}`", f"{row['traversed']}/{row['trials']}", f"{row['qualified']}/{row['trials']}",
                 ', '.join(f'{reason}: {count}' for reason, count in row['reasons'].items()) or 'None']
        if confidence:
            cells += [f"{100*row['qualified_probability_lower_one_sided_95']:.2f}%", 'Yes' if row['meets_repeatability_target'] else 'No']
        lines.append('| '+' | '.join(cells)+' |')
    return '\n'.join(lines)


def figure(rows, output):
    width, height = 980, 100+len(rows)*25
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#101c29"/>',
        '<g font-family="sans-serif" fill="#e6edf3"><text x="20" y="30" font-size="21">Dougie terrain screening: completion vs strict qualification</text>',
        '<text x="20" y="55" font-size="13">Blue: completed traversal. Green: passed contact, stability, corridor and completion checks.</text>']
    for i, row in enumerate(rows):
        y = 85+i*25
        n = row['trials']
        parts += [f'<text x="20" y="{y+8}" font-size="12">{html.escape(row["case"])}</text>',
                  f'<rect x="335" y="{y-4}" width="480" height="7" fill="#263746"/>',
                  f'<rect x="335" y="{y+5}" width="480" height="7" fill="#263746"/>',
                  f'<rect x="335" y="{y-4}" width="{480*row["traversed"]/n}" height="7" fill="#4aa8e8"/>',
                  f'<rect x="335" y="{y+5}" width="{480*row["qualified"]/n}" height="7" fill="#48c78e"/>',
                  f'<text x="830" y="{y+8}" font-size="12">{row["traversed"]}/{n} / {row["qualified"]}/{n}</text>']
    parts += ['</g></svg>']
    output.write_text('\n'.join(parts)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--screening', type=Path, default=Path('results/dougie/terrain_matrix_v1'))
    parser.add_argument('--observed', type=Path, default=Path('results/dougie/terrain_observed_v2'))
    parser.add_argument('--holdout', type=Path, default=Path('results/dougie/terrain_holdout_v1'))
    parser.add_argument('--replay', type=Path, default=Path('results/dougie/terrain_failure_replay_v1'))
    parser.add_argument('--output', type=Path, default=Path('docs/results/dougie_terrain_qualification_20261007.json'))
    args = parser.parse_args()
    cohorts = {}
    records = {}
    for label in ('screening', 'observed', 'holdout', 'replay'):
        manifest, values = load_cohort(getattr(args, label))
        cohorts[label] = dict(directory=str(getattr(args, label)), manifest=manifest,
                              summary=aggregate(values, manifest['protocol']),
                              source_snapshot=source_snapshot(getattr(args, label), manifest))
        records[label] = values
    protocol = cohorts['screening']['manifest']['protocol']
    if any(cohort['manifest']['protocol'] != protocol for cohort in cohorts.values()):
        raise ValueError('Acceptance thresholds differ between cohorts')
    corrected_names = {r['case'] for r in records['observed']}
    retained_screening = [r for r in records['screening'] if r['case'] not in corrected_names]+records['observed']
    screening_rows = aggregate(retained_screening, protocol)
    holdout_rows = cohorts['holdout']['summary']
    replay = records['replay'][0]
    original = next(r for r in records['holdout'] if (r['case'], r['seed']) == (replay['case'], replay['seed']))
    reproducible = bool(replay['reasons'] == original['reasons'] and
                        np.allclose(replay['final_qpos'], original['final_qpos'], rtol=0., atol=1e-12))
    # The interpretation below is the dated checkpoint, not a generic template
    # permitted to silently repeat old numerical claims for different cohorts.
    expected_holdout = dict(flat=60, stairs_up_030=59, stairs_down_030=60,
                            tiles_020=60, tiles_040=57, ramp_up_10=60, ramp_down_10=60)
    if ({r['case']: r['qualified'] for r in holdout_rows} != expected_holdout or
            any(r['trials'] != 60 for r in holdout_rows) or not reproducible):
        raise ValueError('Results differ from the dated interpretation; update the report narrative before exporting')
    expected_screen = dict(legacy_06=(3, 0), stairs_up_060=(2, 2), stairs_down_060=(5, 0),
        flat_observed=(5, 5), stairs_up_060_observed=(4, 4), stairs_down_060_observed=(2, 0), tiles_040_observed=(5, 5))
    by_name = {r['case']: r for r in screening_rows}
    if any((by_name[k]['traversed'], by_name[k]['qualified']) != value for k, value in expected_screen.items()):
        raise ValueError('Screening results differ from the dated interpretation')
    industrial = [r for r in screening_rows if r['case'].startswith('stairs_') and r['case'].endswith(('_150', '_180', '_200'))]
    if sum(r['trials'] for r in industrial) != 30 or any(r['traversed'] for r in industrial):
        raise ValueError('Industrial stair results differ from the dated interpretation')
    compact_keys = ('case', 'seed', 'data_valid', 'qualified', 'traversed', 'reasons', 'terminal_reason',
        'simulation_time_s', 'wall_time_s', 'nonfoot_impulse_ns', 'nonfoot_peak_force_n', 'nonfoot_contacts',
        'max_tilt_deg', 'max_abs_y_m', 'max_base_z_m', 'min_clearance_m', 'max_penetration_m',
        'max_joint_limit_violation_rad', 'settled_speed_m_s', 'motor_saturation_fraction',
        'mechanical_work_j', 'effective_foot_friction_range', 'stance_contact_slip_p95_m_s',
        'scene_sha256', 'trace_sha256', 'request_sha256', 'result_sha256', 'guidance_statistics')
    result = dict(schema_version=1, baseline_commit='7837f41ea0923237572bde76094f59e3ba4b380b',
        interpretation='Simulation qualification of exact fixtures and seeded perturbations; not hardware certification.',
        useful_trial_count=len(retained_screening)+len(records['holdout']),
        retained_cohort_trial_count=sum(map(len, records.values())),
        superseded_trial_count=sum(r['case'] in corrected_names for r in records['screening']),
        replay_reproduced=reproducible, protocol=protocol,
        study=json.loads((ROOT/'config/dougie_terrain_study.json').read_text()),
        report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        cohorts=cohorts, screening_summary=screening_rows, holdout_summary=holdout_rows,
        trials={label:[{key:r[key] for key in compact_keys if key in r} for r in values] for label, values in records.items()})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    figure(screening_rows, ROOT/'docs/media/dougie-terrain-qualification.svg')
    report = ROOT/'docs/DOUGIE_TERRAIN_QUALIFICATION.md'
    report.write_text('''# Go2 terrain qualification

[Project home](../README.md) · [Initial development evidence](DOUGIE_VALIDATION.md)
· [Portable qualification data](results/dougie_terrain_qualification_20261007.json)

## Verdict

**The current walking policy is not qualified for ordinary industrial stairs.**
Completing one small stair demonstration did not establish repeatable or clean
traversal. This study separates crossing a course from passing stricter contact,
stability, physical-validity and completion checks.

There are **680 usable scored trials**: 52 profiles with five screening repeats,
plus seven candidate profiles with 60 fresh holdout repeats each. Twenty original
observed-guidance trials were superseded after correcting a harness finish-goal
error; they remain in the evidence. One held-out failure was replayed serially.
The four earlier pilot runs remain locally available outside these cohorts.

## Scope and protocol

This qualifies the pinned `robot_lab` Go2 policy in MuJoCo 3.3.6 with a fixed
20 ms policy period, nominal 5 ms physics timestep and the documented PD/torque
limits. Robot geometry, model/policy downloads, code, scene XML and traces are
fingerprinted. There is no base pinning, runtime pose reset or external force.

Seeds vary initial lateral position (±4 cm), yaw (±5°), joint pose (±0.01 rad),
height (±3 mm) and, for random tiles, the terrain itself. Nominal effective foot
sliding friction is **0.4**. Both foot and floor parameters are controlled;
MuJoCo foot contact priority would otherwise make floor-only friction edits
ineffective. Effective friction is measured from contacts, not assumed.

The acceptance criteria were written before the matrix runs in
[`config/dougie_terrain_protocol.json`](../config/dougie_terrain_protocol.json).
They were not relaxed after observing failures. They require:

- Reaching the specified finish plane inside the test corridor, with a witnessed
  upper landing for ascent, followed by a two-second stop and correct final height.
- No sustained excessive tilt/low clearance, side-corridor exit, or stalled
  progress below 0.1 m over 20 s; the course deadline is 180 s.
- No forceful non-foot contact above 25 N peak or 0.1 N·s accumulated impulse
  from contacts above 1 N. This is an engineering clean-traversal criterion,
  **not a manufacturer-certified damage threshold**.
- No penetration over 3 cm, joint-limit excursion over 0.05 rad, nonfinite state
  or MuJoCo numerical warnings. The torque command stays within ±23.5 N·m.

Foot forces, slip, mechanical work, effective friction, motor saturation and
non-foot contact identities are retained as diagnostics. Data validity means
finite, unrepaired simulated state; it does not prove model fidelity to hardware.

### Guidance and information boundaries

Velocity-guided trials use the same ideal-pose centre-line heading controller
across conditions. The policy sees proprioception and velocity commands, not
terrain heights. Evaluator-only geometric reference heights never enter control.

Observed-guidance trials use the production 3-D range sensor, `SurfaceMap`, and
radiation route follower, with a prescribed goal XY and allowed corridor. Support
heights come only from observed points. Planning is synchronous in simulation
time; this is a best-case navigation test, not a real-time or dose-budget test.

## Independent holdout results

Candidates were selected after screening; their 60 seeds (101–160) were not
used for that selection. The policy was not tuned on holdout results. Each row
has an exact one-sided Clopper–Pearson lower confidence bound. The target was
at least 95% success with 95% confidence **for that row's seeded simulation
distribution**. These are pointwise bounds, not a simultaneous guarantee for
all terrain, models, payloads or real hardware.

'''+table(holdout_rows, True)+'''

The 3 cm ascent stalled once (seed 154); 4 cm random tiles stalled three times
(seeds 126, 139 and 159). They therefore fail the stated repeatability target
despite high completion fractions. Flat ground, the tested 2 cm tile distribution,
3 cm descent and the two tested 10° ramp courses each passed 60/60, giving a
95.13% pointwise lower bound. “10° ramps” refers to these courses, not every slope
up to 10°; “2 cm tiles” refers to the generated 25 cm-wide cells.

## Stair findings

- At 6 cm rise / 40 cm tread, nominal velocity-guided ascent passed only **2/5**.
  Descent crossed **5/5** but passed **0/5** clean-traversal checks due to calf
  strikes. The original combined staircase crossed **3/5**, with no strict passes.
- All **30 trials** across 15, 18 and 20 cm industrial-sized risers (both directions)
  failed to complete. Stalls, non-foot impacts and joint-limit excursions are
  visible in the raw evidence. The controller is not demonstrated to climb these.
- Results are non-monotonic: some 10 cm ascents completed while smaller steps
  stalled. A single “maximum step height” is therefore an inadequate capability model.
- Lower friction and a small added payload sometimes improved ascent in the
  five-run screening. This is sensitivity, not justification to change simulated
  friction or mass to manufacture a pass.
- A 20 ms observation delay produced 0/5 clean passes on the nominal 6 cm ascent;
  40 ms also caused stronger contact/joint/corridor failures. The policy is latency-sensitive.
- Halving the physics timestep while keeping the policy at 50 Hz changed some
  marginal stair outcomes. Stable broad qualification cannot be inferred from
  one timestep and initial gait phase.

## Observed-route findings and harness correction

The first observed-mode goal was placed on the finish plane. The production
follower correctly stopped within its waypoint tolerance, but the evaluator
required crossing the plane. Flat-ground traces revealed this mismatch.
The correction moves the **goal** 0.5 m beyond the same finish plane, as in
velocity guidance; neither the plane nor the acceptance criteria changed.
All four observed profiles were rerun, including failures. Their original
20 records are retained as superseded harness-development data.

After correction, flat and 4 cm tiles passed 5/5. Observed 6 cm ascent passed
4/5; descent completed 2/5 and passed 0/5. Thus sensed-path integration does not
resolve the underlying stair-descent limitation. Five repeats do not meet the
60-run repeatability standard, even where every run passed.

## Complete screening matrix

“Traversed” records valid completion; “strict passes” additionally apply the
contact/joint/penetration/settling criteria. The corrected observed rows replace
the superseded rows here; both versions are retained in the JSON.

![Terrain qualification matrix](media/dougie-terrain-qualification.svg)

'''+table(screening_rows)+'''

## Reproduce and inspect

Run from the Dougie-Sim root:

```bash
./dougie terrain-qualify --list
./dougie terrain-qualify --suite matrix --workers 6
./dougie terrain-qualify --suite repeatability --workers 6
./dougie terrain-qualify --cases stairs_down_060 --seeds 0 --workers 1 --view
```

An experiment run exits successfully when it produced valid evidence, even if
the robot failed. Add `--require-qualified` to turn any strict failure into a
nonzero exit. Harness errors or incomplete evidence are not counted as success.
Existing output directories are rejected rather than overwritten.

Each run retains `request.json`, `scene.xml`, `result.json`, and a compressed
20 Hz `trace.npz` of time, root/joint state, torques and commands. Metrics inspect
contacts every physics step. Per-cohort manifests and summaries record every
requested case/seed. The report generator checks completeness, duplicate trials,
source identity and artifact hashes before exporting results:

```bash
docker compose run --rm -T dougie-sim python3 scripts/report_dougie_terrain.py
```

The serial replay of the held-out 3 cm ascent failure reproduced its reasons
and terminal joint/root state to 1e-12. Full raw traces are under ignored
`results/dougie/terrain_*`; the portable JSON includes their hashes and per-run metrics.
Each cohort also has a `source_snapshot.tar.gz` assembled at report time. All
source bytes match its original manifest. The superseded v1 harness was
reconstructed by reversing the documented goal/default-suite changes and was
accepted only after its SHA-256 matched the original; this is recorded explicitly.
The dated report exporter refuses different outcome counts rather than copying
stale conclusions onto new experiments.

## Engineering conclusion

The prior 6 cm staircase and 2–4 cm plate demonstrations remain observations of
completion, not broad terrain qualification. The radiation planner's 7.5 cm
edge-admission threshold is a geometry heuristic, not a validated locomotion
envelope. No planner limit or physical acceptance threshold was widened here.

The next capability gate is a terrain-trained/perceptive walking or stepping
controller with explicit swing-foot clearance and controlled descent, followed
by independent held-out qualification. Training should include riser/tread
variation, downward edges, roughness wavelength, realistic friction, sensor
payloads and observation delay. Repeat the contact-aware tests before claiming
industrial stairs or deploying dose-constrained multi-floor missions.

This study does not cover real hardware, damaged/open-riser stairs, cross-slopes,
loose rubble, wet surface physics, pushes, battery exhaustion or arbitrary
payloads. It establishes specific simulated operating limits and failures,
not universal terrain capability.
''')
    print(json.dumps(dict(useful_trials=result['useful_trial_count'], retained_trials=result['retained_cohort_trial_count'],
        replay_reproduced=reproducible, holdout=holdout_rows), indent=2))
    print(report)


if __name__ == '__main__':
    main()
