# S0/S1, R1–R3 implementation and native evidence — 5 October 2026

This report covers local work after the published baseline. It records partial
deliverables in the [phase plan](PROJECT_PLAN.md), not promotion of whole stages.
Run manifests and portable datasets identify the actual candidates; these are
development experiments with failures retained.

Later same-day work is recorded separately in the
[command-handoff and supervisor follow-up](COMMAND_HANDOFF_AND_SUPERVISOR.md).

## Implemented deliverables

### S0.2 — Resolved runtime configuration

`runtime_parameters.py` queries parameter services after graph readiness and
writes each node's resolved values and types. Missing nodes, duplicate names,
service timeouts and malformed responses remain explicit. Transport requires
12 nodes; onboard navigation requires ten. Optional helper nodes have a shorter
budget rather than delaying every run for the full required-node timeout.

The snapshot compares **23 shared motion parameters**: command limits/filtering
across planner, supervisor and policy, and prediction-response parameters across
planner/supervisor. Missing or inconsistent required values block mission start.
This does not assert that the shared response model matches physical behavior.

- Transport writes `effective_parameters.json` with its SHA-256 in mission events.
- Native avoidance matrices request and retain `*_parameters.json` before goals.
- Matrix discovery distinguishes parameter sidecars from benchmark results.
- This is a startup snapshot, not an atomic distributed snapshot or continuous
  parameter-change audit. Clean-machine reproduction and native build-dependency
  attestation remain S0 work.

### S1.4 / R1 — Response observability

The run logger now records monotonic receipt time, filtered policy commands,
policy timing/counters, locomotion diagnostics and transport phase/possession.
Policy timing identifies the exact inference generation and sequence reaching
the joint-target publisher and includes:

- Submission-to-completion and completion-to-publication time.
- Submission-to-publication time.
- Command receipt age and LowState **receipt** age at that publication.

Planner telemetry includes input ages at command publication, configured period,
budget and rollout/static/total scoring time. `scripts/analyze_response_trace.py`
reports median/p95/p99, overruns, invalid samples, duplicate events, clock resets
and simulator real-time-factor windows. Both transport and avoidance matrices
automatically produce `response_diagnostics.json`.

**Measurement boundaries:** joint-target publication is upstream of composition,
bridge/DDS and actual motor application. LowState acquisition time is unavailable.
Input ages are not causal end-to-end latency. Missing legacy fields remain null.
Command-to-motion onset/stopping still use the fixed-command protocol. Periodic
counters carry their own timestamp and may precede the final inference event.

### R3 — Bounded priority reservation

The measured crowd baseline exposed the old scheduler's soft-budget problem:
whole escape families were mandatory, even after the search budget expired.

`priority_candidate_limit` bounds reserved non-core proposals. Stop, previous
command and route checks remain mandatory; reserved escapes and optional
refinements rotate fairly. Deferred candidates are still evaluated if time
remains. Every evaluated/selectable proposal retains full collision checking.
Zero preserves the earlier scheduler; the experimental onboard profile uses four.

Deterministic tests demonstrate an over-budget tick checking three core proposals,
four reserved escapes and one optional refinement rather than an entire 20-member
escape family, while covering every direction across subsequent ticks. This is
still a **soft** deadline: core checks, one-candidate work and recovery can overrun.
Independent high-rate collision checks and worst-case execution bounds remain open.

### S1.2 / R2 — Stop-generation barrier

A transition from movement to an immediate zero command invalidates pending or
completed-but-unconsumed pre-stop inference. Repeated zero commands do not keep
invalidating balance work. Last validated targets are retained while a fresh
zero-command inference is prepared. Results completed on time but consumed after
the configured inference deadline are rejected too.

Fault tests block a worker mid-inference, issue a stop, and verify that its old
result is discarded and a fresh zero-command generation can complete. Telemetry
records stop invalidations and stale-result rejections. This does not implement
the full R2 multi-source lease arbiter or imply instantaneous physical stopping.

## Native results

All crowd cases below used the courtyard, four people, three requested goals,
90-second budget and synchronized actor release. Synchronizing actor release does
not fix asynchronous perception, observed-map goal admission or ROS scheduling.

| Candidate | Seeds | Strict outcome | Evidence |
| --- | --- | --- | --- |
| Instrumented baseline | 42 | 0/1; 1/3 goals; one contact | [R1 baseline](results/avoidance_r1_baseline_20261005.json) |
| Additional scoring phase telemetry | 42 | 0/1; 1/3 goals; one contact | [R3 profiling](results/avoidance_r3_profile_20261005.json) |
| Four reserved priority proposals | 42, 44 | 1/2; 4/6 goals; one contact | [R3 bounded pilot](results/avoidance_r3_bounded_20261005.json) |
| Priority bound plus stop-generation barrier | 42, 44 | 0/2; 2/6 goals; two contacts | [R2 interruption pilot](results/avoidance_r2_stop_generation_20261005.json) |

The baseline's active planner median/p99 were **110/154 ms**. In the bounded
seed-42 run they were **49/82 ms**, with all three goals reached, zero contacts,
0.415 m minimum body clearance and a strict score of 84.027. Seed 44 still
collided. These changing trajectories and small samples do not establish a
reliable success-rate improvement or causal attribution of the later failures.

Active/idle denominators are now separated explicitly: the baseline had **119/127
active-cycle overruns (93.7%)**, not 125/127 (125 was the whole-trace count).
The bounded seed-42 run had **261/578 (45.2%)**. The different trajectory lengths
and decision distributions still preclude a controlled success-rate claim.
Recomputed reports retain the original trace hashes:
[baseline active timing](results/response_r1_active_20261005.json) and
[bounded active timing](results/response_r3_active_20261005.json).

The baseline reported prediction age at command publication p99 **564 ms** and
policy submission-to-target-publication p99 **35.5 ms**. The bounded seed-42 run
reported **368 ms** and **23.0 ms** respectively. The planner therefore remains a
timing priority even though it is faster in this pilot.

The stop-generation pilot recorded ten and two stop invalidations respectively,
with zero stale-result rejections in the last periodic snapshots. Its failures
remain failures; the interruption fix is not promoted as a navigation success.

### Transport capture / deadline diagnostics

Two deliberately short transport runs used the cycle-01 policy, `payload_v1`
posture, no people and 30/35-second mission deadlines:

- `20261005T114006Z`: captured all required parameters; deadline during pickup
  docking; empty possession, no fall/drop, valid evaluator data. Capture took
  10.279 s because optional nodes consumed the full timeout.
- `20261005T114430Z`: captured all required parameters with consistent motion
  settings in 3.053 s after the optional-budget correction; grasped and lifted,
  then deadline during retreat; possession correctly remained `held`, no fall/drop,
  valid evaluator data.

Both report unsuccessful/incomplete missions. They are startup/timeout diagnostics,
not attempted full delivery acceptance. [Portable records](results/transport_runtime_capture_20261005.json).

### Isolated stopping check after the generation barrier

`fixed_command_scene_29dof_empty_20261005T120428Z` used the official unloaded
policy, +0.35 m/s for six seconds, four seconds stopping, and 0.22 m/s² command
slew. It passed the unchanged fixed-command validator: 0.684 s onset, 0.315 m/s
steady response, **0.088 m stopping drift**, maximum tilt 0.049 rad, no fall.
This is one low-speed empty-scene check, not loaded/crowd qualification or a
matched before/after stopping improvement.

## Promotion status and next work

Verification: **357 bridge tests, 83 benchmark tests, 18 bringup tests and 28
focused script tests passed**. The modified shell scripts passed syntax checks;
documentation links, generated graphics/navigation and whitespace checks passed. These results cover software
contracts and orchestration, not the unpassed native acceptance gates.

- **S0.2:** implemented and natively exercised; S0 clean reproduction still open.
- **R1:** useful source-identified baseline now exists; actual actuator acknowledgement,
  causal command lineage, repeatability and loaded coverage remain open.
- **R2:** stop-generation slice implemented; explicit source leases/joint ownership
  and comprehensive interruption scenarios remain open.
- **R3:** priority work is bounded and pilot timing improved; independent fast
  checks and a reliable hard-deadline envelope remain open.
- **S2/S6:** not accepted. Do not relax contact/goal/score thresholds or promote
  the one successful courtyard case to full-matrix reliability.

Next: replay the retained seed-44 encounter with acquisition-time tracking and
actual executed commands, finish source-ownership/command-lineage instrumentation,
then test the next single-factor correction. Broad loaded missions remain gated
on calibrated response and dependable navigation.

## Reproduce diagnostics

Inside a sourced ROS container with the onboard graph running:

```bash
python3 -m unitree_g1_mujoco_bridge.runtime_parameters \
  --scope navigation --output /tmp/effective_parameters.json
```

This standalone command returns nonzero for incomplete required capture or a
shared-motion mismatch. Transport and matrix launchers save their snapshots in
the persistent run directory automatically.

On the host, analyze a retained trace:

```bash
python3 scripts/analyze_response_trace.py TRACE.jsonl --output response_diagnostics.json
```

The trace and analyzer hashes are embedded in the report. To repeat the current
two-case development pilot with a new output identity:

```bash
python3 scripts/run_avoidance_matrix.py --worlds small_open --seeds 42 44 \
  --people 4 --goals 3 --trial-seconds 90 --synchronized-start \
  --label phase-followup-repeat --publish docs/results/phase_followup_repeat.json
```
