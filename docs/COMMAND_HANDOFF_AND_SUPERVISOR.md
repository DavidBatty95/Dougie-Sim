# Command handoff and supervisor follow-up — 5 October 2026

Later same-day work implements the background-search split and tests a configured
50 Hz supervisor: [asynchronous escape checkpoint](ASYNC_ESCAPE_SUPERVISOR.md).
The timing/rate statements below describe this earlier synchronous checkpoint.

Continuation of [S0/R1–R3 phase work](PHASE_PROGRESS_20261005.md). This checkpoint
implements another R2 handoff slice and reduces R3 supervisor work. The complete
source-ownership arbiter, hard real-time response and social-navigation acceptance
remain open.

## Diagnosis of the retained seed-44 encounter

The retained `20261005T120545Z` seed-44 trace shows a turn reference being replaced
by lateral/reverse progress while people are initially distant. As actors start
walking, the robot enters a conflict, attempts translation escapes and eventually
stops while a person continues approaching. Tracks are present by the late
encounter; this is not simply a missing-person detection failure.

Two separable issues were identified:

1. Broad caution activation can replace a feasible alignment turn with sideways
   goal progress. This weakens the requested goal-facing behavior and changes
   where the robot meets people.
2. Velocity, continuation and planner state were received on separate topics.
   A goal change cleared continuation metadata but left the old velocity fresh.
   That was a handoff defect independent of whether it caused this collision.

The native tests below do **not** establish a single root cause of the collision.

## R2: atomic, epoch-qualified planner request

The existing `/human_avoidance_command_sequence` now carries a versioned envelope:

- Publisher instance UUID and monotonically increasing sequence ID.
- ROS production stamp, coordinate frame and active-mission state.
- Full goal signature: frame, stamp, position and orientation.
- First velocity, optional continuation/duration and planner state together.

With `require_command_envelope: true`, the onboard supervisor uses that envelope
as its authoritative planner input. Plain Twist and planner-state topics remain
available for visualization/legacy configurations but cannot overwrite it.

The receiver rejects stale, future, reordered, duplicated, wrong-frame and
wrong-intent packets. Goal or active-state changes immediately clear old input
and establish a timestamp fence. Heading-only goal changes also revoke input.
An expired publisher may be replaced, and a retired publisher cannot return.
Accepted input still expires after at most 0.25 s, with fresh mission heartbeat
required. Fresh collision-avoidance requests remain possible while a mission is
inactive; task cancellation is not confused with disabling collision protection.

This is **one control hop**, not the complete navigation/teleop/docking lease
arbiter, not a hardware command acknowledgement and not authentication.

Startup source auditing now requires exactly one expected ROS writer on:
`/cmd_vel_human_aware`, `/human_avoidance_command_sequence`, `/cmd_vel_safe` and
`/g1/policy_cmd_vel`. Duplicate writers with the same node name are rejected too.
It is a startup check, not continuous DDS/hardware ownership enforcement.

### Command lineage

Supervisor diagnostics retain accepted publisher/sequence IDs. Offline analysis
joins those IDs to the original planner envelope and verifies the input command,
without assuming ROS callbacks arrived in the same order.

- Alignment pilot: **986/986 eligible decisions matched**, zero command mismatch.
- Final full-pose, one-person/three-goal test: **507/507 matched**, zero missing
  planner records or command mismatch.

These establish the considered planner request's identity, not physical execution
of the supervisor's output. An override can deliberately produce a different command.

## Goal-facing selection

`prefer_safe_route_alignment` lets a fully scored pure-turn reference retain
priority when its forecast remains outside the comfort/body reserve. It applies
only to unloaded normal route travel, outside the holonomic finishing region.
It does not bypass static or human collision rejection, force turns during
docking, or suppress escape search when the forecast conflicts.

The feature is experimental. It improved behavioral fidelity in the low-density
test but did not resolve the four-person seed-44 encounter. Yaw limits are unchanged.

## R3: remove repeated collision-check preparation

Profiling found that predictive escape search repeatedly validated and densified
the same pedestrian prediction message for each proposed command. The supervisor
now prepares an immutable copy once per input message. Each command still checks
the current message age, confidence, covariance-adjusted clearance and its own
robot trajectory. Sampling-resolution changes rebuild the preparation correctly.

Static vertex-distance queries are batched with NumPy for larger scans, using
bounded point/obstacle blocks. The scalar path remains for small inputs. Tests
check rotation, exact clearance boundaries, duplicate returns, later-block
collisions and agreement with the scalar predicate. This retains the existing
vertex-based check; it does not add missing full-body swept-volume validation.

### Observed supervisor cost

| Four-person seed-44 variant | Escape-search p99 | Tick-to-publication p99 | Maximum tick-to-publication |
| --- | --- | --- | --- |
| Before preparation | 36.2 ms | 43.2 ms | 56.3 ms |
| Prepared predictions | 12.8 ms | 16.4 ms | 20.6 ms |
| Prepared + batched static checks | 17.0 ms | 20.0 ms | 23.8 ms |

In the longer seed-42 prepared-only run, tick p99 reached 58.0 ms; the later
batched run measured 19.4 ms, with a 45.5 ms maximum. Different trajectories,
scan densities and run durations prevent a controlled system-level speedup claim.
The equivalence tests establish preserved query semantics; native records show
the remaining timing tails.

**The supervisor stays at 20 Hz.** These tails do not justify declaring a bounded
50 Hz loop. Moving expensive alternative search off the immediate-check path,
with fresh revalidation before use, remains the next R3 work package.

## Retained native attempts

All cases used the courtyard and synchronized actor release. Four-person cases
requested three goals with a 90-second budget. They are development variants,
not a frozen reliability cohort.

| Record | Strict result |
| --- | --- |
| [Atomic envelope, seed 44](results/avoidance_r2_atomic_20261005.json) | 0/1; 1/3 goals; one contact |
| [Alignment priority, seeds 44/42](results/avoidance_r2_alignment_20261005.json) | 1/2; 3/6 goals; one contact |
| [Supervisor timing, seed 44](results/avoidance_r3_supervisor_timing_20261005.json) | 0/1; 0/3 goals; one contact |
| [Prepared predictions, seeds 44/42](results/avoidance_r3_prepared_20261005.json) | 1/2; 3/6 goals; one contact |
| [Batched static checks, seeds 42/44](results/avoidance_r3_batched_20261005.json) | 0/2; 1/6 goals; two contacts |
| [One-person protocol smoke, seed 44](results/avoidance_r2_protocol_smoke_20261005.json) | 1/1 requested goal, zero contacts; score 91.206 |
| [Full-pose envelope, one person, seed 44](results/avoidance_r2_protocol_handoffs_20261005.json) | 3/3 goals, zero contacts; score 85.772; minimum body clearance 0.801 m |

The lower-density passes verify native handoff and goal changes. They do not
replace the failed four-person cases or close S2/S6. All thresholds are unchanged.

## Remaining gates

Verification for this checkpoint: **388 bridge tests, 95 benchmark tests and 18
bringup tests passed**, plus **29 focused script tests**. Documentation links,
generated graphics/navigation and whitespace checks passed. This includes epoch/fence/replay faults, missing/duplicate
writers, forecast/static rejection, immutable prediction preparation and batched
clearance equivalence. Native results remain subject to the narrower scopes above.

- R2: upstream source acquisition/release, persistent mission ownership and
  lineage through policy/compositor/bridge to applied motor commands.
- R3: truly independent immediate validation and asynchronous alternative search;
  no stale worker result may cross a goal/ownership epoch.
- R4/R5: executed-motion versus forecast error, onset/occlusion uncertainty and
  tactical encounter choices in the retained seed-44 failures.
- S2/S6: complete multi-world and held-out acceptance remains outstanding.
