# Asynchronous escape search — R3 checkpoint, 5 October 2026

Follow-up to [command handoff and supervisor profiling](COMMAND_HANDOFF_AND_SUPERVISOR.md).
The expensive alternative search is now separated from immediate command validation.
This is an implemented and natively exercised work package, not completion of
crowd-navigation or hard-real-time acceptance.

## Architecture

```text
Latest measured pose, velocity, scan and predicted people
           │
           ├── immediate requested-command / stopping checks ──► output
           │
           └── immutable snapshot ──► one background search
                                       │
                             epoch and age checks
                                       │
                             current-input revalidation ──► output if admitted
```

- `prediction_risk.py` holds ROS-independent prediction preparation/risk queries;
  existing imports through the supervisor remain available.
- `escape_search.py` holds immutable snapshots, pure search/admission functions
  and a single-flight worker. The worker receives no ROS node or mutable node state.
- There is no queued backlog: another request is refused while a job or its
  uncollected result occupies the slot. Polling never waits for computation.
- Completed results are drained even during ordinary/idle control ticks. This
  fixes a case where completed work occupied the slot until a later encounter.
- Goals, active-mission state, publisher epochs, payload changes and prediction
  clock regression invalidate prior generations. Old results cannot restore them.
- Results must satisfy both monotonic and ROS age bounds, currently 0.20 s.
- A proposed velocity is checked again against **current** people predictions,
  scan, pose, measured velocity, command filtering and stopping clearance. Its
  old score is never treated as permission to execute.
- Missing/expired/rejected proposals produce no escape command. The requested
  command is still independently validated; if none is admitted, the fallback
  is a zero request. This does not prove that standing still avoids an incoming person.
- Graceful teardown requests a stop before waiting for finite background work,
  when the ROS context permits publication. This is not an actuator acknowledgement.

Asynchronous mode requires the epoch-bearing command envelope. Legacy synchronous
mode remains available for comparison. The worker is a **thread**, so the process,
Python GIL and OS scheduling remain shared; this is not hard CPU isolation.

## Configuration and observability

The current experimental onboard profile selects:

```yaml
human_collision_avoidance:
  ros__parameters:
    require_command_envelope: true
    async_predictive_escape_search: true
    escape_result_max_age: 0.20
    rate_hz: 50.0
```

The ROS node's standalone default keeps asynchronous mode disabled. Diagnostic
output distinguishes waiting, no feasible proposal, rejected revalidation and
accepted revalidation. Worker counters include completed/proposed/empty results,
failures, stale generations, expired work and busy submissions.
Mode and cadence are read at startup; restart with the desired profile for
comparisons rather than assuming a runtime parameter edit changes an existing timer.

`escape_control_path_ms` measures submission/collection/rechecking on the control
path. Worker submit-to-complete time is separate. The trace analyzer samples worker
duration once per observed completion and reports missing completion durations;
repeated status messages do not multiply timing samples. Early async pilot records
used the old `predictive_search_ms` field for control-path work; their async flag
identifies that scope. Current async records leave synchronous search time null.

## Verification

**404 bridge tests, 95 benchmark tests, 18 bringup tests and 31 focused script
tests passed.** New fault tests cover:

- Slow/blocked search while control validation continues without waiting.
- Single-flight backpressure, failure handling and expiry/clock regression.
- Late completion after goal, mission, publisher or payload changes.
- New people/static obstacles invalidating a previously proposed escape.
- Fresh scan requirements and successful current-state revalidation.
- Completed-job drainage while idle and stop-before-worker-shutdown ordering.
- Rejection of async configuration without command-envelope fencing.

The search and revalidation share the previous short-horizon admission predicate.
The existing sampled human/static geometry limitations are unchanged; full-body
swept contacts and accurate physical response remain separate gates.

## Native experiments

The courtyard pilots used synchronized actor release, three requested goals and
a 90-second trial budget. Source/configuration and resolved runtime values are
retained with every attempt. Seeds do not fix observed-map goal admission or ROS
scheduling, so these are development comparisons, not a causal reliability estimate.

| Candidate | People / seeds | Outcome |
| --- | --- | --- |
| [Async at 20 Hz](results/avoidance_r3_async20_20261005.json) | 4 / 42, 44 | 0/2 strict passes; 1/6 goals; two contacts |
| [Async at 50 Hz](results/avoidance_r3_async50_20261005.json) | 4 / 42, 44 | 0/2 strict passes; 3/6 goals; one contact |
| [Idle-drain/telemetry follow-up](results/avoidance_r3_async_handoffs_20261005.json) | 1 / 44 | Strict pass: 3/3 goals, zero reported contacts, score 81.036 |

In the 50 Hz crowd pilot, seed 42 completed 3/3 goals with zero contacts but its
score **62.004** failed the unchanged minimum of 70. Seed 44 still collided.
The one-person follow-up reached a 0.620 m minimum body clearance and recorded
2,103/2,103 matching planner/supervisor lineage records with no command mismatch.
It exercised two worker proposals and two revalidated escape publications.

### Timing evidence

| Configuration | Supervisor callback p99 | Actual tick interval p99 |
| --- | --- | --- |
| Earlier synchronous batched checks, 4 people | 19.4–20.0 ms | 62.5–63.2 ms at configured 20 Hz |
| Async 20 Hz, 4 people | 3.81–3.83 ms | 61.7–64.8 ms |
| Async 50 Hz, 4 people | 3.18–5.57 ms | 35.0–35.2 ms |
| Async 50 Hz follow-up, 1 person | 3.15 ms | 30.8 ms |

At 50 Hz the median interval was approximately 20 ms. Scheduling tails still
exceeded that period: **configured 50 Hz is not a bounded 50 Hz guarantee**.
The low-density run's final worker duration was 64.1 ms while callback maximum
was 8.63 ms, illustrating the separation of search from publication work; those
are different measurements, not interchangeable end-to-end latency claims.

The initial 50 Hz seed-42 run rejected 15 proposals on fresh revalidation. Eight
completed results expired, including the completed-mailbox issue corrected in
the follow-up. The follow-up reported no worker failure or expiry, but its
lower density does not qualify loaded or four-person reliability.

## Next gates

1. Repeated crowded/loaded timing, scheduling tails and callback-load isolation.
2. Explain no-feasible-search results using captured inputs and executed-motion
   error; faster processing alone does not resolve the retained seed-44 conflict.
3. Complete upstream command leases and provenance through policy/compositor/bridge
   to actual motor application.
4. Preserve strict full-matrix and held-out acceptance before promoting S2/S6.
