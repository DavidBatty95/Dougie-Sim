# Run-to-run planning learning

The reactor dashboard can tune the radiation-aware robot's path-planning weights
between paired missions. The **blind baseline stays fixed**. Each mission still
starts its own GP and plans from onboard observations. After the pair ends, the
tuner receives the recorded evaluator outcomes as a simulation-training reward.
True sources, field queries and reference coverage are not supplied to a running
mission's planner or GP.

## Controls

Enable **Learn planning between runs** beside the rerun controls. This also
enables continuous reruns; the current pair is retained and tuning starts on the
next pair. Disabling learning stops new tuning trials and retains the chosen
weights. A trial already in flight is still recorded and assessed. Turning off
**Continuously rerun** pauses automatic launching after the current pair ends.
Manual interruptions remain recorded and cannot become winning trials.

The **Learning** section shows:

- Actual total dose and physical distance versus paired-attempt number.
- Retained route/frontier dose weights and the current trial's settings.
- Per-attempt final exploration, eligibility and exclusion reasons.
- Every matched-seed challenger decision, including rejected settings.

Plots use actual µSv and metres. They do not show a fabricated monotonic best-dose
curve. Ineligible attempts leave gaps; earlier untracked runs are reference-only
and cannot teach the tuner. Session-average graphs remain descriptive mixed-policy
averages when settings change.

## What is tuned

Two numerical objective weights are bounded to **0.25–16**:

| Weight | Initial value | Effect |
|---|---:|---|
| `route_dose_weight` | 1 | Radiation penalty relative to traversal time in graph search, including return routes |
| `frontier_dose_weight` | 2 | Predicted outbound/dwell dose penalty when choosing useful frontier goals |

Coordinate challengers double or halve one retained weight. The tuner changes
only aware-objective weights. Robot physics, support clearance, localization,
GP inference, the high-confidence hotspot threshold and mission completion rules
remain fixed. Blind commands always receive weights 1 and 2 and sanitized
radiation observations. The `standard` and `responsive` profiles support tuning;
the experimental geometry-scheduled `pathwise` profile is not a tuning target.

## Matched-seed learning rounds

By default each round uses **three detector seeds**. It runs the retained settings
and then the challenger on seed N, both on N+1, then both on N+2: **six paired
attempts per round**. Each attempt itself contains a blind and an aware mission.
The layout/sensor seeds, world and mission settings stay fixed. A new round uses
fresh seeds. Runtime/configuration hashes prevent a source or protocol change
inside a seed panel from being mistaken for a tuning improvement.

An attempt is eligible only when:

1. Both workers end normally with usable telemetry and mission summaries, without
   errors or an unclean return. Manual/server interruptions are excluded.
2. Both reach the fixed **≥75% reference-floor observation** milestone. Their
   actual crossing percentages may differ by at most 2 percentage points.
3. The aware endpoint reaches at least the blind endpoint's observation minus
   **0.5 percentage points**. It cannot regress a completed blind mission to an
   incomplete aware mission.

A challenger is retained only if all three matched-seed comparisons are eligible,
the blind reference is stable between repetitions, and:

- **Actual total aware dose, including the return journey, is lower on every seed.**
- Mean actual total dose improves by at least **2%**.
- Mean aware dose at the fixed milestone does not regress by more than 2%.
- Final progress and completion do not regress against the retained aware policy.

The fixed blind reference must retain completion/progress and stay within 2% in
total dose between repetitions of a seed. Dose ratios are retained as descriptive
metadata; acceptance uses **actual aware dose**, not a ratio that could improve
merely because the blind robot received more radiation.

Incomplete exploration remains labelled incomplete. Comparable-progress learning
is not proof of frontier exhaustion, full-environment coverage, generalization
to independent layouts or hardware performance. Retained parameters are
**provisional simulation-development settings**, not a guaranteed optimum.

## Persistence and launch

`comparison_session.json` stores trials, rewards, seed-panel position, retained
weights and rejected/promoted rounds atomically with run history. Every new run's
configuration records the exact weights, role, seed and protocol fingerprint.
`GET /learning` exports the learning state; `/history` includes the learning
assessment for every attempt. Replaying the session directory restores history
and can resume a partially completed seed panel without scoring trials twice.
Use a fresh session to change the reward milestone or seed-panel size.

```bash
# Mac dashboard: enable learning for the session.
DOUGIE_COMPARE_CAMPAIGN=0 DOUGIE_COMPARE_LEARNING=1 ./demo

# Native launcher, default reactor and open-ended missions.
./dougie compare --world reactor --learn-planning

# Inside the runtime; an explicitly configured learning experiment.
python3 -m dougie_sim.comparison --world reactor --learn-planning \
  --learning-milestone 75 --learning-seeds 3
```

Uncapped missions can take substantial time. Learning waits for both workers to
end; it does not quietly introduce a dose budget or simulated-time cap. The short
`dashboard_planning_learning_validation` run uses a development-only low milestone
and time cap to test controls and serialization, not to claim dose improvements.
