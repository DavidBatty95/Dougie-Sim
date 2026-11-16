# Radiation-lifetime mission packages

`./demo` starts a reactor campaign by default. A package contains up to **six
paired sorties**. Each policy has its own evolving world and retained knowledge.
The default physical lifetime is **2 µSv-equivalent per robot**, with an aware
planning ceiling of **1.6 µSv** to leave headroom for returning.

These are **illustrative simulator damage budgets**, not measured Unitree Go2
electronics tolerances. The synthetic µSv-equivalent field is not a calibrated
electronics-damage/fluence model. Change the lifetime and package size in the
campaign controls and press **New campaign** to start another experiment.

## Survival before additional exploration

The aware planner minimizes expected transit/turning/dwell exposure while imposing
an upper-estimate dose constraint that includes the observed return route, waiting,
recovery reserve and a budget margin. It does not spend its entire physical
lifetime before deciding to return. Nearby detector counts, GP uncertainty and
high-confidence hotspot exclusions remain active. A bounded number of apparently
budget-limited frontier trips are rechecked on executable smoothed geometry before
being rejected; every selected route alternative must still fit the round-trip
planning budget.

Unknown/unallocated GP windows retain their mixture prior and uncertainty. A
zero-weight, untrained neighbour is not allowed to impose its full prior upper
estimate over a well-measured return location. Learned hotspot bounds are retained.
Schema-3 checkpoints record this evidence-aware upper rule; older checkpoints keep
their original rule when loaded.

The physical lifetime is enforced independently of planning estimates for **both**
robots, using integrated true simulated exposure on each 5 ms physics tick. Model
error can still cause a loss; survival is evaluated, not guaranteed. The blind
ablation receives sanitized radiation observations and cannot route or trigger a
dose-reserve return using the evaluator's true field/dose.

## Death and persistent obstacles

At the failure threshold the robot is disabled **at its actual current pose**.
No return or teleport is manufactured. Motion/torque commands stop and the sortie
ends with `dead_radiation`. The final feed colours the failed robot red and shows
its lifetime/death status. Numerical overshoot is bounded by the exposure accrued
on one physics tick.

`dead_robot.json` retains the articulated failure pose. Subsequent sorties add a
static copy of the complete Go2 body/limbs/meshes at that pose. The frozen robot
has collision geometry and is visible to range sensing; it is not just a coloured
marker. The active robot's contact classification excludes the wreck from its
own body. These obstacles are frozen in situ, not a powered-off ragdoll collapse
or fracture model, and are not invented radiation shields/sources.

The navigation map learns new wreck obstacles from range returns. The supervisor
uses the retained failure pose to construct the physical scene and protect spawn
clearance at the origin. A wreck blocking that clearance ends the package instead
of spawning another robot in collision.

## Recovery, replacement and learning

A new body can be deployed when the previous robot:

- Returns alive to within **0.4 m of the physical origin**, or
- Is destroyed, leaving its wreck in the world.

Each new deployment has a new robot ID, a fresh detector/lifetime dose and a fresh
body at the origin. The old robot's dose is never reset in place. A live robot
stranded away from the origin, an interrupted sortie or missing result/memory
artifacts halt automatic progression. **Next sortie** cannot interrupt a robot
and silently give it another allowance. **New campaign** is an explicit new
experiment; it archives the old package before clearing that new world's wrecks
and knowledge.

After each sortie both policies retain observed geometry, visited support and
viewpoint dwell history. The aware policy additionally retains its learned GP
posterior. The blind policy never imports nonzero radiation beliefs. GP geometry
clocks are rebased for the new sortie without replaying/duplicating count data.
Each robot that returns alive also contributes its physically travelled support
trajectory to its own team's shared archive. Later robots match consecutive
observed trajectory points to the current support graph and give previously
traversed edges a modest travel-cost preference. The full radiation cost is still
charged, predicted dose and return budgets are not discounted, and current
obstacle, support and radiation checks can reject any formerly used edge. Routes
from robots that did not return are not marked as successful; their available
map/GP observations remain retained as before. Failed bodies remain obstacles.
Shared routes and GP knowledge guide later deployments; they cannot retroactively
change earlier robots' decisions. The dashboard reports recovered paths and
currently usable route edges for each independent team.
Exploration completion still requires zero frontiers on three fresh scans; a
survival-mode robot must then return before the campaign counts recovery.

Campaign learning is persistent GP/map knowledge. The independent matched-seed
planning-weight tuner is paused: changing wrecks and cumulative map knowledge
must not be mistaken for evidence that a numerical weight improved. Previously
retained planning weights can be used as fixed campaign settings.

## Interpreting the dashboard

- Per-robot lifetime bars and a failure-dose line show the current body's
  remaining allowance. Current-sortie graphs reset with a fresh deployment.
- Campaign graphs accumulate **all active-sortie dose/time/distance** and show
  fixed-reference observation progress, robots lost and bodies recovered.
- The reference floor denominator is carried unchanged between sorties. A new
  wreck cannot shrink it to manufacture better coverage. Previously observed
  cells remain observed; navigation admission is recomputed from new sensing.
- Matched-progress headline comparisons use cumulative campaign dose, not a fresh
  robot's near-zero dose against earlier campaign coverage.
- Standby between sorties and passive irradiation of already-dead wrecks are not
  charged as active robot operating life. Recovered robots are removed from this
  field for the simulated swap.

The paired package concludes at its sortie count, when a survey is exhausted with
the robot recovered, or when a stranded robot, blocked origin or failed artifacts
prevent a valid next deployment. Incomplete geometry remains incomplete. Compare
recovery/loss **together with exploration**, rather than treating lower dose after
less exploration as an equal-work advantage. Worlds can legitimately diverge as
one policy accumulates more wrecks.

```bash
# Reactor campaign (default ./demo), with a different indicative lifetime.
DOUGIE_COMPARE_LIFETIME_DOSE=3 DOUGIE_COMPARE_CAMPAIGN_SORTIES=8 ./demo

# Inside the runtime, explicit package settings.
python3 -m dougie_sim.comparison --world reactor --campaign \
  --lifetime-dose 2 --campaign-sorties 6 --survival-fraction .8

# Independent fresh-run experiments / numerical planning-weight tuning.
DOUGIE_COMPARE_CAMPAIGN=0 DOUGIE_COMPARE_LEARNING=1 ./demo
```

`comparison_session.json` stores the current ledger and archived packages. Each
sortie retains its campaign input, lifetime outcome, GP/map memory, fixed coverage
reference and any wreck. The short `dashboard_campaign_death_validation` and
`dashboard_campaign_recovery_validation` packages exercise actual failures and
swaps with deliberately artificial limits/time caps; they are functional checks,
not independent-environment or equal-work performance evidence.

## Retained verification

[Functional check records and input hashes](results/dougie_campaign_checks_20261114.json)
retain failure, recovery, warm-start and physical-scene import results. A separate
one-sortie diagnostic with the default 2 µSv-equivalent limit produced:

| Outcome | Blind | Aware |
|---|---:|---:|
| Robot | Destroyed in situ | Returned alive |
| Active simulated time | 217.19 s | 496.14 s |
| Actual dose | 2.00005 µSv | 0.84015 µSv |
| Fixed floor observed | 65.13% | 47.00% |

This demonstrates the model's survival/exploration trade-off, **not equal-work
dose superiority**: the aware robot preserved its body but explored less in this
sortie. Subsequent deployments retain its knowledge. The separate two-sortie
tight-budget fixture confirms an actual failed-robot body appears in the next
physical scene; physics tests verify its pose, range visibility and collisions.

The [first full six-sortie reactor package](results/dougie_reactor_campaign_20261114.json)
recovered **6 aware robots with 0 losses**, versus **1 blind robot recovered and
5 losses**. Its final fixed-reference observation was **51.93% aware versus
90.30% blind**. Thus the survival improvement came with less exploration; these
correlated sorties are a demonstration of the resource trade-off, not six
independent trials or equal-work dose evidence. All five blind wrecks remained in
the final physical scene.
