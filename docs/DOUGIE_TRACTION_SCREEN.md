# Production stance traction and disturbance screen — 25 October 2026

[Project home](../README.md) · [Stance controller](DOUGIE_STANCE_RECOVERY.md) · [Active plan](DOUGIE_ONBOARD_PLAN.md)

The production `GravityStanceHold` controller passed all 12 cases at effective
friction .4, but only 6 of 12 at .25. Low-traction downhill stopping remains
unqualified: quiet entry failed and physical drift continued after refusal.

## Protocol and results

The [complete 24-case report](results/dougie_stance_disturbance_20261025.json)
uses flat, 8.53° uphill and 8.53° downhill fixtures; friction .4 and .25; two noisy
sensor seeds; and matched zero-force / 15 N lateral-force cases. After 5 s settling
and 8 s commanded approach, the production terminal stance is requested at 13 s.
A prescribed world-lateral force acts at 23 s for .5 s (7.5 Ns impulse). Scoring
continues for 45 s after the stop request, including every refusal.

The controller receives noisy proprioception and estimated attitude, with no
LiDAR or evaluator pose feedback. Placement, force application and physical drift
are fixture/evaluator operations. This directly exercises mission stance code,
rather than duplicating its load-support logic in a harness comparator.

| Effective friction | Push | Passes | Worst horizontal drift |
|---|---:|---:|---:|
| .4 | none | 6/6 | .018583 m |
| .4 | 15 N for .5 s | 6/6 | .022688 m |
| .25 | none | 4/6 | 1.631583 m |
| .25 | 15 N for .5 s | 2/6 | 1.633573 m |

The predeclared drift gate is .05 m; non-foot-contact gates are .1 Ns impulse and
20 N peak force. All 24 passed the contact gates. The two low-friction uphill push
failures retained stance but drifted .050521/.059750 m. All four low-friction
downhill cases hit `stance_entry_deadline`, never entered stance, and drifted
1.627–1.634 m. These failures are retained; refusal is not successful stopping.

The [independent audit](results/dougie_stance_disturbance_audit_20261025.json)
verifies complete/unique cases, full duration, actual contact friction, applied
impulse, the 23.5 Nm motor limit, exact matched pre-push physical samples, and
recomputes failure scoring. This small fixed-fixture screen is not a reliability
claim or vertical-mission qualification.

## Terrain parameter correction and contact priority

The fixture generator assigned requested friction to feet and the base floor but
left generated terrain boxes at MuJoCo's default 1.0. `7e443cb` applies the declared
parameter consistently to generated contact surfaces, with regression coverage
for flat, ramp, stairs, tiles, ridges and legacy courses.

Crucially, the [contact-level probe](results/dougie_contact_friction_probe_20261025.json)
shows foot priority 1 versus terrain priority 0. With foot friction .25, effective
foot/ramp friction remains .25 for both terrain parameters .25 and 1.0. Thus the
parameter inconsistency **does not invalidate historical foot-traction results
on that basis**. Historical artifacts retain their original hashes and scope;
this cohort records actual contact friction, scene identity, fixture source and
software versions. Its frozen implementation is `7e443cb` (a later comment
clarifies contact priority without changing behavior).

## Reproduction

Use a new output path for the screen:

```bash
docker compose run --rm -T dougie-sim python3 scripts/qualify_dougie_stance_disturbance.py \
  --duration 45 --output /tmp/stance-disturbance.json
docker compose run --rm -T dougie-sim python3 scripts/check_dougie_stance_disturbance.py \
  --report docs/results/dougie_stance_disturbance_20261025.json \
  --output /tmp/stance-disturbance-audit.json
```

Next: diagnose low-traction approach/settling dynamics and test a bounded active
braking strategy before requesting quiet stance. Entry/support gates must remain
intact; longer permissive entry or ignoring drift would not address this failure.
