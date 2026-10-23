# Gravity-supported stance and terminal recovery screening

[Project home](../README.md) · [SLAM qualification](DOUGIE_SLAM_QUALIFICATION.md) · [Active plan](DOUGIE_ONBOARD_PLAN.md)

## Implemented controller

`dougie_sim/stance_hold.py` adds an experimental quasi-static load-support model.
It receives measured joint angles and estimated body-frame gravity, plus the
constant mass of the pinned Go2 model. Leg kinematics provide foot positions and
Jacobians. Nonnegative gravity-aligned foot loads balance total weight and moment
about the base origin; joint support torque follows `tau = -Jᵀ f`.

The base origin is an **approximate centre of mass**, and limb gravity is not
explicitly modelled. This is not full inverse dynamics. No evaluator position,
world geometry, radiation truth or terrain height enters the controller.

Support torque is converted to a joint-target offset through the existing PD
gain. A .5 s blend transfers from the last commanded targets to the supported
stance. Existing gains, action clipping and the **23.5 Nm motor limit** remain in
force. The support term itself is limited to **12 Nm**; unsupported load solutions
are refused rather than clipped into apparent feasibility.

Entry requires .5 s of settling followed by .2 s of quiet four-foot support,
with a three-second entry deadline. Measurements must be ≤.05 s old; each foot
force must exceed 5 N. Entry gyro/joint-speed limits are .15 rad/s and .5 rad/s.
While holding, lost/stale foot support or gyro norm above .3 rad/s permanently
refuses that attempt and restores the zero-command policy. Refusal is recorded;
it is not scored as successful holding. A terminal request cannot restart its
entry timer. No nonzero velocity command is accepted while terminal stance is
requested.

## Paired stance screen

The [20-case screen](results/dougie_gravity_stance_20261023.json) retains flat,
8.53° and 10° up/down fixtures, sensor seeds 0/1, a six-second walking approach,
and 120 seconds after the stop request. Friction is .4. A separate
[downhill pilot](results/dougie_gravity_stance_pilot_20261023.json) is also retained.
The prototype source was frozen in `1299cf9` before the full screen.

| Fixture | Zero-command max XY drift | Gravity stance max XY drift |
|---|---|---|
| Flat | .082–.098 m | .0088–.0092 m |
| 8.53° uphill | 1.545–1.550 m | .0154–.0192 m |
| 10° uphill | 1.513–1.737 m | .0188–.0198 m |
| 8.53° downhill | 1.278–1.411 m | .0112–.0159 m |
| 10° downhill | 1.624 m | .0123–.0191 m |

All ten gravity cases entered and remained available, with **zero non-foot
contact**. The reported maxima include the entry delay and load transfer, not
just the settled final position. [Telemetry animation](media/dougie/gravity-stance.gif).

## Handover after LiDAR loss

The [16-case paired fault screen](results/dougie_gravity_fallback_20261023.json)
uses the observable-wall fixtures and permanent LiDAR dropout at 30 s. It compares
range-feedback inhibition alone against inhibition followed by gravity stance.
Its [pilot](results/dougie_gravity_fallback_pilot_20261023.json) and frozen harness
revision `eddc379` are retained.

All eight fallback cases entered gravity stance at **30.91–32.36 s**, after the
first missing scan at approximately 30.2 s. All range-derived velocity corrections
remained withheld. Gravity support continues joint actuation using proprioception;
it does not restore unobservable position feedback.

- Inhibition-only drift: **1.315–1.522 m** from the original stop request.
- With gravity fallback: **.078–.142 m** from the original stop request, including
  the earlier active-hold period and the wait for supported entry.
- Additional drift **after gravity stance starts**: **.0058–.0132 m**.
- No stance refusals, simulator errors or non-foot contacts in the eight fallback cases.

The audit checks declared case completeness, matched approaches, post-dropout
commands and actual active-stance trace samples. It does not infer stationary
success merely from a zero velocity command.

## Opt-in mission integration

`--terminal-hold gravity` is available through `python3 -m dougie_sim.radiation_run`
and the `dougie` CLI. It requires robust fused localization. The default remains
`--terminal-hold zero-policy`.

Gravity stance is requested **only after a terminal stop**. It does not create a
route, resume exploration, change contact limits, or turn a failed return into
mission success. Physics and detector exposure continue to the original deadline.
The module and selected terminal mode are included in uncertainty-profile source
and domain checks; the historical `7d035f8` calibration artifact does not qualify
this runtime.

### Matched facility experiment

Both runs used seed 4 / sensor seed 1, a 600 s limit, 10 uSv budget and .5 s fixed
planning delay. Both required the strict vertical round trip. The
[independent audit](results/dougie_terminal_hold_audit_20261023.json) confirms
**2383 identical pre-stop physical samples**, including joint poses and commands,
and the same terminal event at **119.195 s** (`return_route_disconnected`).

| Outcome | Zero-command policy | Gravity terminal stance |
|---|---|---|
| End time | 449.335 s, aborted by contact qualification | 600.005 s, original deadline |
| Sampled post-stop XY drift | **3.9346 m** | **.0410 m** |
| Non-foot impulse | .17211 Ns | **0 Ns** |
| Controller status | Zero command, physical downhill motion | Holding from 120.790 s through the deadline |
| Total true dose | .093286 uSv | .195986 uSv |
| True dose after terminal stop | .067475 uSv | .170175 uSv |
| Strict vertical mission | **Failed** | **Failed** |

Drift comes from the independent 20 Hz physical trace, measured from its first
post-stop sample; it is not a continuous-time bound. The gravity run retained
support for about **479 s** after entry. Its support term peaked at 6.505 Nm.
The differing durations and trajectories mean these doses are **not a matched-dose
comparison**. Holding position did not remove exposure; the full post-stop dose
was retained.

Both commands exited 1 under the unchanged round-trip gate. The gravity case
avoided the contact failure, but still had no upper/lower/home completion.
[Portable mission evidence](results/dougie_facility_terminal_hold_20261023.json) ·
[Actual native recording](media/dougie/facility-terminal-hold.gif) ·
[Media provenance](DOUGIE_MEDIA.md#gravity-stance-and-facility-terminal-recording--23-october-2026).

```bash
# The mission is expected to fail the round-trip gate while testing terminal posture.
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim \
  python3 -m dougie_sim.radiation_run --world facility --headless --fast \
  --duration 600 --dose-budget 10 --seed 4 --sensor-seed 1 --fixed-plan-delay .5 \
  --terminal-hold gravity --require-round-trip --record-demo --record-view routes \
  --output results/dougie/NEW_TERMINAL_HOLD
```

## Verification and remaining gates

**279 tests passed**, including force/moment balance, virtual-work torque sign,
invalid stance refusal, load transfer, persistent refusal, entry deadline,
simulation integration, and independent post-stop motion measurement.

The posture failure has a tested opt-in mitigation; **the facility navigation
failure remains**. Next work is robust support fusion and return-graph continuity,
without weakening step/contact admission, plus broader disturbance, loading and
traction tests before changing controller defaults. Local support/clearance and
a freshly validated return route remain necessary for any resumed motion.
The original 10° mezzanine and full-height facility are still not qualified for
sensor-only round-trip completion.
