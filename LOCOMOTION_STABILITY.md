# G1 Locomotion Stability

## Diagnosis

Recorded failures have multiple causes. Several runs fall during startup or while
the requested velocity is zero, before navigation can be the cause. The previous
controller also ramped stale and zero commands slowly, could continue inference
from stale state, and did not refresh the last target while inference was pending.

The automatically selected flat policy was trained over a much broader command
range than ROS uses. It received relatively few exact standing commands and had no
explicit stop-transition objective. More iterations of that unchanged task are
therefore unlikely to fix low-speed movement and stopping reliably.

## Deployment Safeguards

The ROS controller now:

- Stops immediately for stale, zero, or deadband commands.
- Uses a deadband with hysteresis around the policy's standing threshold.
- Rejects stale low-state data and zeros stale odometry observations.
- Refreshes the last safe target while inference is pending.
- Uses a 50 ms inference deadline and conservative action clipping.
- Clamps targets to conservative G1 joint soft limits.
- Rejects incompatible policy joint metadata instead of guessing the order.
- Uses a longer startup hold, blend, upright hold, and action ramp.
- Never releases the walking policy merely because an upright timeout elapsed.

These changes prevent command runaway and reduce deployment-induced falls. They do
not make an inadequately trained policy stable.

## Stability Training Task

Use the all-in-one launcher for the dedicated full-body task. It builds the
container, synchronizes MJLab dependencies, checks the GPU, and starts a fresh
training run:

```bash
./scripts/launch_g1_policy_training.sh
```

Use `--resume` to continue the latest checkpoint. After the first launch,
`--skip-build --skip-sync` starts training immediately when neither the image nor
the dependencies changed.

For a short functional training check:

```bash
MJLAB_NUM_ENVS=256 MJLAB_AUTO_RESUME=0 \
  ./scripts/launch_g1_policy_training.sh --skip-build --skip-sync \
  --agent.max-iterations 10
```

The task preserves the deployed 99-observation, 29-action contract. It uses:

- 38% exact standing commands.
- Frequent moving-to-zero transitions.
- Concentrated 0.03-0.15 m/s creep commands.
- Moderate commands capped at the ROS envelope.
- Explicit planted-foot and low-joint-motion standing costs.
- Tighter low-speed tracking and stronger action smoothing.
- No broad high-speed curriculum or severe early pushes.

Runs are isolated under the `g1_stand_creep_stop` experiment so they cannot resume
an older broad-speed checkpoint accidentally.

## Playback

Inspect the latest checkpoint with:

```bash
./scripts/mjlab_play_g1_stand_creep_stop.sh latest
```

Playback is a qualitative check, not an acceptance test. Evaluate at least these
command sequences from multiple gait phases:

1. Zero command for 10 seconds.
2. Forward commands of 0.05, 0.10, 0.20, and 0.35 m/s.
3. Reverse command of -0.05 and -0.10 m/s.
4. Return to zero after every moving command.
5. Small yaw commands followed by zero.

## Movement Fine-Tuning

After the reverse/yaw stage saves `model_5597.pt`, run one final yaw/action
polish stage with:

```bash
./scripts/mjlab_continue_g1_from_5597.sh
```

This selects exactly `model_5597.pt`, keeps the 99-observation/29-action policy
contract, resets the optimizer, and trains 200 additional iterations to about
iteration 5797. It concentrates sampling on yaw and reverse creep, adds an
absolute action-magnitude cost, and retains the forward and stopping objectives.
After this short stage, freeze locomotion and progress social-navigation learning
at the candidate-selection layer rather than continuing joint-policy tuning.

For a controlled yaw experiment that retains the 99-observation policy contract,
starts from the established navigation checkpoint, and includes bilateral pure-yaw
and deployment-speed arc commands, run:

```bash
./scripts/mjlab_train_g1_yaw_finetune.sh
```

This conservative stage uses a lower learning rate and retains forward and stop
samples to reduce catastrophic forgetting. Evaluate every saved checkpoint rather
than assuming the final checkpoint is best. It defaults to a 50-iteration pilot;
continue from an accepted checkpoint explicitly instead of extending a failed run.

## Acceptance Gates

Do not deploy a checkpoint based only on training reward. Across multiple seeds,
require:

- No falls during a 30-second zero-command hold.
- Less than 0.10 m drift during a 10-second zero-command hold.
- Reliable movement at 0.05 and 0.10 m/s.
- Settling below 0.03 m/s within 1.0 second after a stop command.
- No foot stepping after settling unless balance requires it.
- No saturated or soft-limit-clamped joint targets in nominal operation.
- No inference deadline, stale-state, or metadata faults.

Run the nominal automated acceptance suite against the latest checkpoint with:

```bash
./scripts/mjlab_evaluate_g1_stability.sh latest
```

After nominal evaluation passes, exercise the retained dynamics randomization
across multiple parallel environments:

```bash
./scripts/mjlab_evaluate_g1_stability.sh latest \
  --mode randomized --num-envs 64
```

The evaluator writes a JSON report beside the checkpoint and exits with status 0
on pass, 1 on a failed gate, or 2 on an execution error. A pass means that the
defined acceptance gates succeeded; it does not prove global policy optimality.

Evaluate nominal dynamics first, then progressively add mild pushes and deployment-
centered dynamics randomization. Retain zero and creep samples during every later
curriculum stage to prevent catastrophic forgetting.

## ROS Deployment

Use an explicit exported ONNX path. Do not use `policy_path:=latest` for controlled
evaluation because modification time is not a quality metric:

```bash
./scripts/bringup_full_sim.sh \
  policy_path:=/workspace/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop/\
<run>/<export>.onnx
```

The default simulation currently selects the validated `model_5597` export at:

```text
/workspace/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop/
2026-08-03_13-14-21_g1_stand_creep_stop_reverse_yaw_from_5298/
2026-08-03_13-14-21_g1_stand_creep_stop_reverse_yaw_from_5298.onnx
```

Confirm the startup log reports `obs_dim=99`, `action_dim=29`, velocity commands,
and the expected joint metadata before enabling autonomous goals.

## Fall Recovery

The current tilt guard and bridge damping are fall protection, not get-up control.
A separately trained get-up task exists, but automatic policy switching is not yet
implemented. Safe integration requires an explicit state machine that isolates
walking, damping, get-up, stable-upright dwell, and zero-command walking restart.
Until that exists, a fallen robot requires reset or operator intervention.
