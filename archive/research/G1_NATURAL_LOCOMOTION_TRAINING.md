# G1 Natural Locomotion Training

> Archived promotion plan. Named local checkpoints are not distributed with the repository.
> See `docs/TRAINING_AND_CALIBRATION.md` for the maintained handoff status.

This runbook defines the promotion path for improved stepping, turning, and
sidestepping. The official Unitree 480-observation ONNX remains the deployed
baseline, but it is not a resumable PPO checkpoint. New policies are trained
with the 99- or 101-observation MJLab contracts and must pass every earlier
stage before deployment.

## Evidence Rules

- Select checkpoints using development and calibration seeds only.
- Evaluate a frozen candidate once on held-out seeds.
- Preserve falls, clipping, poor tracking, and limit violations as outcomes.
- Record the parent checkpoint hash, training seed, code/config hashes, exact
  command, environment count, and iterations in a training manifest.
- Do not promote a checkpoint based on modification time or reward alone.
- Do not enable lateral commands in ROS until the sidestep stage passes.

## Stage 0: Measurement and Provenance

`evaluate_g1_stability.py` records tracking and safety gates plus the
`g1_gait_quality_v1` diagnostic contract:

- touchdown count and alternation;
- stride period and coefficient of variation per foot;
- duty factor and double-support fraction;
- swing peak height and toe-scuff count;
- stance slip distance;
- touchdown vertical speed;
- root-height excursion and projected-gravity sway;
- action velocity, acceleration, and jerk.

The gait metrics remain diagnostic until baseline distributions are collected.
Thresholds must be frozen before final policy evaluation.

Every non-dry-run fine-tune writes an atomic manifest under
`vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop/training_manifests/`.

## Stage 1: Step Foundation

Task: `Mjlab-Velocity-Flat-StepFoundation-Unitree-G1`

The task rehearses standing, stopping, low-speed forward/reverse motion, and
bilateral low-rate yaw. Runner action clipping is disabled so the environment's
action-excess reward observes true actor output; joint targets remain clipped to
soft limits.

The existing `model_6595.pt` is only a bootstrap source. It is not an accepted
parent: its nominal report records a raw action magnitude of 1.855 and multiple
yaw and stop failures.

Run three development seeds from the same immutable source:

```bash
SOURCE_RUN=2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796
for SEED in 101 103 107; do
  ./scripts/mjlab_train_g1_step_foundation.sh \
    --source-run "$SOURCE_RUN" \
    --checkpoint model_6595.pt \
    --seed "$SEED" \
    --num-envs 4096 \
    --iterations 3000
```

Evaluate intermediate and final checkpoints on development seeds before
selecting candidates. Use explicit mode and seed arguments; non-default reports
receive unique mode/seed filenames and do not overwrite one another.

```bash
./scripts/mjlab_evaluate_g1_stability.sh CHECKPOINT \
  --task Mjlab-Velocity-Flat-StepFoundation-Unitree-G1 \
  --mode nominal --seed 211 --num-envs 64

./scripts/mjlab_evaluate_g1_stability.sh CHECKPOINT \
  --task Mjlab-Velocity-Flat-StepFoundation-Unitree-G1 \
  --mode randomized --seed 211 --num-envs 64
```

Promotion requires finite actions, no nominal falls, no target-limit requests,
stable standing and stopping, bilateral yaw execution, forward/reverse tracking,
and actor outputs inside the deployment envelope. Gait diagnostics are compared
against the bootstrap checkpoint without collapsing them into one score.

## Remaining Stages

1. Natural-gait ablation: non-phase, fixed-phase, and speed-adaptive phase.
2. Bilateral pure-yaw and arc training without forced forward-turn floors.
3. Bilateral pure lateral steps, then diagonal and lateral-yaw commands.
4. Mixed-command and emergency-stop transition training.
5. Dynamics, latency, sensor, actuator, push, and mild-terrain randomization.
6. Mandatory ONNX equivalence, metadata, and policy-manifest promotion.
7. Standalone ROS/DDS MuJoCo response calibration.
8. Paired waypoint and pedestrian-navigation evaluation.

Each stage begins only after the preceding checkpoint has passed its frozen
promotion contract. Earlier command families remain in every later training and
evaluation schedule to detect catastrophic forgetting.
