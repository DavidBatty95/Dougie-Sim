#!/usr/bin/env python3
"""Distill a G1 motion-tracking policy into a phase velocity policy."""

from __future__ import annotations

import argparse
import copy
import math
from dataclasses import dataclass
from pathlib import Path

import torch
from torch import nn

import mjlab.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from mjlab.tasks.tracking.mdp import MotionCommand, MotionCommandCfg
from mjlab.utils.lab_api.math import quat_apply_inverse
from mjlab.utils.torch import configure_torch_backends

TRACKING_TASK = "Mjlab-Tracking-Flat-Unitree-G1"


class CheckpointActor(nn.Module):
  """Deterministic actor reconstructed from an RSL-RL checkpoint."""

  def __init__(self, state: dict[str, torch.Tensor], device: torch.device):
    super().__init__()
    dims = [state["mlp.0.weight"].shape[1], 512, 256, 128, 29]
    self.mlp = nn.Sequential(
      nn.Linear(dims[0], dims[1]),
      nn.ELU(),
      nn.Linear(dims[1], dims[2]),
      nn.ELU(),
      nn.Linear(dims[2], dims[3]),
      nn.ELU(),
      nn.Linear(dims[3], dims[4]),
    ).to(device)
    self.mean = state["obs_normalizer._mean"].to(device)
    self.std = state["obs_normalizer._std"].to(device).clamp_min(1.0e-6)
    mlp_state = {key.removeprefix("mlp."): value for key, value in state.items() if key.startswith("mlp.")}
    self.mlp.load_state_dict(mlp_state)

  def forward(self, obs: torch.Tensor) -> torch.Tensor:
    return self.mlp((obs - self.mean) / self.std)


@dataclass
class DistillationStats:
  loss_sum: float = 0.0
  action_error_sum: float = 0.0
  steps: int = 0


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--teacher-checkpoint", type=Path, required=True)
  parser.add_argument("--student-checkpoint", type=Path, required=True)
  parser.add_argument("--motion-file", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--steps", type=int, default=10_000)
  parser.add_argument("--learning-rate", type=float, default=3.0e-5)
  parser.add_argument("--gait-cycle", type=float, default=0.8)
  parser.add_argument("--seed", type=int, default=42)
  return parser.parse_args()


def build_student_observation(
  env: ManagerBasedRlEnv,
  command: MotionCommand,
  gait_cycle: float,
) -> torch.Tensor:
  robot = env.scene["robot"]
  reference_lin_vel_b = quat_apply_inverse(
    command.anchor_quat_w, command.anchor_lin_vel_w
  )
  reference_ang_vel_b = quat_apply_inverse(
    command.anchor_quat_w, command.anchor_ang_vel_w
  )
  twist = torch.stack(
    (reference_lin_vel_b[:, 0], reference_lin_vel_b[:, 1], reference_ang_vel_b[:, 2]),
    dim=1,
  )
  phase = command.time_steps.float() * env.step_dt * (2.0 * math.pi / gait_cycle)
  gait_phase = torch.stack((torch.sin(phase), torch.cos(phase)), dim=1)
  return torch.cat(
    (
      robot.data.root_link_lin_vel_b,
      robot.data.root_link_ang_vel_b,
      robot.data.projected_gravity_b,
      robot.data.joint_pos - robot.data.default_joint_pos,
      robot.data.joint_vel - robot.data.default_joint_vel,
      env.action_manager.action,
      twist,
      gait_phase,
    ),
    dim=1,
  )


def main() -> None:
  args = parse_args()
  for path in (args.teacher_checkpoint, args.student_checkpoint, args.motion_file):
    if not path.is_file():
      raise FileNotFoundError(path)
  if args.num_envs < 1 or args.steps < 1 or args.gait_cycle <= 0.0:
    raise ValueError("num_envs, steps, and gait_cycle must be positive")

  configure_torch_backends()
  if not torch.cuda.is_available():
    raise RuntimeError("Distillation requires CUDA")
  device = torch.device("cuda:0")

  env_cfg = load_env_cfg(TRACKING_TASK)
  agent_cfg = load_rl_cfg(TRACKING_TASK)
  motion_cfg = env_cfg.commands["motion"]
  if not isinstance(motion_cfg, MotionCommandCfg):
    raise TypeError("Tracking task does not contain a motion command")
  motion_cfg.motion_file = str(args.motion_file.resolve())
  env_cfg.scene.num_envs = args.num_envs
  env_cfg.seed = args.seed

  raw_env = ManagerBasedRlEnv(cfg=env_cfg, device=str(device))
  env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
  command = raw_env.command_manager.get_term("motion")
  if not isinstance(command, MotionCommand):
    raise TypeError("Resolved command is not MotionCommand")

  teacher_checkpoint = torch.load(
    args.teacher_checkpoint, map_location="cpu", weights_only=False
  )
  student_checkpoint = torch.load(
    args.student_checkpoint, map_location="cpu", weights_only=False
  )
  teacher = CheckpointActor(teacher_checkpoint["actor_state_dict"], device).eval()
  student = CheckpointActor(student_checkpoint["actor_state_dict"], device).train()
  optimizer = torch.optim.Adam(student.mlp.parameters(), lr=args.learning_rate)
  stats = DistillationStats()
  obs = env.get_observations()

  try:
    for step in range(args.steps):
      with torch.no_grad():
        teacher_actions = teacher(obs["actor"]).clamp(-1.0, 1.0)
        student_obs = build_student_observation(raw_env, command, args.gait_cycle)
      student_actions = student(student_obs)
      loss = torch.nn.functional.smooth_l1_loss(student_actions, teacher_actions)
      optimizer.zero_grad(set_to_none=True)
      loss.backward()
      torch.nn.utils.clip_grad_norm_(student.mlp.parameters(), 1.0)
      optimizer.step()
      with torch.no_grad():
        error = (student_actions - teacher_actions).abs().mean()
        stats.loss_sum += loss.item()
        stats.action_error_sum += error.item()
        stats.steps += 1
        obs, _, _, _ = env.step(teacher_actions)
      if (step + 1) % 500 == 0:
        print(
          f"step={step + 1} loss={stats.loss_sum / stats.steps:.6f} "
          f"mean_abs_action_error={stats.action_error_sum / stats.steps:.6f}"
        )
        stats = DistillationStats()
  finally:
    env.close()

  output_checkpoint = copy.deepcopy(student_checkpoint)
  trained_state = student.mlp.state_dict()
  for key, value in trained_state.items():
    output_checkpoint["actor_state_dict"][f"mlp.{key}"] = value.cpu()
  output_checkpoint["optimizer_state_dict"] = {}
  output_checkpoint["iter"] = 0
  output_checkpoint["infos"] = {
    "distilled_from": str(args.teacher_checkpoint),
    "motion_file": str(args.motion_file),
    "distillation_steps": args.steps,
  }
  args.output.parent.mkdir(parents=True, exist_ok=True)
  torch.save(output_checkpoint, args.output)
  print(f"Saved distilled phase velocity checkpoint to {args.output}")


if __name__ == "__main__":
  main()
