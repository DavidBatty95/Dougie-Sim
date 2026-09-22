#!/usr/bin/env python3
"""Headless acceptance evaluation for the G1 stand/creep/stop policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import torch

import mjlab.tasks  # noqa: F401 - imports register the task
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.sensor import ContactSensor
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.utils.torch import configure_torch_backends

TASK_ID = "Mjlab-Velocity-Flat-StandCreepStop-Unitree-G1"
NAVIGATION_TASK_ID = "Mjlab-Velocity-Flat-NavigationFineTune-Unitree-G1"
REACTIVE_TASK_ID = "Mjlab-Velocity-Flat-ReactiveAvoidanceFineTune-Unitree-G1"
NATURAL_GAIT_TASK_ID = "Mjlab-Velocity-Flat-NaturalGaitFineTune-Unitree-G1"
PHASE_GAIT_TASK_ID = "Mjlab-Velocity-Flat-PhaseGaitFineTune-Unitree-G1"
PHASE_TURN_TASK_ID = "Mjlab-Velocity-Flat-PhaseTurnFineTune-Unitree-G1"
STEP_FOUNDATION_TASK_ID = "Mjlab-Velocity-Flat-StepFoundation-Unitree-G1"


@dataclass(frozen=True)
class Phase:
  name: str
  command: tuple[float, float, float]
  duration_s: float
  kind: str


class GaitMetricsAccumulator:
  """Accumulate physical gait measurements for one commanded movement phase."""

  def __init__(self, num_envs: int, num_feet: int, device: torch.device):
    shape = (num_envs, num_feet)
    self.contact_samples = torch.zeros(shape, device=device)
    self.double_support_samples = torch.zeros(num_envs, device=device)
    self.active_samples = torch.zeros(num_envs, device=device)
    self.samples = 0
    self.previous_contact: torch.Tensor | None = None
    self.last_touchdown_s = torch.full(shape, -1.0, device=device)
    self.touchdown_count = torch.zeros(shape, device=device)
    self.stride_interval_sum = torch.zeros(shape, device=device)
    self.stride_interval_sq_sum = torch.zeros(shape, device=device)
    self.stride_interval_count = torch.zeros(shape, device=device)
    self.last_touchdown_foot = torch.full(
      (num_envs,), -1, dtype=torch.long, device=device
    )
    self.alternating_touchdowns = torch.zeros(num_envs, device=device)
    self.touchdowns_after_first = torch.zeros(num_envs, device=device)
    self.swing_peak_height = torch.zeros(shape, device=device)
    self.swing_peak_sum = torch.zeros(shape, device=device)
    self.swing_peak_count = torch.zeros(shape, device=device)
    self.previous_scuff = torch.zeros(shape, dtype=torch.bool, device=device)
    self.toe_scuff_count = torch.zeros(shape, device=device)
    self.stance_slip_distance = torch.zeros(shape, device=device)
    self.touchdown_vertical_speed_sum = torch.zeros(shape, device=device)
    self.touchdown_vertical_speed_max = torch.zeros(shape, device=device)
    self.root_height_min = torch.full((num_envs,), float("inf"), device=device)
    self.root_height_max = torch.full((num_envs,), -float("inf"), device=device)
    self.gravity_xy_sq_sum = torch.zeros((num_envs, 2), device=device)
    self.previous_action: torch.Tensor | None = None
    self.previous_action_velocity: torch.Tensor | None = None
    self.previous_action_acceleration: torch.Tensor | None = None
    self.action_velocity_sq_sum = torch.zeros(num_envs, device=device)
    self.action_acceleration_sq_sum = torch.zeros(num_envs, device=device)
    self.action_jerk_sq_sum = torch.zeros(num_envs, device=device)
    self.action_velocity_samples = 0
    self.action_acceleration_samples = 0
    self.action_jerk_samples = 0

  def update(
    self,
    *,
    elapsed_s: float,
    step_dt: float,
    active: torch.Tensor,
    actions: torch.Tensor,
    in_contact: torch.Tensor,
    foot_height: torch.Tensor,
    foot_velocity: torch.Tensor,
    root_height: torch.Tensor,
    projected_gravity: torch.Tensor,
  ) -> None:
    active_feet = active[:, None]
    self.samples += 1
    self.active_samples += active
    self.contact_samples += active_feet * in_contact
    self.double_support_samples += active * in_contact.all(dim=1)
    self.root_height_min = torch.minimum(
      self.root_height_min, torch.where(active, root_height, self.root_height_min)
    )
    self.root_height_max = torch.maximum(
      self.root_height_max, torch.where(active, root_height, self.root_height_max)
    )
    self.gravity_xy_sq_sum += active[:, None] * projected_gravity[:, :2].square()

    foot_speed_xy = torch.linalg.vector_norm(foot_velocity[..., :2], dim=2)
    self.stance_slip_distance += (
      active_feet * in_contact * foot_speed_xy * step_dt
    )
    in_air = ~in_contact
    self.swing_peak_height = torch.where(
      active_feet & in_air,
      torch.maximum(self.swing_peak_height, foot_height),
      self.swing_peak_height,
    )
    scuffing = active_feet & in_air & (foot_height < 0.015) & (foot_speed_xy > 0.03)
    self.toe_scuff_count += scuffing & ~self.previous_scuff
    self.previous_scuff = scuffing

    if self.previous_contact is not None:
      first_contact = active_feet & in_contact & ~self.previous_contact
      for foot in range(in_contact.shape[1]):
        landed = first_contact[:, foot]
        previous_time = self.last_touchdown_s[:, foot]
        has_previous = landed & (previous_time >= 0.0)
        interval = elapsed_s - previous_time
        self.stride_interval_sum[:, foot] += has_previous * interval
        self.stride_interval_sq_sum[:, foot] += has_previous * interval.square()
        self.stride_interval_count[:, foot] += has_previous
        self.last_touchdown_s[:, foot] = torch.where(
          landed,
          torch.full_like(previous_time, elapsed_s),
          previous_time,
        )
        self.touchdown_count[:, foot] += landed
        had_touchdown = landed & (self.last_touchdown_foot >= 0)
        self.touchdowns_after_first += had_touchdown
        self.alternating_touchdowns += had_touchdown & (
          self.last_touchdown_foot != foot
        )
        self.last_touchdown_foot = torch.where(
          landed,
          torch.full_like(self.last_touchdown_foot, foot),
          self.last_touchdown_foot,
        )
        self.swing_peak_sum[:, foot] += landed * self.swing_peak_height[:, foot]
        self.swing_peak_count[:, foot] += landed
        self.swing_peak_height[:, foot] = torch.where(
          landed,
          torch.zeros_like(self.swing_peak_height[:, foot]),
          self.swing_peak_height[:, foot],
        )
        touchdown_speed = foot_velocity[:, foot, 2].abs()
        self.touchdown_vertical_speed_sum[:, foot] += landed * touchdown_speed
        self.touchdown_vertical_speed_max[:, foot] = torch.maximum(
          self.touchdown_vertical_speed_max[:, foot], landed * touchdown_speed
        )
    self.previous_contact = in_contact.clone()

    if self.previous_action is not None:
      action_velocity = (actions - self.previous_action) / step_dt
      self.action_velocity_sq_sum += active * action_velocity.square().mean(dim=1)
      self.action_velocity_samples += 1
      if self.previous_action_velocity is not None:
        action_acceleration = (
          action_velocity - self.previous_action_velocity
        ) / step_dt
        self.action_acceleration_sq_sum += (
          active * action_acceleration.square().mean(dim=1)
        )
        self.action_acceleration_samples += 1
        if self.previous_action_acceleration is not None:
          action_jerk = (
            action_acceleration - self.previous_action_acceleration
          ) / step_dt
          self.action_jerk_sq_sum += active * action_jerk.square().mean(dim=1)
          self.action_jerk_samples += 1
        self.previous_action_acceleration = action_acceleration
      self.previous_action_velocity = action_velocity
    self.previous_action = actions.clone()

  def result(self) -> dict[str, Any]:
    stride_count = self.stride_interval_count.clamp(min=1.0)
    stride_mean = self.stride_interval_sum / stride_count
    stride_variance = (
      self.stride_interval_sq_sum / stride_count - stride_mean.square()
    ).clamp(min=0.0)
    stride_cv = torch.where(
      self.stride_interval_count > 1.0,
      stride_variance.sqrt() / stride_mean.clamp(min=1.0e-6),
      torch.zeros_like(stride_mean),
    )
    touchdown_count = self.swing_peak_count.clamp(min=1.0)
    active_samples = self.active_samples.clamp(min=1.0)

    def rms(total: torch.Tensor, count: int) -> torch.Tensor:
      return (total / max(1, count)).sqrt()

    return {
      "touchdown_count_left_right": self.touchdown_count.cpu().tolist(),
      "stride_period_mean_s_left_right": stride_mean.cpu().tolist(),
      "stride_period_cv_left_right": stride_cv.cpu().tolist(),
      "alternating_touchdown_fraction": tensor_values(
        self.alternating_touchdowns / self.touchdowns_after_first.clamp(min=1.0)
      ),
      "duty_factor_left_right": (
        self.contact_samples / active_samples[:, None]
      ).cpu().tolist(),
      "double_support_fraction": tensor_values(
        self.double_support_samples / active_samples
      ),
      "mean_swing_peak_height_m_left_right": (
        self.swing_peak_sum / touchdown_count
      ).cpu().tolist(),
      "toe_scuff_count_left_right": self.toe_scuff_count.cpu().tolist(),
      "stance_slip_distance_m_left_right": (
        self.stance_slip_distance
      ).cpu().tolist(),
      "mean_touchdown_vertical_speed_mps_left_right": (
        self.touchdown_vertical_speed_sum / touchdown_count
      ).cpu().tolist(),
      "max_touchdown_vertical_speed_mps_left_right": (
        self.touchdown_vertical_speed_max
      ).cpu().tolist(),
      "root_height_excursion_m": tensor_values(
        torch.nan_to_num(
          self.root_height_max - self.root_height_min,
          nan=0.0,
          posinf=0.0,
          neginf=0.0,
        ).clamp(min=0.0)
      ),
      "projected_gravity_xy_rms": (
        self.gravity_xy_sq_sum.div(active_samples[:, None]).sqrt().cpu().tolist()
      ),
      "action_velocity_rms_per_s": tensor_values(
        rms(self.action_velocity_sq_sum, self.action_velocity_samples)
      ),
      "action_acceleration_rms_per_s2": tensor_values(
        rms(self.action_acceleration_sq_sum, self.action_acceleration_samples)
      ),
      "action_jerk_rms_per_s3": tensor_values(
        rms(self.action_jerk_sq_sum, self.action_jerk_samples)
      ),
    }


def command_schedule(task: str) -> list[Phase]:
  if task == PHASE_TURN_TASK_ID:
    return [
      Phase("zero_hold", (0.0, 0.0, 0.0), 10.0, "zero"),
      Phase("yaw_left", (0.0, 0.0, 0.20), 8.0, "move"),
      Phase("stop_after_yaw_left", (0.0, 0.0, 0.0), 3.0, "stop"),
      Phase("yaw_right", (0.0, 0.0, -0.20), 8.0, "move"),
      Phase("stop_after_yaw_right", (0.0, 0.0, 0.0), 3.0, "stop"),
    ]
  if task == REACTIVE_TASK_ID:
    phases = [Phase("zero_hold", (0.0, 0.0, 0.0), 10.0, "zero")]
    commands = (
      ("lateral_left_005", (0.0, 0.05, 0.0)),
      ("lateral_right_005", (0.0, -0.05, 0.0)),
      ("lateral_left_010", (0.0, 0.10, 0.0)),
      ("lateral_right_010", (0.0, -0.10, 0.0)),
      ("yaw_left_012", (0.0, 0.0, 0.12)),
      ("yaw_right_012", (0.0, 0.0, -0.12)),
      ("yaw_left_020", (0.0, 0.0, 0.20)),
      ("yaw_right_020", (0.0, 0.0, -0.20)),
      ("diagonal_arc_left", (0.08, 0.08, 0.12)),
      ("diagonal_arc_right", (0.08, -0.08, -0.12)),
      ("forward_010", (0.10, 0.0, 0.0)),
      ("forward_020", (0.20, 0.0, 0.0)),
    )
    for name, command in commands:
      phases.append(Phase(name, command, 5.0, "move"))
      phases.append(Phase(f"stop_after_{name}", (0.0, 0.0, 0.0), 3.0, "stop"))
    return phases
  if task in (NAVIGATION_TASK_ID, NATURAL_GAIT_TASK_ID, PHASE_GAIT_TASK_ID):
    phases = [Phase("zero_hold", (0.0, 0.0, 0.0), 10.0, "zero")]
    commands = (
      ("yaw_left_012", (0.0, 0.0, 0.12)),
      ("yaw_right_012", (0.0, 0.0, -0.12)),
      ("yaw_left_020", (0.0, 0.0, 0.20)),
      ("yaw_right_020", (0.0, 0.0, -0.20)),
      ("arc_left", (0.20, 0.0, 0.12)),
      ("arc_right", (0.20, 0.0, -0.12)),
      ("forward_005", (0.05, 0.0, 0.0)),
      ("forward_010", (0.10, 0.0, 0.0)),
      ("forward_020", (0.20, 0.0, 0.0)),
    )
    for name, command in commands:
      phases.append(Phase(name, command, 5.0, "move"))
      phases.append(Phase(f"stop_after_{name}", (0.0, 0.0, 0.0), 3.0, "stop"))
    return phases
  phases = [Phase("zero_hold", (0.0, 0.0, 0.0), 30.0, "zero")]
  commands = (
    ("forward_005", (0.05, 0.0, 0.0)),
    ("forward_010", (0.10, 0.0, 0.0)),
    ("forward_020", (0.20, 0.0, 0.0)),
    ("forward_035", (0.35, 0.0, 0.0)),
    ("reverse_005", (-0.05, 0.0, 0.0)),
    ("reverse_010", (-0.10, 0.0, 0.0)),
    ("yaw_left", (0.0, 0.0, 0.20)),
    ("yaw_right", (0.0, 0.0, -0.20)),
  )
  for name, command in commands:
    phases.append(Phase(name, command, 5.0, "move"))
    phases.append(Phase(f"stop_after_{name}", (0.0, 0.0, 0.0), 3.0, "stop"))
  return phases


def force_command(term: Any, command: torch.Tensor) -> None:
  term.vel_command_b.copy_(command)
  term.vel_command_w.copy_(command)
  term.is_standing_env.fill_(False)
  term.is_heading_env.fill_(False)
  term.is_world_env.fill_(False)
  term.is_forward_env.fill_(False)
  term.time_left.fill_(float("inf"))


def tensor_values(value: torch.Tensor) -> list[float]:
  return [float(item) for item in value.detach().cpu().tolist()]


def file_sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(block)
  return digest.hexdigest()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--checkpoint", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument(
    "--task",
    choices=(
      TASK_ID,
      NAVIGATION_TASK_ID,
      REACTIVE_TASK_ID,
      NATURAL_GAIT_TASK_ID,
      PHASE_GAIT_TASK_ID,
      PHASE_TURN_TASK_ID,
      STEP_FOUNDATION_TASK_ID,
    ),
    default=TASK_ID,
  )
  parser.add_argument("--num-envs", type=int, default=1)
  parser.add_argument("--seed", type=int, default=42)
  parser.add_argument("--device", default=None)
  parser.add_argument(
    "--mode",
    choices=("nominal", "randomized"),
    default="nominal",
    help="Nominal disables startup dynamics randomization.",
  )
  parser.add_argument("--max-drift", type=float, default=0.10)
  parser.add_argument("--max-stop-speed", type=float, default=0.03)
  parser.add_argument("--max-foot-speed", type=float, default=0.05)
  parser.add_argument("--max-action", type=float, default=1.0)
  parser.add_argument("--linear-error-floor", type=float, default=0.04)
  parser.add_argument("--yaw-error-floor", type=float, default=0.05)
  parser.add_argument("--max-axis-leakage", type=float, default=0.04)
  parser.add_argument("--tracking-error-fraction", type=float, default=0.25)
  args = parser.parse_args()
  if args.num_envs < 1:
    parser.error("--num-envs must be positive")
  return args


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
  checkpoint = args.checkpoint.resolve()
  if not checkpoint.is_file():
    raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")

  configure_torch_backends(allow_tf32=False, deterministic=True)
  device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
  env_cfg = load_env_cfg(args.task, play=True)
  agent_cfg = load_rl_cfg(args.task)
  phases = command_schedule(args.task)
  total_duration = sum(phase.duration_s for phase in phases)

  env_cfg.seed = args.seed
  env_cfg.scene.num_envs = args.num_envs
  env_cfg.episode_length_s = total_duration + 1.0
  env_cfg.auto_reset = True
  env_cfg.events["reset_base"].params["pose_range"] = {}
  env_cfg.events["reset_base"].params["velocity_range"] = {}
  if args.mode == "nominal":
    for event in ("foot_friction", "encoder_bias", "base_com", "randomize_terrain"):
      env_cfg.events.pop(event, None)

  raw_env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=None)
  env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
  runner_cls = load_runner_cls(args.task) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=device)
  runner.load(
    str(checkpoint),
    load_cfg={"actor": True},
    strict=True,
    map_location=device,
  )
  policy = runner.get_inference_policy(device=device)

  base = env.unwrapped
  robot = base.scene["robot"]
  twist = base.command_manager.get_term("twist")
  feet_contact = base.scene["feet_ground_contact"]
  if not isinstance(feet_contact, ContactSensor):
    raise TypeError("feet_ground_contact is not a ContactSensor")
  foot_ids, _ = robot.find_sites(("left_foot", "right_foot"), preserve_order=True)
  foot_height_sensor = base.scene["foot_height_scan"]
  joint_action = base.action_manager.get_term("joint_pos")
  num_envs = args.num_envs
  dev = torch.device(device)

  fallen = torch.zeros(num_envs, dtype=torch.bool, device=dev)
  nonfinite_action = torch.zeros_like(fallen)
  clipped_action = torch.zeros_like(fallen)
  soft_limit_request = torch.zeros_like(fallen)
  max_action = torch.zeros(num_envs, device=dev)
  max_tilt_deg = torch.zeros(num_envs, device=dev)
  initial_xy = robot.data.root_link_pos_w[:, :2].clone()
  zero_max_drift = torch.zeros(num_envs, device=dev)
  zero_foot_motion = torch.zeros_like(fallen)
  zero_foot_air = torch.zeros_like(fallen)
  phase_stats: dict[str, dict[str, torch.Tensor]] = {}

  command = torch.zeros((num_envs, 3), device=dev)
  force_command(twist, command)
  base.observation_manager._obs_buffer = None
  obs = env.get_observations()

  try:
    for phase in phases:
      command[:] = torch.tensor(phase.command, device=dev)
      force_command(twist, command)
      base.observation_manager._obs_buffer = None
      obs = env.get_observations()
      steps = round(phase.duration_s / base.step_dt)
      stats = {
        "tracking_error_sum": torch.zeros(num_envs, device=dev),
        "tracking_samples": torch.zeros(num_envs, device=dev),
        "max_settled_speed": torch.zeros(num_envs, device=dev),
        "foot_motion": torch.zeros(num_envs, dtype=torch.bool, device=dev),
        "foot_air": torch.zeros(num_envs, dtype=torch.bool, device=dev),
        "max_foot_speed": torch.zeros(num_envs, device=dev),
        "max_axis_leakage": torch.zeros(num_envs, device=dev),
        "gait": (
          GaitMetricsAccumulator(num_envs, len(foot_ids), dev)
          if phase.kind == "move"
          else None
        ),
      }

      for step in range(steps):
        force_command(twist, command)
        with torch.inference_mode():
          actions = policy(obs)
        finite = torch.isfinite(actions).all(dim=1)
        nonfinite_action |= ~finite
        safe_actions = torch.nan_to_num(actions)
        action_magnitude = safe_actions.abs().amax(dim=1)
        max_action = torch.maximum(max_action, action_magnitude)
        clipped_action |= action_magnitude > args.max_action
        obs, _, _, _ = env.step(safe_actions)

        fell_now = base.termination_manager.get_term("fell_over")
        fallen |= fell_now
        active = ~fallen
        gravity_z = -robot.data.projected_gravity_b[:, 2].clamp(-1.0, 1.0)
        tilt = torch.rad2deg(torch.acos(gravity_z))
        max_tilt_deg = torch.maximum(max_tilt_deg, tilt)

        targets = robot.data.joint_pos_target[:, joint_action.target_ids]
        limits = robot.data.soft_joint_pos_limits[:, joint_action.target_ids]
        outside = (targets < limits[..., 0]) | (targets > limits[..., 1])
        soft_limit_request |= outside.any(dim=1)

        foot_velocity = robot.data.site_lin_vel_w[:, foot_ids, :]
        foot_speed = torch.linalg.vector_norm(foot_velocity, dim=-1).amax(dim=1)
        found = feet_contact.data.found
        if found is None:
          raise RuntimeError("Contact sensor did not provide the 'found' field")
        in_contact = found.reshape(num_envs, len(foot_ids), -1).any(dim=2)
        any_foot_air = ~in_contact.all(dim=1)
        elapsed = (step + 1) * base.step_dt
        if phase.kind == "move":
          stats["max_foot_speed"] = torch.maximum(
            stats["max_foot_speed"], active * foot_speed
          )
          stats["foot_air"] |= active & any_foot_air
          gait = stats["gait"]
          assert isinstance(gait, GaitMetricsAccumulator)
          gait.update(
            elapsed_s=elapsed,
            step_dt=base.step_dt,
            active=active,
            actions=safe_actions,
            in_contact=in_contact,
            foot_height=foot_height_sensor.data.heights,
            foot_velocity=foot_velocity,
            root_height=robot.data.root_link_pos_w[:, 2],
            projected_gravity=robot.data.projected_gravity_b,
          )

        if phase.kind == "zero" and elapsed <= 10.0:
          drift = torch.linalg.vector_norm(
            robot.data.root_link_pos_w[:, :2] - initial_xy, dim=1
          )
          zero_max_drift = torch.maximum(zero_max_drift, drift)
        if phase.kind == "zero" and elapsed >= 2.0:
          zero_foot_motion |= active & (foot_speed > args.max_foot_speed)
          zero_foot_air |= active & any_foot_air

        if phase.kind == "move" and elapsed >= phase.duration_s - 2.0:
          pure_yaw = phase.command[:2] == (0.0, 0.0) and phase.command[2] != 0.0
          pure_lateral = (
            phase.command[0] == 0.0
            and phase.command[1] != 0.0
            and phase.command[2] == 0.0
          )
          if pure_yaw:
            actual = robot.data.root_link_ang_vel_b[:, 2]
            target = phase.command[2]
            leakage = torch.linalg.vector_norm(
              robot.data.root_link_lin_vel_b[:, :2], dim=1
            )
          elif pure_lateral:
            actual = robot.data.root_link_lin_vel_b[:, 1]
            target = phase.command[1]
            leakage = torch.maximum(
              robot.data.root_link_lin_vel_b[:, 0].abs(),
              robot.data.root_link_ang_vel_b[:, 2].abs(),
            )
          else:
            actual = robot.data.root_link_lin_vel_b[:, 0]
            target = phase.command[0]
            leakage = torch.zeros(num_envs, device=dev)
          stats["tracking_error_sum"] += active * (actual - target).abs()
          stats["tracking_samples"] += active
          stats["max_axis_leakage"] = torch.maximum(
            stats["max_axis_leakage"], active * leakage
          )

        if phase.kind == "stop" and elapsed >= 1.0:
          planar_speed = torch.linalg.vector_norm(
            robot.data.root_link_lin_vel_b[:, :2], dim=1
          )
          stop_speed = torch.maximum(
            planar_speed, robot.data.root_link_ang_vel_b[:, 2].abs()
          )
          stats["max_settled_speed"] = torch.maximum(
            stats["max_settled_speed"], active * stop_speed
          )
          stats["foot_motion"] |= active & (foot_speed > args.max_foot_speed)
          stats["foot_air"] |= active & any_foot_air

      phase_stats[phase.name] = stats

    checks: dict[str, list[bool]] = {
      "no_falls": (~fallen).cpu().tolist(),
      "zero_drift": (zero_max_drift <= args.max_drift).cpu().tolist(),
      "zero_feet_planted": (~(zero_foot_motion | zero_foot_air)).cpu().tolist(),
      "finite_actions": (~nonfinite_action).cpu().tolist(),
      "actions_within_deployment_clip": (~clipped_action).cpu().tolist(),
      "targets_within_soft_limits": (~soft_limit_request).cpu().tolist(),
    }
    phase_results: dict[str, Any] = {}
    for phase in phases:
      stats = phase_stats[phase.name]
      if phase.kind == "move":
        samples = stats["tracking_samples"].clamp(min=1.0)
        error = stats["tracking_error_sum"] / samples
        target = max(abs(value) for value in phase.command)
        floor = (
          args.yaw_error_floor
          if abs(phase.command[2]) > 0.0
          else args.linear_error_floor
        )
        threshold = max(floor, args.tracking_error_fraction * target)
        passed = (stats["tracking_samples"] > 0) & (error <= threshold)
        checks[f"tracking_{phase.name}"] = passed.cpu().tolist()
        pure_axis = (phase.command[:2] == (0.0, 0.0) and phase.command[2] != 0.0) or (
          phase.command[0] == 0.0
          and phase.command[1] != 0.0
          and phase.command[2] == 0.0
        )
        if pure_axis:
          leakage_ok = stats["max_axis_leakage"] <= args.max_axis_leakage
          checks[f"axis_leakage_{phase.name}"] = leakage_ok.cpu().tolist()
        phase_results[phase.name] = {
          "mean_absolute_tracking_error": tensor_values(error),
          "threshold": threshold,
          "max_foot_speed": tensor_values(stats["max_foot_speed"]),
          "foot_became_airborne": stats["foot_air"].cpu().tolist(),
          "max_axis_leakage": tensor_values(stats["max_axis_leakage"]),
          "gait": stats["gait"].result(),
        }
      elif phase.kind == "stop":
        speed_ok = stats["max_settled_speed"] <= args.max_stop_speed
        feet_ok = ~(stats["foot_motion"] | stats["foot_air"])
        checks[f"settled_{phase.name}"] = speed_ok.cpu().tolist()
        checks[f"feet_planted_{phase.name}"] = feet_ok.cpu().tolist()
        phase_results[phase.name] = {
          "max_speed_after_one_second": tensor_values(stats["max_settled_speed"]),
          "speed_threshold": args.max_stop_speed,
        }

    env_pass = [
      all(values[index] for values in checks.values()) for index in range(num_envs)
    ]
    result: dict[str, Any] = {
      "schema_version": 1,
      "metric_contract": "g1_gait_quality_v1",
      "status": "PASS" if all(env_pass) else "FAIL",
      "meaning": "Status reflects the defined gates, not global policy optimality.",
      "checkpoint": str(checkpoint),
      "checkpoint_sha256": file_sha256(checkpoint),
      "task": args.task,
      "mode": args.mode,
      "seed": args.seed,
      "num_envs": num_envs,
      "duration_s": total_duration,
      "pass_rate": sum(env_pass) / num_envs,
      "environment_pass": env_pass,
      "checks": checks,
      "metrics": {
        "zero_max_drift_m": tensor_values(zero_max_drift),
        "max_action_magnitude": tensor_values(max_action),
        "max_tilt_deg": tensor_values(max_tilt_deg),
        "phases": phase_results,
      },
      "thresholds": {
        "max_drift_m": args.max_drift,
        "max_stop_speed_mps_or_radps": args.max_stop_speed,
        "max_foot_speed_mps": args.max_foot_speed,
        "max_action_magnitude": args.max_action,
        "linear_error_floor_mps": args.linear_error_floor,
        "yaw_error_floor_radps": args.yaw_error_floor,
        "tracking_error_fraction": args.tracking_error_fraction,
        "max_axis_leakage_mps_or_radps": args.max_axis_leakage,
      },
    }
    return result
  finally:
    env.close()


def print_summary(result: dict[str, Any]) -> None:
  print("\nG1 policy stability acceptance")
  print("=" * 40)
  print(f"Result:    {result['status']}")
  print(f"Mode:      {result['mode']}")
  print(f"Pass rate: {result['pass_rate']:.1%}")
  failed = [name for name, values in result["checks"].items() if not all(values)]
  if failed:
    print("Failed checks:")
    for name in failed:
      failures = sum(not value for value in result["checks"][name])
      print(f"  - {name}: {failures}/{result['num_envs']} environments")
  else:
    print("All acceptance checks passed.")
  print(f"Report:    {result['output_file']}")
  print("PASS means the defined gates passed, not that the policy is globally optimal.")


def main() -> int:
  args = parse_args()
  try:
    result = evaluate(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result["output_file"] = str(args.output.resolve())
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print_summary(result)
    return 0 if result["status"] == "PASS" else 1
  except Exception as error:
    print(f"Evaluation error: {error}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
