#!/usr/bin/env python3
"""Expand a 99-input G1 checkpoint with neutral gait-phase inputs."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from mjlab.rl.checkpoint_migration import migrate_phase_gait_checkpoint


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  parser.add_argument("output", type=Path)
  args = parser.parse_args()

  checkpoint = torch.load(args.source, map_location="cpu", weights_only=False)
  migrated = migrate_phase_gait_checkpoint(checkpoint)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  torch.save(migrated, args.output)
  actor_inputs = migrated["actor_state_dict"]["mlp.0.weight"].shape[1]
  critic_inputs = migrated["critic_state_dict"]["mlp.0.weight"].shape[1]
  print(f"Wrote {args.output} with actor={actor_inputs}, critic={critic_inputs}")


if __name__ == "__main__":
  main()
