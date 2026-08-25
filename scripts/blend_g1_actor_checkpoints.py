#!/usr/bin/env python3
"""Blend actor parameters from two compatible G1 checkpoints."""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import torch


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("base", type=Path)
  parser.add_argument("source", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument("--source-weight", type=float, required=True)
  args = parser.parse_args()
  if not 0.0 <= args.source_weight <= 1.0:
    parser.error("--source-weight must be between 0 and 1")

  base = torch.load(args.base, map_location="cpu", weights_only=False)
  source = torch.load(args.source, map_location="cpu", weights_only=False)
  output = copy.deepcopy(base)
  for key, base_value in base["actor_state_dict"].items():
    source_value = source["actor_state_dict"].get(key)
    if source_value is None or source_value.shape != base_value.shape:
      raise ValueError(f"Incompatible actor parameter: {key}")
    if key.startswith("mlp."):
      output["actor_state_dict"][key] = torch.lerp(
        base_value, source_value, args.source_weight
      )
  output["optimizer_state_dict"] = {}
  output["iter"] = 0
  output["infos"] = {
    "actor_blend_base": str(args.base),
    "actor_blend_source": str(args.source),
    "actor_blend_source_weight": args.source_weight,
  }
  args.output.parent.mkdir(parents=True, exist_ok=True)
  torch.save(output, args.output)
  print(f"Saved actor blend to {args.output}")


if __name__ == "__main__":
  main()
