#!/usr/bin/env python3
"""Write immutable intent and provenance for one MJLab training run."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


TRACKED_TRAINING_FILES = (
    "vendor/mjlab/src/mjlab/tasks/velocity/velocity_env_cfg.py",
    "vendor/mjlab/src/mjlab/tasks/velocity/config/g1/env_cfgs.py",
    "vendor/mjlab/src/mjlab/tasks/velocity/config/g1/rl_cfg.py",
    "vendor/mjlab/src/mjlab/tasks/velocity/mdp/rewards.py",
    "vendor/mjlab/src/mjlab/tasks/velocity/mdp/velocity_command.py",
    "scripts/mjlab_train_g1_navigation_finetune.sh",
    "scripts/mjlab_train_g1_step_foundation.sh",
)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_identity(path: Path) -> dict[str, object]:
    def run(*arguments: str) -> str:
        completed = subprocess.run(
            ("git", "-C", str(path), *arguments),
            text=True,
            capture_output=True,
            check=True,
        )
        return completed.stdout.strip()

    return {
        "commit": run("rev-parse", "HEAD"),
        "dirty": bool(run("status", "--porcelain")),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--run-name", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-envs", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--source-checkpoint", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    return parser.parse_args()


def build_manifest(args: argparse.Namespace) -> dict[str, object]:
    root = args.repository_root.resolve()
    checkpoint = args.source_checkpoint.resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"Source checkpoint does not exist: {checkpoint}")
    if args.seed < 0 or args.num_envs < 1 or args.iterations < 1:
        raise ValueError("Seed must be nonnegative; environments and iterations must be positive")

    tracked_files = {}
    for relative in TRACKED_TRAINING_FILES:
        path = root / relative
        if not path.is_file():
            raise FileNotFoundError(f"Training input does not exist: {path}")
        tracked_files[relative] = file_sha256(path)

    command = list(args.command)
    if command[:1] == ["--"]:
        command = command[1:]
    return {
        "schema_version": 1,
        "manifest_id": str(uuid4()),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "stage": args.stage,
        "task": args.task,
        "run_name": args.run_name,
        "seed": args.seed,
        "num_envs": args.num_envs,
        "iterations": args.iterations,
        "source_checkpoint": {
            "path": str(checkpoint),
            "sha256": file_sha256(checkpoint),
        },
        "repositories": {
            "workspace": git_identity(root),
            "mjlab": git_identity(root / "vendor/mjlab"),
        },
        "training_files": tracked_files,
        "command": command,
    }


def atomic_write_json(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    atomic_write_json(args.output, build_manifest(args))
    print(f"Training manifest: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
