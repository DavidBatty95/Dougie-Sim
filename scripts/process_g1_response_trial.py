#!/usr/bin/env python3
"""Create provenance metadata and a validated response result for one trial."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from extract_g1_response_metrics import (
    DEFAULT_SCHEMA,
    assemble_dataset,
    sha256_file,
    validate_dataset,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def git_output(*args: str) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def resolve_samples(result: dict, explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.resolve()
    raw_path = result.get("raw_samples")
    if not isinstance(raw_path, str) or not raw_path:
        raise ValueError("measurement result does not identify raw samples")
    if raw_path.startswith("/workspace/"):
        return REPO_ROOT / raw_path.removeprefix("/workspace/")
    path = Path(raw_path)
    return path if path.is_absolute() else REPO_ROOT / path


def collected_at(trial_id: str) -> str:
    match = re.search(r"(\d{8}T\d{6}Z)$", trial_id)
    if match is None:
        raise ValueError("trial ID must end with a UTC basic timestamp")
    parsed = datetime.strptime(match.group(1), "%Y%m%dT%H%M%SZ").replace(
        tzinfo=timezone.utc
    )
    return parsed.isoformat().replace("+00:00", "Z")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurement_result", type=Path)
    parser.add_argument("--samples", type=Path)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--split", required=True, choices=("calibration", "held_out"))
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--repetition", required=True, type=int)
    parser.add_argument(
        "--command-family",
        required=True,
        choices=("forward_step_stop", "yaw_step_stop"),
    )
    parser.add_argument("--configuration", required=True, type=Path)
    parser.add_argument("--robot-model", required=True, type=Path)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--metadata-output", required=True, type=Path)
    parser.add_argument("--response-output", required=True, type=Path)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--description", default="Development response trial")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = json.loads(args.measurement_result.read_text())
        trial_id = result["trial_id"]
        samples = resolve_samples(result, args.samples)
        actual_samples_hash = sha256_file(samples)
        if actual_samples_hash != result.get("raw_samples_sha256"):
            raise ValueError("raw sample hash does not match the measurement result")
        policy_hash = (result.get("provenance") or {}).get("policy_sha256")
        if not isinstance(policy_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", policy_hash):
            raise ValueError("measurement result has no valid policy SHA-256")
        metadata = {
            "dataset_id": args.dataset_id,
            "platform": "mujoco",
            "collected_at": collected_at(trial_id),
            "description": args.description,
            "provenance": {
                "git_commit": git_output("rev-parse", "HEAD"),
                "git_dirty": bool(git_output("status", "--porcelain")),
                "simulator_commit": git_output(
                    "-C", "ros2_ws/src/unitree_mujoco", "rev-parse", "HEAD"
                ),
                "container_image_id": None,
                "policy_sha256": policy_hash,
                "robot_model_sha256": sha256_file(args.robot_model.resolve()),
                "scene_sha256": sha256_file(args.scene.resolve()),
                "configuration_sha256": sha256_file(args.configuration.resolve()),
            },
            "trial": {
                "trial_id": trial_id,
                "split": args.split,
                "seed": args.seed,
                "repetition": args.repetition,
                "command_family": args.command_family,
            },
        }
        dataset = assemble_dataset(samples, metadata, 0.20)
        validate_dataset(dataset, args.schema)
        args.metadata_output.parent.mkdir(parents=True, exist_ok=True)
        args.response_output.parent.mkdir(parents=True, exist_ok=True)
        args.metadata_output.write_text(
            json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
        args.response_output.write_text(
            json.dumps(dataset, indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
    except (KeyError, OSError, ValueError, json.JSONDecodeError, subprocess.SubprocessError) as error:
        print(f"Response trial processing failed: {error}")
        return 2
    print(f"Response metadata: {args.metadata_output}")
    print(f"Response result: {args.response_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
