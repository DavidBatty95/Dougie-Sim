#!/usr/bin/env python3
"""Combine validated single-trial G1 response files into one frozen dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = REPO_ROOT / "docs" / "schemas" / "g1_response_calibration.schema.json"


def validate_dataset(dataset: dict[str, Any], schema: dict[str, Any]) -> None:
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    validator = validator_class(schema, format_checker=jsonschema.FormatChecker())
    errors = sorted(validator.iter_errors(dataset), key=lambda error: list(error.path))
    if errors:
        details = "; ".join(f"{'.'.join(map(str, error.path))}: {error.message}" for error in errors)
        raise ValueError(details)
    for trial in dataset["trials"]:
        metrics = trial["metrics"]
        available = set(metrics["available_post_stop_drift_horizons_s"])
        for horizon in (1, 3, 5):
            present = metrics[f"post_stop_drift_{horizon}s_m"] is not None
            yaw_present = metrics[f"post_stop_yaw_drift_{horizon}s_rad"] is not None
            if (horizon in available) != present or present != yaw_present:
                raise ValueError(
                    f"trial {trial['trial_id']} has inconsistent {horizon}s drift availability"
                )


def assemble(metadata: dict[str, Any], inputs: list[dict[str, Any]]) -> dict[str, Any]:
    if not inputs:
        raise ValueError("at least one response input is required")
    expected = {
        "schema_version": 1,
        "dataset_id": metadata["dataset_id"],
        "platform": metadata["platform"],
        "protocol_version": "g1_response_v1",
        "provenance": metadata["provenance"],
    }
    trials = []
    trial_ids: set[str] = set()
    for index, dataset in enumerate(inputs):
        for key, value in expected.items():
            if dataset.get(key) != value:
                raise ValueError(f"input {index} has mismatched {key}")
        for trial in dataset["trials"]:
            trial_id = trial["trial_id"]
            if trial_id in trial_ids:
                raise ValueError(f"duplicate trial ID: {trial_id}")
            trial_ids.add(trial_id)
            trials.append(trial)
    return {
        **expected,
        "collected_at": metadata["collected_at"],
        "description": metadata.get("description", ""),
        "trials": sorted(trials, key=lambda trial: trial["trial_id"]),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        metadata = json.loads(args.metadata.read_text())
        schema = json.loads(args.schema.read_text())
        inputs = [json.loads(path.read_text()) for path in args.inputs]
        for dataset in inputs:
            validate_dataset(dataset, schema)
        output = assemble(metadata, inputs)
        validate_dataset(output, schema)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2, sort_keys=True, allow_nan=False) + "\n")
    except (OSError, KeyError, ValueError, json.JSONDecodeError, jsonschema.SchemaError) as error:
        print(f"Response dataset assembly failed: {error}")
        return 2
    print(f"Response dataset: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
