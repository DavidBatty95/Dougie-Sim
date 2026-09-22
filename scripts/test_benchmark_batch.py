from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest
import yaml


SCRIPT = Path(__file__).with_name("benchmark_batch.py")
SPEC = importlib.util.spec_from_file_location("benchmark_batch", SCRIPT)
assert SPEC and SPEC.loader
benchmark_batch = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = benchmark_batch
SPEC.loader.exec_module(benchmark_batch)


def write_fake_benchmark(path: Path) -> None:
    path.write_text(
        """#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--method', required=True)
parser.add_argument('--scenario')
parser.add_argument('--seed', type=int, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)
base = 20.0 if args.method == 'candidate' else 10.0
data = {
    'schema_version': 2,
    'valid_trial': True,
    'invalid_reason': None,
    'success': True,
    'failure_reason': None,
    'goals_requested': 1,
    'goals_reached': 1,
    'collisions': 0,
    'fallen': False,
    'no_movement': False,
    'final_score': base + args.seed,
    'minimum_body_clearance_m': 0.5,
    'total_time_s': 5.0,
    'actual_path_length_m': 2.0,
}
args.output.write_text(json.dumps(data))
for suffix in ('_goals.csv', '_events.csv', '_scenario.yaml'):
    args.output.with_name(args.output.stem + suffix).write_text('data\\n')
"""
    )


def make_spec(tmp_path: Path, fake: Path) -> Path:
    data = {
        "schema_version": 1,
        "experiment": {
            "id": "test_batch",
            "seeds": [1, 2],
            "repetitions": 2,
            "timeout_s": 10,
            "require_repository_output": False,
        },
        "aggregation": {
            "reference_method": "reference",
            "bootstrap_samples": 100,
            "bootstrap_seed": 7,
            "continuous_metrics": ["final_score"],
            "binary_metrics": ["success", "collisions"],
        },
        "methods": [
            {
                "id": method,
                "command": [sys.executable, str(fake), "--method", "{method_id}"],
            }
            for method in ("reference", "candidate")
        ],
        "scenarios": [{"id": "crossing", "args": ["--scenario", "{scenario_id}"]}],
    }
    path = tmp_path / "spec.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def test_expands_stable_cartesian_matrix(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    write_fake_benchmark(fake)
    spec = benchmark_batch.load_spec(make_spec(tmp_path, fake))
    trials = benchmark_batch.expand_trials(spec, tmp_path / "run")

    assert len(trials) == 8
    assert trials[0].trial_id == "method=reference__scenario=crossing__seed=1__rep=001"
    assert trials[-1].trial_id == "method=candidate__scenario=crossing__seed=2__rep=002"
    assert isinstance(trials[0].command, tuple)
    assert "--seed" in trials[0].command
    assert "--output" in trials[0].command


def test_rejects_reserved_options_and_duplicate_ids(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    write_fake_benchmark(fake)
    path = make_spec(tmp_path, fake)
    data = yaml.safe_load(path.read_text())
    data["methods"][0]["args"] = ["--seed", "9"]
    with pytest.raises(ValueError, match="runner-owned"):
        benchmark_batch.validate_spec(data)

    data = yaml.safe_load(path.read_text())
    data["methods"][1]["id"] = "reference"
    with pytest.raises(ValueError, match="unique"):
        benchmark_batch.validate_spec(data)


def test_runs_full_matrix_and_preserves_trial_artifacts(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    write_fake_benchmark(fake)
    run_dir = tmp_path / "batch"
    actual_dir, return_code = benchmark_batch.run_batch(
        make_spec(tmp_path, fake), run_dir=run_dir
    )

    assert actual_dir == run_dir
    assert return_code == 0
    manifest = json.loads((run_dir / "manifest.json").read_text())
    status = json.loads((run_dir / "status.json").read_text())
    assert manifest["expected_trials"] == 8
    assert status["successful_trials"] == 8
    for trial in manifest["trials"]:
        trial_dir = run_dir / trial["relative_dir"]
        trial_status = json.loads((trial_dir / "trial_status.json").read_text())
        assert trial_status["outcome"] == "valid_success"
        assert (trial_dir / "attempt=001" / "result.json").is_file()
        assert (trial_dir / "attempt=001" / "stdout.log").is_file()


def test_dry_run_records_every_planned_trial(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    write_fake_benchmark(fake)
    run_dir = tmp_path / "dry"
    _, return_code = benchmark_batch.run_batch(
        make_spec(tmp_path, fake), max_trials=3, dry_run=True, run_dir=run_dir
    )
    assert return_code == 0
    assert json.loads((run_dir / "manifest.json").read_text())["expected_trials"] == 3
    statuses = list(run_dir.glob("trials/**/trial_status.json"))
    assert len(statuses) == 3
    assert all(json.loads(path.read_text())["outcome"] == "not_run" for path in statuses)


def test_classifies_missing_unsupported_and_infrastructure_results(tmp_path: Path) -> None:
    missing = benchmark_batch.classify_result(tmp_path / "missing.json", 1, set())
    assert missing["outcome"] == "artifact_missing"

    unsupported_path = tmp_path / "unsupported.json"
    unsupported_path.write_text(json.dumps({"schema_version": 1}))
    unsupported = benchmark_batch.classify_result(unsupported_path, 0, set())
    assert unsupported["outcome"] == "schema_unsupported"

    invalid_path = tmp_path / "invalid.json"
    invalid_path.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "valid_trial": False,
                "invalid_reason": "nav2_not_ready",
            }
        )
    )
    invalid = benchmark_batch.classify_result(
        invalid_path, 1, {"nav2_not_ready"}
    )
    assert invalid["outcome"] == "benchmark_invalid_infrastructure"


def test_collects_file_and_repository_provenance(tmp_path: Path) -> None:
    artifact = tmp_path / "artifact.bin"
    artifact.write_bytes(b"research artifact")
    provenance = benchmark_batch.collect_provenance(
        {
            "provenance": {
                "files": [{"id": "artifact", "path": str(artifact)}],
                "repositories": [{"id": "missing", "path": str(tmp_path / "missing")}],
            }
        }
    )
    assert provenance["files"][0]["exists"] is True
    assert provenance["files"][0]["size_bytes"] == len(b"research artifact")
    assert len(provenance["files"][0]["sha256"]) == 64
    assert provenance["repositories"][0]["exists"] is False
