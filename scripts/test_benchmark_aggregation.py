from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


def load_script(name: str, filename: str | None = None):
    path = Path(__file__).with_name(filename or f"{name}.py")
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


benchmark_batch = load_script("benchmark_batch")
aggregate = load_script("aggregate_benchmark_batch")
batch_tests = load_script("test_benchmark_batch_helpers", "test_benchmark_batch.py")


def test_wilson_interval_boundaries() -> None:
    assert aggregate.wilson_interval(0, 0) == (None, None)
    zero_low, zero_high = aggregate.wilson_interval(0, 10)
    full_low, full_high = aggregate.wilson_interval(10, 10)
    assert zero_low == 0.0
    assert 0.0 < zero_high < 0.5
    assert 0.5 < full_low < 1.0
    assert full_high == 1.0


def test_aggregation_is_deterministic_and_paired_by_seed(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    batch_tests.write_fake_benchmark(fake)
    spec_path = batch_tests.make_spec(tmp_path, fake)
    batch_dir = tmp_path / "batch"
    _, return_code = benchmark_batch.run_batch(spec_path, run_dir=batch_dir)
    assert return_code == 0

    first = aggregate.aggregate_batch(batch_dir)
    aggregate_bytes = (batch_dir / "aggregate" / "aggregate.json").read_bytes()
    groups_bytes = (batch_dir / "aggregate" / "groups.csv").read_bytes()
    second = aggregate.aggregate_batch(batch_dir)

    assert first == second
    assert (batch_dir / "aggregate" / "aggregate.json").read_bytes() == aggregate_bytes
    assert (batch_dir / "aggregate" / "groups.csv").read_bytes() == groups_bytes
    comparison = first["paired_comparisons"][0]
    assert comparison["metric"] == "final_score"
    assert comparison["paired_seeds"] == 2
    assert comparison["excluded_seeds"] == []
    assert comparison["mean"] == 10.0
    assert comparison["mean_ci_low"] == 10.0
    assert comparison["mean_ci_high"] == 10.0


def test_aggregation_keeps_unstarted_trial_in_run_table(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    batch_tests.write_fake_benchmark(fake)
    spec_path = batch_tests.make_spec(tmp_path, fake)
    batch_dir = tmp_path / "dry"
    benchmark_batch.run_batch(spec_path, max_trials=2, dry_run=True, run_dir=batch_dir)

    result = aggregate.aggregate_batch(batch_dir)
    group = result["groups"][0]
    assert group["expected_trials"] == 2
    assert group["evaluable_trials"] == 0
    assert group["missing_or_malformed_trials"] == 2
    runs = (batch_dir / "aggregate" / "runs.csv").read_text().splitlines()
    assert len(runs) == 3
    assert all("not_run" in row for row in runs[1:])


def test_unsupported_result_schema_is_not_evaluable(tmp_path: Path) -> None:
    fake = tmp_path / "fake.py"
    batch_tests.write_fake_benchmark(fake)
    spec_path = batch_tests.make_spec(tmp_path, fake)
    batch_dir = tmp_path / "batch"
    benchmark_batch.run_batch(spec_path, max_trials=1, run_dir=batch_dir)
    manifest = json.loads((batch_dir / "manifest.json").read_text())
    trial_dir = batch_dir / manifest["trials"][0]["relative_dir"]
    result_path = trial_dir / "attempt=001" / "result.json"
    data = json.loads(result_path.read_text())
    data["schema_version"] = 1
    result_path.write_text(json.dumps(data))

    result = aggregate.aggregate_batch(batch_dir)
    group = result["groups"][0]
    assert group["evaluable_trials"] == 0
    assert group["missing_or_malformed_trials"] == 1
