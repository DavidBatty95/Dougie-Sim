#!/usr/bin/env python3
"""Aggregate one benchmark batch without requiring ROS, Docker, or MuJoCo."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


BATCH_SCHEMA_VERSION = 1
RESULT_SCHEMA_VERSION = 2
Z_95 = 1.959963984540054
DEFAULT_CONTINUOUS_METRICS = (
    "final_score",
    "minimum_body_clearance_m",
    "total_time_s",
    "actual_path_length_m",
)
DEFAULT_BINARY_METRICS = ("success", "collisions", "fallen", "no_movement")
UNEVALUABLE_OUTCOMES = {
    "not_run",
    "artifact_missing",
    "artifact_malformed",
    "schema_unsupported",
    "benchmark_invalid_infrastructure",
}


def read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"JSON object required: {path}")
    return data


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def bootstrap_mean_interval(
    values: list[float], samples: int, seed: int, confidence: float
) -> tuple[float | None, float | None]:
    if not values:
        return None, None
    if len(values) == 1:
        return values[0], values[0]
    rng = random.Random(seed)
    means = [
        statistics.fmean(rng.choice(values) for _ in values)
        for _ in range(samples)
    ]
    alpha = (1.0 - confidence) / 2.0
    return percentile(means, alpha), percentile(means, 1.0 - alpha)


def wilson_interval(successes: int, total: int, z: float = Z_95) -> tuple[float | None, float | None]:
    if total == 0:
        return None, None
    proportion = successes / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(
        proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total)
    ) / denominator
    low = 0.0 if successes == 0 else max(0.0, center - margin)
    high = 1.0 if successes == total else min(1.0, center + margin)
    return low, high


def numeric_value(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def binary_value(data: dict[str, Any], metric: str) -> bool | None:
    value = data.get(metric)
    if metric == "collisions":
        number = numeric_value(value)
        return None if number is None else number > 0
    return value if isinstance(value, bool) else None


def continuous_summary(
    values: list[float], bootstrap_samples: int, bootstrap_seed: int, confidence: float
) -> dict[str, Any]:
    low, high = bootstrap_mean_interval(values, bootstrap_samples, bootstrap_seed, confidence)
    return {
        "n": len(values),
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "stddev": statistics.stdev(values) if len(values) > 1 else (0.0 if values else None),
        "minimum": min(values) if values else None,
        "q1": percentile(values, 0.25),
        "q3": percentile(values, 0.75),
        "maximum": max(values) if values else None,
        "mean_ci_low": low,
        "mean_ci_high": high,
    }


def load_runs(batch_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest = read_json(batch_dir / "manifest.json")
    if type(manifest.get("schema_version")) is not int or manifest["schema_version"] != BATCH_SCHEMA_VERSION:
        raise ValueError(f"unsupported batch schema in {batch_dir / 'manifest.json'}")
    trials = manifest.get("trials")
    if not isinstance(trials, list):
        raise ValueError("batch manifest trials must be a list")
    runs: list[dict[str, Any]] = []
    for trial in trials:
        trial_dir = batch_dir / trial["relative_dir"]
        status_path = trial_dir / "trial_status.json"
        result_path = trial_dir / "attempt=001" / "result.json"
        status: dict[str, Any]
        if status_path.is_file():
            try:
                status = read_json(status_path)
            except (OSError, json.JSONDecodeError, ValueError):
                status = {"execution_state": "unknown", "outcome": "artifact_malformed"}
        else:
            status = {"execution_state": "unstarted", "outcome": "not_run"}
        result: dict[str, Any] = {}
        result_schema_supported = False
        if result_path.is_file():
            try:
                result = read_json(result_path)
                result_schema_supported = all(
                    (
                        type(result.get("schema_version")) is int,
                        result.get("schema_version") == RESULT_SCHEMA_VERSION,
                    )
                )
            except (OSError, json.JSONDecodeError, ValueError):
                result = {}
        outcome = status.get("outcome", "not_run")
        if result and not result_schema_supported:
            outcome = "schema_unsupported"
        run = {
            "trial_id": trial["trial_id"],
            "method_id": trial["method_id"],
            "scenario_id": trial["scenario_id"],
            "seed": trial["seed"],
            "repetition": trial["repetition"],
            "execution_state": status.get("execution_state", "unknown"),
            "outcome": outcome,
            "return_code": status.get("return_code"),
            "timed_out": bool(status.get("timed_out", False)),
            "duration_s": status.get("duration_s"),
            "result_path": str(result_path),
            "result_schema_version": result.get("schema_version"),
            "valid_trial": result.get("valid_trial"),
            "invalid_reason": result.get("invalid_reason"),
            "success": result.get("success"),
            "failure_reason": result.get("failure_reason"),
            "goals_requested": result.get("goals_requested"),
            "goals_reached": result.get("goals_reached"),
            "collisions": result.get("collisions"),
            "fallen": result.get("fallen"),
            "no_movement": result.get("no_movement"),
            "result": result if result_schema_supported else {},
        }
        runs.append(run)
    runs.sort(key=lambda run: (run["method_id"], run["scenario_id"], run["seed"], run["repetition"]))
    return manifest, runs


def aggregate_groups(
    runs: list[dict[str, Any]],
    continuous_metrics: Iterable[str],
    binary_metrics: Iterable[str],
    bootstrap_samples: int,
    bootstrap_seed: int,
    confidence: float,
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        grouped[(run["method_id"], run["scenario_id"])].append(run)
    summaries: list[dict[str, Any]] = []
    for group_index, ((method_id, scenario_id), group_runs) in enumerate(sorted(grouped.items())):
        evaluable = [run for run in group_runs if run["outcome"] not in UNEVALUABLE_OUTCOMES]
        summary: dict[str, Any] = {
            "method_id": method_id,
            "scenario_id": scenario_id,
            "expected_trials": len(group_runs),
            "evaluable_trials": len(evaluable),
            "infrastructure_invalid_trials": sum(
                run["outcome"] == "benchmark_invalid_infrastructure" for run in group_runs
            ),
            "algorithm_failure_trials": sum(
                run["outcome"] in {"valid_algorithm_failure", "benchmark_invalid_algorithm"}
                for run in group_runs
            ),
            "missing_or_malformed_trials": sum(
                run["outcome"] in {"artifact_missing", "artifact_malformed", "schema_unsupported", "not_run"}
                for run in group_runs
            ),
        }
        for metric in binary_metrics:
            observed = [
                value
                for run in evaluable
                if (value := binary_value(run["result"], metric)) is not None
            ]
            positives = sum(observed)
            low, high = wilson_interval(positives, len(observed))
            summary[f"{metric}_positive"] = positives
            summary[f"{metric}_n"] = len(observed)
            summary[f"{metric}_rate"] = positives / len(observed) if observed else None
            summary[f"{metric}_ci_low"] = low
            summary[f"{metric}_ci_high"] = high
            if metric == "success":
                summary["success_all_expected_rate"] = positives / len(group_runs)
        for metric_index, metric in enumerate(continuous_metrics):
            values = [
                value
                for run in evaluable
                if all(
                    (
                        run["valid_trial"] is True,
                        (value := numeric_value(run["result"].get(metric))) is not None,
                    )
                )
            ]
            metric_summary = continuous_summary(
                values,
                bootstrap_samples,
                bootstrap_seed + group_index * 1000 + metric_index,
                confidence,
            )
            for key, value in metric_summary.items():
                summary[f"{metric}_{key}"] = value
        summaries.append(summary)
    return summaries


def paired_comparisons(
    runs: list[dict[str, Any]],
    reference_method: str | None,
    continuous_metrics: Iterable[str],
    bootstrap_samples: int,
    bootstrap_seed: int,
    confidence: float,
) -> list[dict[str, Any]]:
    if not reference_method:
        return []
    cell_values: dict[tuple[str, str, int, str], list[float]] = defaultdict(list)
    methods = sorted({run["method_id"] for run in runs if run["method_id"] != reference_method})
    scenarios = sorted({run["scenario_id"] for run in runs})
    for run in runs:
        if run["outcome"] in UNEVALUABLE_OUTCOMES or run["valid_trial"] is not True:
            continue
        for metric in continuous_metrics:
            value = numeric_value(run["result"].get(metric))
            if value is not None:
                cell_values[(run["method_id"], run["scenario_id"], run["seed"], metric)].append(value)
    comparisons: list[dict[str, Any]] = []
    comparison_index = 0
    all_seeds = sorted({run["seed"] for run in runs})
    for method in methods:
        for scenario in scenarios:
            for metric in continuous_metrics:
                differences: list[float] = []
                excluded_seeds: list[int] = []
                for seed in all_seeds:
                    reference = cell_values.get((reference_method, scenario, seed, metric), [])
                    candidate = cell_values.get((method, scenario, seed, metric), [])
                    if not reference or not candidate:
                        excluded_seeds.append(seed)
                        continue
                    differences.append(statistics.fmean(candidate) - statistics.fmean(reference))
                summary = continuous_summary(
                    differences,
                    bootstrap_samples,
                    bootstrap_seed + 100000 + comparison_index,
                    confidence,
                )
                comparisons.append(
                    {
                        "reference_method": reference_method,
                        "method_id": method,
                        "scenario_id": scenario,
                        "metric": metric,
                        "difference_direction": "method_minus_reference",
                        "paired_seeds": len(differences),
                        "excluded_seeds": excluded_seeds,
                        **summary,
                    }
                )
                comparison_index += 1
    return comparisons


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = sorted({key for row in rows for key in row})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialized = {
                key: json.dumps(value, sort_keys=True) if isinstance(value, (list, dict)) else value
                for key, value in row.items()
                if key in fieldnames
            }
            writer.writerow(serialized)


def aggregate_batch(batch_dir: Path, output_dir: Path | None = None) -> dict[str, Any]:
    batch_dir = batch_dir.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve() if output_dir else batch_dir / "aggregate"
    manifest, runs = load_runs(batch_dir)
    aggregation = manifest.get("resolved_spec", {}).get("aggregation", {})
    continuous_metrics = tuple(aggregation.get("continuous_metrics", DEFAULT_CONTINUOUS_METRICS))
    binary_metrics = tuple(aggregation.get("binary_metrics", DEFAULT_BINARY_METRICS))
    confidence = float(aggregation.get("confidence_level", 0.95))
    if not 0.0 < confidence < 1.0:
        raise ValueError("aggregation confidence_level must be between zero and one")
    bootstrap_samples = int(aggregation.get("bootstrap_samples", 2000))
    bootstrap_seed = int(aggregation.get("bootstrap_seed", 91357))
    if bootstrap_samples < 1:
        raise ValueError("aggregation bootstrap_samples must be positive")
    groups = aggregate_groups(
        runs,
        continuous_metrics,
        binary_metrics,
        bootstrap_samples,
        bootstrap_seed,
        confidence,
    )
    comparisons = paired_comparisons(
        runs,
        aggregation.get("reference_method"),
        continuous_metrics,
        bootstrap_samples,
        bootstrap_seed,
        confidence,
    )
    result = {
        "schema_version": BATCH_SCHEMA_VERSION,
        "batch_id": manifest["batch_id"],
        "expected_trials": len(runs),
        "confidence_level": confidence,
        "binary_interval": "two-sided Wilson score interval",
        "continuous_interval": "deterministic percentile bootstrap of the mean",
        "bootstrap_samples": bootstrap_samples,
        "bootstrap_seed": bootstrap_seed,
        "continuous_metrics": list(continuous_metrics),
        "binary_metrics": list(binary_metrics),
        "groups": groups,
        "paired_comparisons": comparisons,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "aggregate.json", result)
    run_rows = [{key: value for key, value in run.items() if key != "result"} for run in runs]
    write_csv(output_dir / "runs.csv", run_rows)
    write_csv(output_dir / "groups.csv", groups)
    write_csv(output_dir / "paired_differences.csv", comparisons)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = aggregate_batch(args.batch_dir, args.output_dir)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Benchmark aggregation failed: {error}", file=sys.stderr)
        return 2
    print(
        f"Aggregated {result['expected_trials']} trials: "
        f"{args.output_dir or args.batch_dir / 'aggregate'}"
    )
    if args.strict:
        incomplete = any(
            group["evaluable_trials"] != group["expected_trials"] for group in result["groups"]
        )
        return 1 if incomplete else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
