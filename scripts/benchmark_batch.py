#!/usr/bin/env python3
"""Run a versioned matrix of benchmark methods, scenarios, seeds, and repetitions."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC_SCHEMA_VERSION = 1
BATCH_SCHEMA_VERSION = 1
RESULT_SCHEMA_VERSION = 2
ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
RESERVED_OPTIONS = {"--seed", "--output"}


@dataclass(frozen=True)
class Trial:
    trial_id: str
    method_id: str
    scenario_id: str
    seed: int
    repetition: int
    timeout_s: float
    command: tuple[str, ...]
    relative_dir: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "trial_id": self.trial_id,
            "method_id": self.method_id,
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "repetition": self.repetition,
            "timeout_s": self.timeout_s,
            "command": list(self.command),
            "command_shell": shlex.join(self.command),
            "relative_dir": self.relative_dir,
        }


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def _git_output(*args: str) -> str:
    try:
        return subprocess.check_output(
            ("git", *args), cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_provenance_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def collect_provenance(spec: dict[str, Any]) -> dict[str, Any]:
    configured = spec.get("provenance", {})
    files: list[dict[str, Any]] = []
    for item in configured.get("files", []):
        item = {"id": item, "path": item} if isinstance(item, str) else item
        path = _resolve_provenance_path(item["path"])
        record = {
            "id": item.get("id", item["path"]),
            "path": str(path),
            "required": bool(item.get("required", True)),
            "exists": path.is_file(),
        }
        if path.is_file():
            record.update({"size_bytes": path.stat().st_size, "sha256": sha256_file(path)})
        files.append(record)

    repositories: list[dict[str, Any]] = []
    for item in configured.get("repositories", []):
        item = {"id": item, "path": item} if isinstance(item, str) else item
        path = _resolve_provenance_path(item["path"])
        record = {
            "id": item.get("id", item["path"]),
            "path": str(path),
            "exists": path.is_dir(),
            "commit": "",
            "dirty": None,
        }
        if path.is_dir():
            try:
                record["commit"] = subprocess.check_output(
                    ("git", "-C", str(path), "rev-parse", "HEAD"),
                    text=True,
                    stderr=subprocess.DEVNULL,
                ).strip()
                record["dirty"] = bool(
                    subprocess.check_output(
                        ("git", "-C", str(path), "status", "--porcelain"),
                        text=True,
                        stderr=subprocess.DEVNULL,
                    ).strip()
                )
            except (OSError, subprocess.CalledProcessError):
                pass
        repositories.append(record)

    containers: list[dict[str, Any]] = []
    for service in configured.get("container_services", []):
        record = {"service": service, "image_id": None}
        try:
            image_id = subprocess.check_output(
                ("docker", "compose", "images", "-q", service),
                cwd=REPO_ROOT,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            if image_id:
                record["image_id"] = subprocess.check_output(
                    ("docker", "image", "inspect", "--format={{.Id}}", image_id),
                    text=True,
                    stderr=subprocess.DEVNULL,
                ).strip()
        except (OSError, subprocess.CalledProcessError):
            pass
        containers.append(record)
    return {"files": files, "repositories": repositories, "containers": containers}


def load_spec(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError("experiment specification must be a mapping")
    validate_spec(data)
    return data


def _validate_id(value: Any, field: str) -> str:
    if not isinstance(value, str) or not ID_PATTERN.fullmatch(value):
        raise ValueError(f"{field} must match {ID_PATTERN.pattern}: {value!r}")
    return value


def _validate_tokens(tokens: Any, field: str) -> list[str]:
    if not isinstance(tokens, list) or not tokens or not all(
        isinstance(token, str) and token for token in tokens
    ):
        raise ValueError(f"{field} must be a nonempty list of string argv tokens")
    return tokens


def _reject_reserved_options(tokens: list[str], field: str) -> None:
    used = RESERVED_OPTIONS.intersection(tokens)
    if used:
        raise ValueError(f"{field} contains runner-owned options: {sorted(used)}")


def validate_spec(data: dict[str, Any]) -> None:
    if type(data.get("schema_version")) is not int or data["schema_version"] != SPEC_SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {SPEC_SCHEMA_VERSION}")
    experiment = data.get("experiment")
    if not isinstance(experiment, dict):
        raise ValueError("experiment must be a mapping")
    _validate_id(experiment.get("id"), "experiment.id")
    seeds = experiment.get("seeds")
    if not isinstance(seeds, list) or not seeds or any(type(seed) is not int for seed in seeds):
        raise ValueError("experiment.seeds must be a nonempty list of integers")
    if len(set(seeds)) != len(seeds):
        raise ValueError("experiment.seeds must not contain duplicates")
    repetitions = experiment.get("repetitions", 1)
    if type(repetitions) is not int or repetitions < 1:
        raise ValueError("experiment.repetitions must be a positive integer")
    timeout_s = experiment.get("timeout_s", 300.0)
    if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) or timeout_s <= 0:
        raise ValueError("experiment.timeout_s must be positive")

    methods = data.get("methods")
    scenarios = data.get("scenarios")
    if not isinstance(methods, list) or not methods:
        raise ValueError("methods must be a nonempty list")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("scenarios must be a nonempty list")
    method_ids: list[str] = []
    for index, method in enumerate(methods):
        if not isinstance(method, dict):
            raise ValueError(f"methods[{index}] must be a mapping")
        method_ids.append(_validate_id(method.get("id"), f"methods[{index}].id"))
        _validate_tokens(method.get("command"), f"methods[{index}].command")
        args = method.get("args", [])
        if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
            raise ValueError(f"methods[{index}].args must be a list of strings")
        _reject_reserved_options(args, f"methods[{index}].args")
    if len(set(method_ids)) != len(method_ids):
        raise ValueError("method IDs must be unique")

    scenario_ids: list[str] = []
    for index, scenario in enumerate(scenarios):
        if not isinstance(scenario, dict):
            raise ValueError(f"scenarios[{index}] must be a mapping")
        scenario_ids.append(_validate_id(scenario.get("id"), f"scenarios[{index}].id"))
        args = scenario.get("args", [])
        if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
            raise ValueError(f"scenarios[{index}].args must be a list of strings")
        _reject_reserved_options(args, f"scenarios[{index}].args")
    if len(set(scenario_ids)) != len(scenario_ids):
        raise ValueError("scenario IDs must be unique")

    aggregation = data.get("aggregation", {})
    if not isinstance(aggregation, dict):
        raise ValueError("aggregation must be a mapping")
    reference = aggregation.get("reference_method")
    if reference is not None and reference not in method_ids:
        raise ValueError("aggregation.reference_method must identify a configured method")

    provenance = data.get("provenance", {})
    if not isinstance(provenance, dict):
        raise ValueError("provenance must be a mapping")
    for field in ("files", "repositories"):
        items = provenance.get(field, [])
        if not isinstance(items, list):
            raise ValueError(f"provenance.{field} must be a list")
        for item in items:
            if isinstance(item, str):
                continue
            if not isinstance(item, dict) or not isinstance(item.get("path"), str):
                raise ValueError(f"provenance.{field} entries require a string path")
    services = provenance.get("container_services", [])
    if not isinstance(services, list) or not all(isinstance(item, str) for item in services):
        raise ValueError("provenance.container_services must be a list of strings")


def _format_tokens(tokens: list[str], variables: dict[str, Any], field: str) -> list[str]:
    formatted: list[str] = []
    for token in tokens:
        try:
            formatted.append(token.format_map(variables))
        except KeyError as error:
            raise ValueError(f"unknown template variable in {field}: {error.args[0]}") from error
    return formatted


def expand_trials(
    spec: dict[str, Any],
    run_dir: Path,
    method_filter: set[str] | None = None,
    scenario_filter: set[str] | None = None,
) -> list[Trial]:
    experiment = spec["experiment"]
    repetitions = experiment.get("repetitions", 1)
    default_timeout = float(experiment.get("timeout_s", 300.0))
    trials: list[Trial] = []
    for method in spec["methods"]:
        method_id = method["id"]
        if method_filter and method_id not in method_filter:
            continue
        for scenario in spec["scenarios"]:
            scenario_id = scenario["id"]
            if scenario_filter and scenario_id not in scenario_filter:
                continue
            for seed in experiment["seeds"]:
                for repetition in range(1, repetitions + 1):
                    trial_id = (
                        f"method={method_id}__scenario={scenario_id}__"
                        f"seed={seed}__rep={repetition:03d}"
                    )
                    relative_dir = (
                        f"trials/method={method_id}/scenario={scenario_id}/"
                        f"seed={seed}/repetition={repetition:03d}"
                    )
                    result_json = run_dir / relative_dir / "attempt=001" / "result.json"
                    variables = {
                        "repo": str(REPO_ROOT),
                        "run_dir": str(run_dir),
                        "trial_dir": str(result_json.parent.parent),
                        "attempt_dir": str(result_json.parent),
                        "result_json": str(result_json),
                        "method_id": method_id,
                        "scenario_id": scenario_id,
                        "seed": seed,
                        "repetition": repetition,
                    }
                    command = _format_tokens(method["command"], variables, "method.command")
                    command += _format_tokens(scenario.get("args", []), variables, "scenario.args")
                    command += _format_tokens(method.get("args", []), variables, "method.args")
                    command += ["--seed", str(seed), "--output", str(result_json)]
                    timeout_s = float(method.get("timeout_s", default_timeout))
                    trials.append(
                        Trial(
                            trial_id,
                            method_id,
                            scenario_id,
                            seed,
                            repetition,
                            timeout_s,
                            tuple(command),
                            relative_dir,
                        )
                    )
    if not trials:
        raise ValueError("filters selected no trials")
    return trials


def classify_result(result_path: Path, return_code: int, infrastructure_reasons: set[str]) -> dict[str, Any]:
    classification: dict[str, Any] = {
        "outcome": "artifact_missing",
        "result_path": str(result_path),
        "result_schema_version": None,
        "valid_trial": None,
        "invalid_reason": None,
        "success": None,
        "failure_reason": None,
    }
    if not result_path.is_file():
        return classification
    try:
        data = json.loads(result_path.read_text())
    except (OSError, json.JSONDecodeError):
        classification["outcome"] = "artifact_malformed"
        return classification
    schema_version = data.get("schema_version")
    classification.update(
        {
            "result_schema_version": schema_version,
            "valid_trial": data.get("valid_trial"),
            "invalid_reason": data.get("invalid_reason"),
            "success": data.get("success"),
            "failure_reason": data.get("failure_reason"),
        }
    )
    if type(schema_version) is not int or schema_version != RESULT_SCHEMA_VERSION:
        classification["outcome"] = "schema_unsupported"
    elif data.get("valid_trial") is not True:
        classification["outcome"] = (
            "benchmark_invalid_infrastructure"
            if data.get("invalid_reason") in infrastructure_reasons
            else "benchmark_invalid_algorithm"
        )
    elif data.get("success") is True:
        classification["outcome"] = "valid_success" if return_code == 0 else "validation_failed"
    else:
        classification["outcome"] = "valid_algorithm_failure"
    return classification


def run_process(command: tuple[str, ...], attempt_dir: Path, timeout_s: float) -> tuple[int, bool, float]:
    attempt_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    with (attempt_dir / "stdout.log").open("w", encoding="utf-8") as stdout, (
        attempt_dir / "stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            text=True,
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )
        try:
            return_code = process.wait(timeout=timeout_s)
            timed_out = False
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                return_code = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                return_code = process.wait()
    return return_code, timed_out, time.monotonic() - started


def create_manifest(
    spec_path: Path,
    spec: dict[str, Any],
    run_dir: Path,
    trials: list[Trial],
    dry_run: bool,
    provenance: dict[str, Any],
) -> dict[str, Any]:
    spec_bytes = spec_path.read_bytes()
    return {
        "schema_version": BATCH_SCHEMA_VERSION,
        "batch_id": run_dir.name,
        "started_at": utc_now(),
        "repository": str(REPO_ROOT),
        "git_commit": _git_output("rev-parse", "HEAD"),
        "git_dirty": bool(_git_output("status", "--porcelain")),
        "python": sys.version,
        "platform": platform.platform(),
        "spec_path": str(spec_path),
        "spec_sha256": hashlib.sha256(spec_bytes).hexdigest(),
        "provenance": provenance,
        "dry_run": dry_run,
        "expected_trials": len(trials),
        "resolved_spec": spec,
        "trials": [trial.as_dict() for trial in trials],
    }


def run_batch(
    spec_path: Path,
    output_root: Path | None = None,
    method_filter: set[str] | None = None,
    scenario_filter: set[str] | None = None,
    max_trials: int | None = None,
    dry_run: bool = False,
    run_dir: Path | None = None,
) -> tuple[Path, int]:
    spec_path = spec_path.expanduser().resolve()
    spec = load_spec(spec_path)
    if run_dir is None:
        configured_root = output_root or Path(spec["experiment"].get("output_root", "results/batches"))
        if not configured_root.is_absolute():
            configured_root = REPO_ROOT / configured_root
        digest = hashlib.sha256(spec_path.read_bytes()).hexdigest()[:10]
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        run_dir = configured_root.resolve() / f"{spec['experiment']['id']}_{stamp}_{digest}"
    else:
        run_dir = run_dir.expanduser().resolve()
    if spec["experiment"].get("require_repository_output", True) and not run_dir.is_relative_to(REPO_ROOT):
        raise ValueError(f"run directory must be inside repository: {run_dir}")
    if run_dir.exists():
        raise ValueError(f"run directory already exists: {run_dir}")

    trials = expand_trials(spec, run_dir, method_filter, scenario_filter)
    if max_trials is not None:
        if max_trials < 1:
            raise ValueError("max_trials must be positive")
        trials = trials[:max_trials]
    provenance = collect_provenance(spec)
    missing_required = [
        item["id"]
        for item in provenance["files"]
        if item["required"] and not item["exists"]
    ]
    if missing_required:
        raise ValueError(f"required provenance files are missing: {missing_required}")
    run_dir.mkdir(parents=True)
    (run_dir / "resolved_spec.yaml").write_text(yaml.safe_dump(spec, sort_keys=False))
    manifest = create_manifest(spec_path, spec, run_dir, trials, dry_run, provenance)
    write_json(run_dir / "manifest.json", manifest)

    infrastructure_reasons = set(
        spec.get("invalid_trial_policy", {}).get("infrastructure_reasons", [])
    )
    statuses: list[dict[str, Any]] = []
    interrupted = False
    try:
        for trial in trials:
            trial_dir = run_dir / trial.relative_dir
            attempt_dir = trial_dir / "attempt=001"
            write_json(trial_dir / "trial_manifest.json", trial.as_dict())
            print(f"[{trial.trial_id}] {shlex.join(trial.command)}", flush=True)
            if dry_run:
                status = {
                    "schema_version": BATCH_SCHEMA_VERSION,
                    "trial_id": trial.trial_id,
                    "execution_state": "planned",
                    "outcome": "not_run",
                }
            else:
                write_json(
                    trial_dir / "trial_status.json",
                    {
                        "schema_version": BATCH_SCHEMA_VERSION,
                        "trial_id": trial.trial_id,
                        "execution_state": "starting",
                        "started_at": utc_now(),
                    },
                )
                try:
                    return_code, timed_out, duration_s = run_process(
                        trial.command, attempt_dir, trial.timeout_s
                    )
                    result_path = attempt_dir / "result.json"
                    classification = classify_result(
                        result_path, return_code, infrastructure_reasons
                    )
                    status = {
                        "schema_version": BATCH_SCHEMA_VERSION,
                        "trial_id": trial.trial_id,
                        "execution_state": "timed_out" if timed_out else "exited",
                        "return_code": return_code,
                        "timed_out": timed_out,
                        "duration_s": round(duration_s, 3),
                        "finished_at": utc_now(),
                        **classification,
                    }
                except OSError as error:
                    status = {
                        "schema_version": BATCH_SCHEMA_VERSION,
                        "trial_id": trial.trial_id,
                        "execution_state": "spawn_error",
                        "outcome": "not_run",
                        "error": str(error),
                        "finished_at": utc_now(),
                    }
            write_json(trial_dir / "trial_status.json", status)
            statuses.append(status)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        completed = len(statuses)
        successful = sum(status.get("outcome") == "valid_success" for status in statuses)
        batch_status = {
            "schema_version": BATCH_SCHEMA_VERSION,
            "batch_id": run_dir.name,
            "status": (
                "interrupted"
                if interrupted
                else ("dry_run" if dry_run else ("passed" if successful == len(trials) else "completed_with_failures"))
            ),
            "expected_trials": len(trials),
            "completed_trials": completed,
            "successful_trials": successful,
            "finished_at": utc_now(),
        }
        write_json(run_dir / "status.json", batch_status)
    return run_dir, 0 if not interrupted and (dry_run or successful == len(trials)) else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", required=True, type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--run-dir", type=Path, help="exact new run directory, mainly for automation")
    parser.add_argument("--method", action="append", default=[])
    parser.add_argument("--scenario", action="append", default=[])
    parser.add_argument("--max-trials", type=int)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_dir, return_code = run_batch(
            args.spec,
            args.output_root,
            set(args.method) or None,
            set(args.scenario) or None,
            args.max_trials,
            args.dry_run,
            args.run_dir,
        )
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"Benchmark batch failed: {error}", file=sys.stderr)
        return 2
    print(f"Batch artifacts: {run_dir}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
