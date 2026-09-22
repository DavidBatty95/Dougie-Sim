from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("process_g1_response_trial.py")
SPEC = importlib.util.spec_from_file_location("process_g1_response_trial", SCRIPT)
assert SPEC and SPEC.loader
processor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = processor
SPEC.loader.exec_module(processor)


def test_collected_at_is_derived_from_trial_id() -> None:
    assert processor.collected_at("fixed_command_scene_20260827T135602Z") == (
        "2026-08-27T13:56:02Z"
    )


def test_collected_at_rejects_unstamped_trial_id() -> None:
    with pytest.raises(ValueError, match="UTC basic timestamp"):
        processor.collected_at("fixed_command_scene")


def test_resolve_samples_maps_container_workspace_path() -> None:
    resolved = processor.resolve_samples(
        {"raw_samples": "/workspace/results/trial.csv"}, None
    )

    assert resolved == processor.REPO_ROOT / "results/trial.csv"
