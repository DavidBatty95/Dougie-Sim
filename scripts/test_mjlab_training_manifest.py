from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path


def load_module():
    path = Path(__file__).with_name("write_mjlab_training_manifest.py")
    spec = importlib.util.spec_from_file_location("write_mjlab_training_manifest", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_manifest_records_seed_parent_hash_and_training_inputs(tmp_path: Path) -> None:
    module = load_module()
    repository_root = Path(__file__).resolve().parents[1]
    checkpoint = tmp_path / "model_100.pt"
    checkpoint.write_bytes(b"checkpoint")
    args = argparse.Namespace(
        output=tmp_path / "manifest.json",
        repository_root=repository_root,
        stage="g1_natural_gait_finetune",
        task="test-task",
        run_name="test-run-seed-7",
        seed=7,
        num_envs=128,
        iterations=200,
        source_checkpoint=checkpoint,
        command=["--", "train", "test-task", "--agent.seed", "7"],
    )

    manifest = module.build_manifest(args)
    module.atomic_write_json(args.output, manifest)
    written = json.loads(args.output.read_text())

    assert written["schema_version"] == 1
    assert written["seed"] == 7
    assert written["source_checkpoint"]["sha256"] == module.file_sha256(checkpoint)
    assert written["command"] == ["train", "test-task", "--agent.seed", "7"]
    assert written["repositories"]["workspace"]["commit"]
    assert written["repositories"]["mjlab"]["commit"]
    assert set(written["training_files"]) == set(module.TRACKED_TRAINING_FILES)
