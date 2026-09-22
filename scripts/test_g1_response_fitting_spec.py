from __future__ import annotations

import copy
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml


SCRIPT = Path(__file__).with_name("validate_g1_response_fitting_spec.py")
SPEC = importlib.util.spec_from_file_location("validate_g1_response_fitting_spec", SCRIPT)
assert SPEC and SPEC.loader
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)


def specifications():
    fit = yaml.safe_load(validator.DEFAULT_FIT_SPEC.read_text())
    commands = yaml.safe_load(validator.DEFAULT_COMMAND_SPEC.read_text())
    return fit, commands


def test_frozen_fitting_spec_has_sufficient_held_out_trials() -> None:
    result = validator.validate(*specifications())

    assert result["held_out_seed_count"] == 5
    assert result["planned_trials_by_family"] == {
        "forward_step_stop": 30,
        "yaw_step_stop": 30,
    }
    assert result["all_success_lower_bound_at_minimum"] > 0.90


def test_original_three_seed_plan_cannot_meet_declared_sample_size() -> None:
    fit, commands = specifications()
    fit = copy.deepcopy(fit)
    commands = copy.deepcopy(commands)
    fit["splits"]["held_out_seeds"] = [211, 223, 227]
    commands["splits"]["held_out"]["seeds"] = [211, 223, 227]
    commands["execution_order_by_seed"] = {
        seed: order
        for seed, order in commands["execution_order_by_seed"].items()
        if seed not in (229, 233)
    }

    with pytest.raises(ValueError, match="planned held-out trials"):
        validator.validate(fit, commands)


def test_rejects_split_overlap() -> None:
    fit, commands = specifications()
    fit["splits"]["held_out_seeds"][0] = 101

    with pytest.raises(ValueError, match="splits overlap"):
        validator.validate(fit, commands)
