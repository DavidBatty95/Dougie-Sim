from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("measure_g1_fixed_command.py")
SPEC = importlib.util.spec_from_file_location("measure_g1_fixed_command", SCRIPT)
assert SPEC and SPEC.loader
measurement = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = measurement
SPEC.loader.exec_module(measurement)


def args(**overrides):
    values = {
        "linear_x": 0.2,
        "linear_y": 0.0,
        "angular_z": 0.0,
        "duration": 8.0,
        "settle": 3.0,
        "stop_duration": 5.0,
        "timeout": 20.0,
        "odom_timeout": 0.5,
        "diagnostics_timeout": 1.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_sustained_crossing_rejects_spike_and_accepts_held_response() -> None:
    spike = [(0.0, 0.0), (0.1, 0.04), (0.2, 0.0), (0.3, 0.0)]
    assert measurement.first_sustained_crossing(spike, 0.03, 0.2) is None

    held = [(0.0, 0.0), (0.1, 0.04), (0.2, 0.05), (0.3, 0.06), (0.4, 0.06)]
    assert measurement.first_sustained_crossing(held, 0.03, 0.2) == pytest.approx(0.1)


def test_settling_requires_complete_below_threshold_window() -> None:
    samples = [(0.0, 0.05), (0.1, 0.02), (0.2, 0.02), (0.3, 0.04), (0.4, 0.01)]
    assert measurement.settling_time(samples, 0.03, 0.2) is None
    samples.extend([(0.5, 0.01), (0.6, 0.01)])
    assert measurement.settling_time(samples, 0.03, 0.2) == pytest.approx(0.4)


def test_argument_validation_rejects_zero_nonfinite_and_invalid_durations() -> None:
    measurement.validate_args(args())
    with pytest.raises(ValueError, match="exactly one"):
        measurement.validate_args(args(linear_x=0.0))
    with pytest.raises(ValueError, match="exactly one"):
        measurement.validate_args(args(angular_z=0.1))
    with pytest.raises(ValueError, match="policy limit"):
        measurement.validate_args(args(max_linear_x=0.1))
    with pytest.raises(ValueError, match="finite"):
        measurement.validate_args(args(linear_x=math.nan))
    with pytest.raises(ValueError, match="duration"):
        measurement.validate_args(args(duration=0.0))
