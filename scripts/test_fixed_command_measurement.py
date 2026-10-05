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


def test_mixed_diagnostic_opt_in_checks_every_command_limit():
    measurement.validate_args(args(angular_z=.2,mixed_diagnostic=True,max_angular_z=.3))
    with pytest.raises(ValueError,match='policy limit'):
        measurement.validate_args(args(angular_z=.4,mixed_diagnostic=True,max_angular_z=.3))


def test_velocity_uses_unique_source_stamps_not_delivery_timing(monkeypatch):
    from collections import deque
    from types import SimpleNamespace
    node = SimpleNamespace(last_odom_source_time=None,last_odom_arrival=None,odom_count=0,
                           _odom_history=deque(),pose_samples=[],derived_velocity=(0.,0.,0.),
                           velocity=(0.,0.,0.),sample_writer=None)
    receipts = iter((10.,10.05,10.3,10.4))
    monkeypatch.setattr(measurement.time,'monotonic',lambda: next(receipts))
    msg = measurement.Odometry()
    msg.header.stamp.sec = 1
    msg.pose.pose.orientation.w = 1.
    measurement.FixedCommandMeasurement._odom_callback(node,msg)
    measurement.FixedCommandMeasurement._odom_callback(node,msg)
    assert node.odom_count == 1 and node.last_odom_arrival == 10.
    msg.header.stamp.nanosec = 100_000_000
    msg.pose.pose.position.x = .1
    measurement.FixedCommandMeasurement._odom_callback(node,msg)
    assert node.derived_velocity[0] == pytest.approx(1.)
    msg.header.stamp.nanosec = 50_000_000
    measurement.FixedCommandMeasurement._odom_callback(node,msg)
    assert node.odom_count == 2


def test_transition_interpolation_excludes_pre_stop_motion_from_stop_drift():
    samples = [(0.,0.,0.,0.),(.1,.1,0.,0.),(.2,.2,0.,0.)]
    start,end,path,_ = measurement.source_interval_metrics(samples,.15,.2)
    assert start[1] == pytest.approx(.15)
    assert end[1]-start[1] == pytest.approx(.05)
    assert path == pytest.approx(.05)
    with pytest.raises(RuntimeError,match='pose bracket'):
        measurement.source_interval_metrics(samples,-.1,.2)
