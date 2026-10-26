import numpy as np
import pytest

from scripts.qualify_dougie_stance_disturbance import approach_velocity
from scripts.qualify_dougie_stance_disturbance import settling_yaw_command
from types import SimpleNamespace


@pytest.mark.parametrize('duration', [1., 2., 3.])
def test_planned_stop_taper_is_bounded_and_ends_before_terminal_request(duration):
    times = np.arange(13.-duration, 13.+.05, .005)
    commands = np.array([approach_velocity(t, 13., duration) for t in times])
    assert np.all((commands >= 0.) & (commands <= .15))
    assert np.all(np.diff(commands) <= 1e-12)
    assert np.max(np.abs(np.diff(commands))) <= .15/duration*.005+1e-12
    assert approach_velocity(5., 13., duration) == .15
    assert approach_velocity(13., 13., duration) == 0.
    assert approach_velocity(15., 13., duration) == 0.
    assert approach_velocity(4., 13., duration) == 0.


def test_zero_braking_retains_step_stop_comparator():
    assert approach_velocity(12.995, 13., 0.) == .15
    assert approach_velocity(13., 13., 0.) == 0.


def test_settling_yaw_damps_measured_rotation_with_bounded_slew():
    measured = SimpleNamespace(stamp_s=12., gyro=np.array([0., 0., .15]), foot_force=np.full(4, 30.))
    command = 0.
    for _ in range(100):
        previous = command
        command = settling_yaw_command(measured, 12., 13., 2., 2., command, .005)
        assert abs(command) <= .2 and abs(command-previous) <= .0025+1e-12
    assert command == pytest.approx(-.2)
    assert settling_yaw_command(measured, 13., 13., 2., 2., command, .005) == 0.
    assert settling_yaw_command(measured, 10., 13., 2., 2., command, .005) == 0.


@pytest.mark.parametrize('fault', ['stale', 'support', 'gyro', 'nan'])
def test_settling_yaw_withholds_motion_on_invalid_measurements(fault):
    measured = SimpleNamespace(stamp_s=12., gyro=np.array([0., 0., .15]), foot_force=np.full(4, 30.))
    if fault == 'stale':
        measured.stamp_s = 11.
    elif fault == 'support':
        measured.foot_force[0] = 0.
    elif fault == 'gyro':
        measured.gyro[0] = 1.
    else:
        measured.gyro[0] = np.nan
    assert settling_yaw_command(measured, 12., 13., 2., 1., -.1, .005) == 0.


@pytest.mark.parametrize('up,enabled', [([-.15, 0., .99], True), ([.15, 0., .99], False),
                                      ([0., 0., 1.], False), ([np.nan, 0., 1.], False)])
def test_settling_posture_gate_uses_estimated_gravity(up, enabled):
    measured = SimpleNamespace(stamp_s=12., gyro=np.array([0., 0., .15]), foot_force=np.full(4, 30.))
    command = settling_yaw_command(measured, 12., 13., 2., 1., 0., .005, 'downhill', up)
    assert (command < 0.) == enabled
