import numpy as np
import pytest

from scripts.qualify_dougie_stance_disturbance import approach_velocity


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
