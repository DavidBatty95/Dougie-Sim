from types import SimpleNamespace

import numpy as np

from scripts.qualify_dougie_slope_stop import hold_eligibility


def observation(stamp, force=20., speed=0.):
    return SimpleNamespace(stamp_s=stamp, foot_force=np.full(4, force),
                           gyro=np.zeros(3), joint_velocity=np.full(12, speed))


def test_hold_requires_settling_and_uninterrupted_quiet_support():
    assert hold_eligibility(observation(10.4), 10.4, 10., None) == (False, None)
    ready, since = hold_eligibility(observation(10.5), 10.5, 10., None)
    assert not ready and since == 10.5
    assert hold_eligibility(observation(10.8), 10.8, 10., since)[0]
    assert hold_eligibility(observation(10.7, force=0.), 10.7, 10., since) == (False, None)
    assert hold_eligibility(observation(10.7, speed=1.), 10.7, 10., since) == (False, None)


def test_hold_refuses_stale_future_and_late_measurements():
    assert hold_eligibility(observation(10.5), 10.8, 10., 10.5) == (False, None)
    assert hold_eligibility(observation(10.9), 10.8, 10., 10.5) == (False, None)
    assert not hold_eligibility(observation(13.1), 13.1, 10., 10.5)[0]
