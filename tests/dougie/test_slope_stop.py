from types import SimpleNamespace

import numpy as np

from scripts.qualify_dougie_slope_stop import HoldAnchor, hold_eligibility, position_hold_command


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


def test_position_hold_is_frame_invariant_and_bounded():
    anchor, position, velocity = np.array([1., 2., 0.]), np.array([0., 0., 0.]), np.array([.1, .2, 0.])
    previous = np.zeros(3)
    for _ in range(100):
        command = position_hold_command(anchor, position, velocity, .3, previous, .02)
        assert np.linalg.norm(command[:2]) <= .08+1e-12
        assert np.linalg.norm(command[:2]-previous[:2]) <= .004+1e-12
        previous = command
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    offset = np.array([5., -7., 1.])
    transformed = position_hold_command(rotation@anchor+offset, rotation@position+offset,
                                        rotation@velocity, .3+np.pi/2, previous, .02)
    np.testing.assert_allclose(transformed, command, atol=1e-12)
    assert command[2] == 0.


def test_position_hold_rejects_invalid_inputs():
    import pytest
    for dt in (0., .1, float('nan')):
        with pytest.raises(ValueError):
            position_hold_command(np.zeros(3), np.zeros(3), np.zeros(3), 0., np.zeros(3), dt)


def test_hold_anchor_uses_capture_time_and_applies_each_revision_once():
    from dougie_sim.map_corrections import MapCorrection
    anchor = HoldAnchor([1., 2., 0.], 5., 1)
    events = [MapCorrection(1, 0., 10., (0., 0., 0.), (100., 0., 0.)),
              MapCorrection(2, 0., 10., (0., 0., 0.), (1., 0., 0.)),
              MapCorrection(3, 0., 10., (0., 0., 0.), (0., 2., 0.))]
    anchor.correct(events)
    np.testing.assert_allclose(anchor.position, [1.5, 3., 0.])
    anchor.correct(events)
    np.testing.assert_allclose(anchor.position, [1.5, 3., 0.])
    assert anchor.revision == 3 and anchor.stamp == 5.


def test_hold_anchor_refuses_missing_revision():
    import pytest
    from dougie_sim.map_corrections import MapCorrection
    anchor = HoldAnchor([1., 2., 0.], 5., 0)
    with pytest.raises(ValueError, match='missed a map revision'):
        anchor.correct([MapCorrection(2, 0., 10., (0., 0., 0.), (1., 0., 0.))])
    np.testing.assert_array_equal(anchor.position, [1., 2., 0.])
