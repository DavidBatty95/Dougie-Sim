from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from dougie_sim.comparison_stream import FloorCoverage


def scene(height):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <geom name="floor" type="plane" size="0 0 .1"/>
      <geom name="upper_floor_test" type="box" pos="0 0 {height-.1}" size="1 1 .1"/>
      <geom name="service_ramp_lower" type="box" pos="4 0 .6" size="1 .5 .05"
            quat="0.9961947 0 -0.0871557 0"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return FloorCoverage(SimpleNamespace(model=model, data=data), 'facility')


@pytest.mark.parametrize('height', [.7, 2.4])
def test_visible_upper_floor_requires_physical_arrival_and_fresh_scan(height):
    coverage = scene(height)
    upper = coverage.flat & np.isclose(coverage.points[:, 2], height)
    assert upper.any()
    # Even perfect 3D returns from every storey do not credit a floor from below.
    coverage.observe(coverage.points, support_position=[-2., 0., 0.])
    assert coverage.seen[coverage.points[:, 2] == 0.].any()
    assert not coverage.seen[upper].any()
    # Elevation alone, without proximity to that physical floor, is insufficient.
    coverage.observe(coverage.points, support_position=[100., 100., height])
    assert not coverage.seen[upper].any()
    # Entering the minimap's upper display band halfway up is not arrival upstairs.
    coverage.observe(coverage.points, support_position=[0., 0., height/2+.01])
    assert not coverage.seen[upper].any()
    # Arrival does not retrospectively turn earlier glimpses into exploration.
    coverage.observe([], support_position=[0., 0., height])
    assert coverage.reached_levels[np.isclose(coverage.level_heights, height)].all()
    assert not coverage.seen[upper].any()
    subset = coverage.points[upper][::2]
    coverage.observe(subset, support_position=[0., 0., height])
    assert 0 < coverage.seen[upper].sum() < upper.sum()
    before = coverage.seen.copy()
    # Back downstairs, additional upper-storey returns cannot fill in the rest.
    coverage.observe(coverage.points[upper], support_position=[0., 0., 0.])
    np.testing.assert_array_equal(coverage.seen, before)
    coverage.observe(coverage.points[upper], support_position=[0., 0., height])
    assert coverage.seen[upper].all()


def test_slopes_are_explored_progressively_by_support_height():
    coverage = scene(2.4)
    slope = ~coverage.flat
    assert slope.any()
    coverage.observe(coverage.points[slope], support_position=[0., 0., 0.])
    assert not coverage.seen[slope].any()
    slope_points = coverage.points[slope]
    lowest = slope_points[np.argmin(slope_points[:, 2])]
    coverage.observe(slope_points, support_position=lowest)
    credited = coverage.points[coverage.seen]
    assert len(credited)
    assert np.all(abs(credited[:, 2]-lowest[2]) <= coverage.observation_height_tolerance+1e-9)
    assert not coverage.seen[slope].all()
    for position in slope_points:
        coverage.observe(slope_points, support_position=position)
    assert coverage.seen[slope].all()
    assert not coverage.seen[coverage.points[:, 2] > 2.].any()


def test_missing_or_invalid_capture_pose_cannot_silently_credit_exploration():
    coverage = scene(2.4)
    with pytest.raises(TypeError):
        coverage.observe(coverage.points)
    for position in ([0., 0.], [0., 0., float('nan')]):
        with pytest.raises(ValueError):
            coverage.observe(coverage.points, support_position=position)
    assert coverage.percent == 0.
