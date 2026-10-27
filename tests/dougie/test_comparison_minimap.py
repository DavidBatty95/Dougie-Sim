import numpy as np
import pytest

from dougie_sim.comparison_minimap import ComparisonMinimap
from dougie_sim.survey_comparison import AVAILABLE, RADIATION, BUDGET, project_admission


def floor_points(heights):
    return np.array([[x, y, z] for z in heights for x in np.arange(-2., 2., .25)
                     for y in np.arange(-1., 1., .25)])


@pytest.mark.parametrize('world,heights', [('reactor', [0.]), ('mezzanine', [0., .7]),
                                         ('facility', [0., 1.2, 2.4])])
def test_slices_follow_actual_floor_heights_and_keep_all_routes(world, heights):
    points = floor_points(heights)
    minimap = ComparisonMinimap(points, world, [0., 0., 0.], 'blind')
    np.testing.assert_allclose(minimap.heights, heights)
    assert minimap.project([0., 1.])[1] < minimap.project([0., -1.])[1]
    for height in heights:
        minimap.record([-.5, 0.], height)
        minimap.record([.5, 0.], height)
    masks = minimap.route_masks()
    assert all(mask.any() for mask in masks)
    before = masks.copy()
    # Repeated display updates do not erase historical route segments.
    for height in heights:
        image = minimap.draw(np.zeros(len(points), dtype=bool), masks, [.5, 0.], height, 0.)
        assert image.width == 180 and image.height < 350
        assert minimap.layer(height) == heights.index(height)
    np.testing.assert_array_equal(minimap.route_masks(), before)


def test_coverage_and_admission_do_not_bleed_between_overlapping_floors():
    points = floor_points([0., 2.4])
    minimap = ComparisonMinimap(points, 'facility', [0., 0., 0.])
    seen = points[:, 2] == 0.
    admission = np.zeros(len(points), dtype=np.uint8)
    minimap.draw(seen, minimap.route_masks(), [0, 0], 0., 0., admission)
    x, y = np.rint(minimap.project([0., 0.])).astype(int)
    assert minimap.backgrounds[0].getpixel((x, y)) == minimap.explored_color
    assert minimap.backgrounds[1].getpixel((x, y)) == minimap.unseen_color
    for status, color in ((AVAILABLE, minimap.available_color), (RADIATION, minimap.radiation_color),
                          (BUDGET, minimap.budget_color), (0, minimap.unseen_color)):
        admission[~seen] = status
        minimap.draw(seen, minimap.route_masks(), [0, 0], 0., 0., admission)
        assert minimap.backgrounds[1].getpixel((x, y)) == color
        assert minimap.backgrounds[0].getpixel((x, y)) == minimap.explored_color


def test_admission_projection_is_local_and_height_aware():
    reference = np.array([[0., 0., 0.], [0., 0., 2.4], [10., 0., 0.]])
    result = project_admission(reference, [[1., 0., 0.]], [RADIATION],
                               np.eye(3), np.array([-1., 0., 0.]))
    np.testing.assert_array_equal(result, [RADIATION, 0, 0])


def test_ramp_cross_sections_do_not_displace_the_actual_mezzanine_level():
    floors = floor_points([0., .7])
    ramp = np.array([[x, y, z] for x, z in zip(np.linspace(-1, 1, 16), np.linspace(.02, .68, 16))
                     for y in np.linspace(-1, 1, 20)])
    minimap = ComparisonMinimap(np.vstack((floors, ramp)), 'mezzanine', [0, 0, 0])
    np.testing.assert_allclose(minimap.heights, [0., .7])
