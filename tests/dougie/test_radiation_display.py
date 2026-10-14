import numpy as np

from dougie_sim.radiation import RadiationSample
from dougie_sim.radiation_display import RadiationHighlights


def sample(stamp, counts, position=(0., 0., .44)):
    return RadiationSample(stamp, position, counts, .5, counts*.144, (counts+1)*.144**2,
                           .001, .002, 50000.)


def test_uncertain_prior_and_background_counts_do_not_paint_a_hazard():
    display = RadiationHighlights(.2)
    query = np.array([[0., 0., .44], [4., 0., 2.84]])
    assert not display.mask(query, [6., 40.]).any()
    for i in range(20):
        display.observe(sample((i+1)*.5, 1 if i % 2 else 2))
    assert len(display.confirmed()[0]) == 0
    assert not display.mask(query, [6., 40.]).any()


def test_confirmed_counts_enable_only_nearby_above_background_predictions():
    display = RadiationHighlights(.2)
    display.observe(sample(.5, 100))
    query = np.array([[0., 0., .44], [.3, .1, .6], [0., 0., 2.84], [5., 0., .44], [.1, 0., .44]])
    mean = np.array([14., 5., 100., 20., .15])
    original = mean.copy()
    np.testing.assert_array_equal(display.mask(query, mean), [True, True, False, False, False])
    np.testing.assert_array_equal(mean, original)
    assert display.mask(query, mean, 'all').all()


def test_redraws_do_not_double_count_and_display_storage_is_bounded():
    display = RadiationHighlights(.2, capacity=1)
    reading = sample(.5, 100)
    display.observe(reading)
    before = display.confirmed()[1].copy()
    display.observe(reading)
    np.testing.assert_array_equal(display.confirmed()[1], before)
    display.observe(sample(1., 100, (5., 5., .44)))
    assert len(display.cells) == 1


def test_detected_colors_are_prominent_and_increase_towards_red():
    colors = RadiationHighlights(.2).colors([.3, 2., 50.])
    assert np.all(colors[:, 0] == 1.) and np.all(colors[:, 3] >= .8)
    assert colors[0, 1] > colors[-1, 1]
