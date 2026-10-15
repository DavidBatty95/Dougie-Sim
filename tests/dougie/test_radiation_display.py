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


def test_cloud_is_local_translucent_and_does_not_modify_posterior():
    display = RadiationHighlights(.2)
    query = np.array([[0., 0., .44], [0., 0., 2.84], [10., 0., .44]])
    mean = np.array([4., 40., 40.])
    volume = (query, mean, np.ones(3), mean*2)
    original = [v.copy() for v in volume]
    assert len(display.cloud(volume)[0]) == 0
    display.observe(sample(.5, 100))
    points, colors = display.cloud(volume)
    assert len(points) == 7
    assert np.all(np.linalg.norm(points-query[0], axis=1) <= .3)
    assert np.all((colors[:, 3] > 0) & (colors[:, 3] < .3))
    for actual, expected in zip(volume, original):
        np.testing.assert_array_equal(actual, expected)


def test_cloud_payload_and_mujoco_geometry_are_bounded():
    import mujoco
    from dougie_sim.demo_overlay import DemoControls, draw_overlay
    from dougie_sim.simulation import Simulation
    display = RadiationHighlights()
    volume = (np.zeros((450, 3)), np.ones(450), np.ones(450), np.ones(450))
    points, colors = display.cloud(volume, mode='all')
    assert 0 < len(points) == len(colors) <= 1200
    sparse = tuple(v[:1] for v in volume)
    sparse_points, sparse_colors = display.cloud(sparse, mode='all')
    np.testing.assert_allclose(points, sparse_points)
    np.testing.assert_allclose(colors, sparse_colors)
    sim = Simulation(controller='stand')
    scene = mujoco.MjvScene(sim.model, maxgeom=20)
    controls = DemoControls(show_sources=True)
    controls.field_display = 'all'
    controls.sources = [('test', np.array([0., 0., .5]), np.array([1., .1, .1, 1.]), 2.)]
    draw_overlay(scene, volume, {}, [], controls, 100.)
    assert scene.ngeom <= scene.maxgeom
    assert any('TRUE SOURCE' in scene.geoms[i].label for i in range(scene.ngeom))


def test_cloud_interpolates_local_gradient_without_connecting_floors():
    display = RadiationHighlights()
    xyz = np.array([[0., 0., .48], [.36, 0., .48], [0., 0., 2.88]])
    rates = np.array([1., 10., 100.])
    points, colors = display.cloud((xyz, rates, np.ones(3), rates*2), mode='all')
    row = (np.abs(points[:, 1]) < 1e-8) & (np.abs(points[:, 2]-.48) < 1e-8)
    green = colors[row, 1][np.argsort(points[row, 0])]
    assert np.all(np.diff(green) <= 1e-12) and green[0] > green[-1]
    assert not np.any((points[:, 2] > .8) & (points[:, 2] < 2.5))
