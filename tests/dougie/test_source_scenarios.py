import json

import numpy as np
import pytest

from dougie_sim.radiation import RadiationField
from dougie_sim.reactor import radiation_description
from dougie_sim.simulation import Simulation
from dougie_sim.source_config import DEFAULT_SOURCE_CONFIG, load_source_profile


def test_combined_field_is_sum_of_hotspots_and_leaks_with_one_background():
    hot = RadiationField(radiation_description('hotspots'))
    leaks = RadiationField(radiation_description('leaks'))
    combined = RadiationField(radiation_description('hotspots-and-leaks'))
    assert len(combined.sources) == 6
    for point in ([0., 0., .44], [2.6, -1.8, .12], [-1.5, 2.8, 2.8], [5., 2., 1.]):
        assert combined.rate(point) == pytest.approx(hot.rate(point)+leaks.rate(point)-.2)
        assert np.isfinite(combined.rate(point))


def test_hotspot_has_a_finite_concentrated_peak_and_inverse_square_far_field():
    source = load_source_profile('hotspots')['sources'][2]
    field = RadiationField(dict(background_usv_h=0., sources=[source]))
    origin = np.asarray(source['position'])
    peak = field.rate(origin)
    assert peak == pytest.approx(source['strength_usv_h_m2']/source['core_radius_m']**2)
    assert peak > 100*field.rate(origin+[1, 0, 0])
    assert field.rate(origin+[1, 0, 0])/field.rate(origin+[2, 0, 0]) == pytest.approx(4., rel=.002)


def test_named_sources_survive_mujoco_round_trip_without_changing_geometry():
    description = radiation_description('hotspots-and-leaks')
    sim = Simulation('reactor', radiation_sources=description)
    field = RadiationField.from_mujoco(sim.model, sim.data)
    assert field.sources == description['sources']
    assert {s['kind'] for s in field.sources} == {'hotspot', 'leak'}
    assert all(sim.model.site('radiation_source_'+s['id']).group == 5 for s in field.sources)
    baseline = Simulation('reactor')
    assert sim.model.ngeom == baseline.model.ngeom
    np.testing.assert_allclose(sim.scan(), baseline.scan())


def test_baseline_field_preserves_original_values():
    description = radiation_description('baseline')
    reference = RadiationField(dict(background_usv_h=.2, shields=description['shields'], sources=[
        dict(position=[5.8, 3.8, 1.2], strength_usv_h_m2=450., core_radius_m=.25),
        dict(position=[-5.2, 4.8, .6], strength_usv_h_m2=65., core_radius_m=.2),
        dict(position=[4.2, -3., 1.1], strength_usv_h_m2=35., core_radius_m=.15)]))
    sim = Simulation('reactor', radiation_sources=description)
    actual = RadiationField.from_mujoco(sim.model, sim.data)
    for point in ([0., 0., .44], [3., 3., 1.], [-5., 4., .5], [4., -3., 1.]):
        assert actual.rate(point) == pytest.approx(reference.rate(point), rel=1e-12)


def test_custom_profile_disabled_source_and_custom_background(tmp_path):
    document = json.loads(DEFAULT_SOURCE_CONFIG.read_text())
    document['sources']['pump_seal_leak']['enabled'] = False
    document['profiles']['custom'] = dict(background_usv_h=.75, sources=['pump_seal_leak', 'drain_leak'])
    path = tmp_path/'sources.json'
    path.write_text(json.dumps(document))
    description = radiation_description('custom', path)
    assert [s['id'] for s in description['sources']] == ['drain_leak']
    sim = Simulation('reactor', radiation_sources=description)
    field = RadiationField.from_mujoco(sim.model, sim.data)
    assert field.background == .75


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), True])
def test_invalid_source_core_is_rejected(tmp_path, value):
    document = json.loads(DEFAULT_SOURCE_CONFIG.read_text())
    document['sources']['drain_leak']['core_radius_m'] = value
    path = tmp_path/'sources.json'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError):
        load_source_profile('leaks', path)


@pytest.mark.parametrize('change', ['duplicate', 'missing', 'bad_position', 'bad_strength', 'unknown_profile'])
def test_malformed_scenarios_are_rejected(tmp_path, change):
    document = json.loads(DEFAULT_SOURCE_CONFIG.read_text())
    if change == 'duplicate':
        document['profiles']['leaks']['sources'].append('drain_leak')
    elif change == 'missing':
        document['profiles']['leaks']['sources'].append('nonexistent')
    elif change == 'bad_position':
        document['sources']['drain_leak']['position'] = [0., 1.]
    elif change == 'bad_strength':
        document['sources']['drain_leak']['strength_usv_h_m2'] = -2.
    path = tmp_path/'sources.json'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError):
        load_source_profile('unknown' if change == 'unknown_profile' else 'leaks', path)
