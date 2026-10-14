from dataclasses import replace
import json

import mujoco
import numpy as np
import pytest

from dougie_sim.assets import ROOT
from dougie_sim.simulation import Simulation
from dougie_sim.terrain_courses import Course, CourseSpec, catalogue
from dougie_sim.terrain_qualification import aggregate, observed_target, qualification_failures

PROTOCOL = json.loads((ROOT/'config/dougie_terrain_protocol.json').read_text())


@pytest.mark.parametrize('name', ['stairs_up_060', 'stairs_down_060', 'legacy_06', 'ramp_up_15', 'ramp_down_15',
                                  'ramp_up_10_long', 'ramp_down_10_pause', 'tiles_040'])
def test_reference_height_matches_actual_collision_geometry(name):
    course = Course(catalogue()[name], 4)
    sim = Simulation(model_xml=course.xml)
    groups = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    geom = np.empty(1, dtype=np.int32)
    for x in np.linspace(.05, course.spec.finish_x, 25):
        origin = np.array([x, .031, 5.])
        distance = mujoco.mj_ray(sim.model, sim.data, origin, np.array([0., 0., -1.]), groups, 1, sim.base, geom)
        assert 5.-distance == pytest.approx(course.surface_height(x, .031), abs=1e-6)


@pytest.mark.parametrize('friction', [.2, .8])
def test_friction_variation_changes_effective_contacts_not_just_visual_floor(friction):
    course = Course(CourseSpec('flat', kind='flat', friction=friction), 1)
    sim = Simulation(model_xml=course.xml)
    values = []
    for _ in range(100):
        sim.step()
        values.extend(c.friction[0] for c in sim.data.contact)
    assert values
    np.testing.assert_allclose(values, friction)


def test_payload_has_mass_and_half_step_keeps_policy_at_50_hz():
    plain = Simulation(model_xml=Course(CourseSpec('flat', kind='flat'), 0).xml)
    loaded = Simulation(model_xml=Course(CourseSpec('loaded', kind='flat', payload_kg=2., timestep=.0025), 0).xml)
    assert loaded.model.body_mass.sum()-plain.model.body_mass.sum() == pytest.approx(2.)
    assert loaded.control_decimation == 8
    assert loaded.control_decimation*loaded.model.opt.timestep == pytest.approx(.02)


def passing_metrics():
    return dict(data_valid=True, terminal_reason='completed', upper_landing_witness=True,
        nonfoot_impulse_ns=0., nonfoot_peak_force_n=0., max_penetration_m=.001,
        max_joint_limit_violation_rad=0., settled_speed_m_s=.02)


def test_crossing_with_leg_impacts_is_not_qualified():
    metrics = passing_metrics()
    metrics.update(nonfoot_impulse_ns=10., nonfoot_peak_force_n=50.)
    assert qualification_failures(metrics, PROTOCOL) == ['nonfoot_impact']


@pytest.mark.parametrize('change,reason', [
    ({'terminal_reason': 'left_test_corridor'}, 'left_test_corridor'),
    ({'data_valid': False}, 'invalid_simulation_data'),
    ({'upper_landing_witness': False}, 'missing_upper_landing_witness'),
    ({'settled_speed_m_s': .5}, 'failed_to_settle'),
])
def test_evaluator_rejects_bypass_invalid_data_and_unsettled_completion(change, reason):
    metrics = {**passing_metrics(), **change}
    assert reason in qualification_failures(metrics, PROTOCOL)


def test_repeatability_claim_needs_enough_repeats_and_counts_failures():
    record = dict(case='test', qualified=True, traversed=True, reasons=[], simulation_time_s=10.)
    assert not aggregate([record]*5, PROTOCOL)[0]['meets_repeatability_target']
    assert aggregate([record]*60, PROTOCOL)[0]['meets_repeatability_target']
    failed = {**record, 'qualified': False, 'reasons': ['nonfoot_impact']}
    assert not aggregate([record]*59+[failed], PROTOCOL)[0]['meets_repeatability_target']


def test_seeded_geometry_repeats_and_invalid_specs_are_rejected():
    case = catalogue()['tiles_040']
    assert Course(case, 12).xml == Course(case, 12).xml
    assert Course(case, 12).xml != Course(case, 13).xml
    with pytest.raises(ValueError):
        replace(case, friction=float('nan'))
    with pytest.raises(ValueError):
        replace(case, width=.1)


def test_observed_goal_is_beyond_finish_plane_even_with_cell_tolerance():
    nodes = {'before': np.array([5.625, .125, 0.]), 'after': np.array([6.125, .125, 0.])}
    target = observed_target(nodes, nodes, 5.7)
    assert target == 'after'
    assert nodes[target][0]-.22 > 5.7
