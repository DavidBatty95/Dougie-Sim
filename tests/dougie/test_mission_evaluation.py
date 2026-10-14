from copy import deepcopy

import pytest

from dougie_sim.mission_evaluation import round_trip_failures


def completed():
    return dict(world='facility', error=None, end_state='returned', mission_completed=True,
        clean_return=True, true_dose_usv=.5, budget_usv=5., actual_budget_exceeded=False,
        nonfoot_impulse_ns=0., nonfoot_peak_force_n=0.,
        simulation=dict(upright=.99, simulation_time_s=120.),
        physical_witness=dict(upper_landing_witness_s=30., lower_return_witness_s=90.,
                              vertical_round_trip_witness=True))


def test_round_trip_requires_ordered_witnesses_even_after_clean_home_return():
    summary = completed()
    assert round_trip_failures(summary) == []
    summary['physical_witness']['upper_landing_witness_s'] = None
    assert 'missing_ordered_vertical_witness' in round_trip_failures(summary)


@pytest.mark.parametrize('upper,lower,end', [(90., 30., 120.), (30., 130., 120.),
    (-1., 30., 120.), (30., float('nan'), 120.), (30., 30., 120.)])
def test_invalid_witness_timestamps_fail_closed(upper, lower, end):
    summary = completed()
    summary['physical_witness'].update(upper_landing_witness_s=upper, lower_return_witness_s=lower)
    summary['simulation']['simulation_time_s'] = end
    assert 'missing_ordered_vertical_witness' in round_trip_failures(summary)


@pytest.mark.parametrize('key,value', [('true_dose_usv', float('nan')), ('budget_usv', -1.),
    ('nonfoot_impulse_ns', .2), ('nonfoot_peak_force_n', 26.), ('clean_return', False),
    ('end_state', 'exploring'), ('actual_budget_exceeded', True), ('world', 'reactor')])
def test_invalid_or_failed_outcomes_are_not_round_trips(key, value):
    summary = deepcopy(completed())
    summary[key] = value
    assert round_trip_failures(summary)
