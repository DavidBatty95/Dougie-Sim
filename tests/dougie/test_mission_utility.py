from dataclasses import replace
from types import SimpleNamespace as S

import numpy as np

from dougie_sim.mission_utility import ExposureUtility, CountCalibration
from dougie_sim.observed_circuit import ObservedCircuit
from dougie_sim.circuit_evaluation import CircuitWitness
from dougie_sim.radiation import RadiationSample
from dougie_sim.terrain import SurfaceMap


def utility_score(utility, **changes):
    values = dict(point=[0., 0., .44], novelty=0., fresh_fraction=0., mean_rate=10., sigma=.2,
        outbound_s=10., return_s=10., outbound_dose=.01, return_dose=.01, goal_upper=15.,
        current_upper=15., wait_s=3., recovery_s=3., remaining_time=120., remaining_budget=1.)
    values.update(changes)
    return utility.score(**values)


def test_information_score_charges_return_turn_dwell_and_wait_exposure():
    utility = ExposureUtility()
    a = utility_score(utility)
    assert utility_score(utility, return_dose=.2)['score'] < a['score']
    assert utility_score(utility, return_s=60.)['score'] < a['score']
    assert utility_score(utility, wait_s=10.)['score'] < a['score']
    assert a['upper_round_trip_dose_usv'] > .02
    assert a['round_trip_duration_s'] == 34.


def test_repeated_measurements_reduce_utility_but_uncertain_revisits_remain_possible():
    utility = ExposureUtility()
    fresh = utility_score(utility)
    sample = RadiationSample(.5, (0., 0., .44), 60, .5, 8.64, 1., .001, .002)
    for i in range(20):
        utility.observe(replace(sample, stamp_s=(i+1)*.5))
    repeated = utility_score(utility)
    assert repeated['observed_seconds'] == 10.
    assert 0 < repeated['score'] < fresh['score']
    assert utility_score(utility, sigma=1.5)['score'] > repeated['score']


def test_count_calibration_scores_before_assimilation_without_changing_bounds():
    calls = []
    gp = S(predict=lambda p: (calls.append(len(p)) or np.ones(len(p)), np.ones(len(p)), np.full(len(p), 2.)))
    calibration = CountCalibration()
    sample = RadiationSample(.5, (0., 0., .44), 100, .5, 14.4, 1., .001, .002)
    calibration.observe(gp, sample)
    assert calls == [1]
    assert calibration.metrics()['upper_count_exceedances'] == 1
    calibration.observe(gp, replace(sample, position_valid=False))
    assert calibration.windows == 1


def grid():
    terrain = SurfaceMap()
    terrain.nodes = {(x, y, 0): np.array([x*.25, y*.25, 0.]) for x in range(-10, 11) for y in range(-10, 11)}
    terrain.graph = {k: [(k[0]+dx, k[1]+dy, 0) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                        if (k[0]+dx, k[1]+dy, 0) in terrain.nodes] for k in terrain.nodes}
    return terrain


def test_circuit_freezes_only_observed_goals_and_requires_all_four():
    terrain, circuit = grid(), ObservedCircuit()
    assert circuit.prepare(terrain, (0, 0, 0), np.zeros(3), 1., 7)
    np.testing.assert_allclose(circuit.initial_goals, [[1.5, 0., 0.], [0., 1.5, 0.], [-1.5, 0., 0.], [0., -1.5, 0.]])
    initial = np.array(circuit.initial_goals)
    terrain.nodes.clear()
    assert circuit.prepare(terrain, (0, 0, 0), np.zeros(3), 2., 8)
    np.testing.assert_array_equal(circuit.initial_goals, initial)
    for goal in initial[:3]:
        circuit.observe_position(goal, 3.)
    assert not circuit.completed
    circuit.observe_position(initial[3], 4.)
    assert circuit.completed


def test_unknown_or_disconnected_direction_cannot_become_circuit_goal():
    terrain, circuit = grid(), ObservedCircuit()
    terrain.graph = {k: [n for n in neighbours if not (k[0] == 3 and n[0] == 4)] for k, neighbours in terrain.graph.items()}
    assert not circuit.prepare(terrain, (0, 0, 0), np.zeros(3), 1., 7)
    assert not circuit.goals


def test_physical_circuit_witness_is_independent_and_requires_dwell():
    circuit = ObservedCircuit()
    terrain = grid()
    circuit.prepare(terrain, (0, 0, 0), np.zeros(3), 1., 7)
    witness = CircuitWitness()
    witness.set_goals(circuit.metrics(), (np.eye(3), np.array([.1, 0., 0.])))
    for goal in circuit.initial_goals:
        circuit.observe_position(np.asarray(goal), 2.)
    assert circuit.completed and not witness.metrics()['all_goals_reached']
    for goal in witness.goals:
        witness.tick(goal, 3., .1)
        witness.tick(goal, 3.1, .1)
    assert witness.metrics()['all_goals_reached']


def test_whole_task_lookahead_uses_checked_diagonals_instead_of_crossing_home():
    terrain, circuit = grid(), ObservedCircuit()
    root = (0, 0, 0)
    circuit.prepare(terrain, root, np.zeros(3), 1., 7)
    rates = {key: 1. for key in terrain.nodes}
    gp = S(predict=lambda p: (np.ones(len(p)), np.ones(len(p))*.1, np.ones(len(p))))
    pending = circuit.pending(terrain)
    tour = circuit.best_tour(terrain, root, root, np.zeros(3), np.zeros(3), rates, gp, 100.,
                            set(pending), 300., 2., None)
    assert tour is not None and len(tour['order']) == 4
    assert tour['length_m'] < 10.  # Four cardinal goals: crossed grid tour is 12 m.
    assert tour['first'] in pending
