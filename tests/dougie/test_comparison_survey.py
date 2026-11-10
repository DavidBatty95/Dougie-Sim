from types import SimpleNamespace
from dataclasses import replace

import numpy as np

from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.radiation import RadiationSample
from dougie_sim.survey_comparison import AVAILABLE, RADIATION, DISCONNECTED, outcome


def explorer(monkeypatch, survey=True, hot=False, disconnected=False):
    planner = ClusteredDoseExplorer(mission_duration=1e12)
    planner.survey_until_complete = survey
    terrain = planner.terrain
    terrain.nodes = {0: np.array([0., 0., 0.]), 1: np.array([.5, 0., 0.])}
    terrain.graph = {0: [] if disconnected else [1], 1: [] if disconnected else [0]}
    terrain.build_graph = lambda: (terrain.nodes, terrain.graph)
    planner.gp = SimpleNamespace(predict=lambda points: (
        np.zeros(len(points)), np.zeros(len(points)),
        np.array([200. if hot and p[0] > .1 else 0. for p in points])))
    planner.sample = RadiationSample(1., (0., 0., .44), 0, .5, 0., 0., 0., 0.)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain', lambda terrain: SimpleNamespace(value=lambda key: int(key != 0), cache={}))
    return planner


def test_full_survey_visits_remaining_supported_viewpoint_beyond_cluster_heuristic(monkeypatch):
    ordinary = explorer(monkeypatch, survey=False)
    ordinary.replan([0., 0., 0.], 1.)
    assert ordinary.returning
    sweep = explorer(monkeypatch)
    sweep.replan([0., 0., 0.], 1.)
    assert not sweep.returning and sweep.path
    np.testing.assert_allclose(sweep.target, [.5, 0., 0.])
    assert sweep.survey_status[1] == AVAILABLE
    assert sweep.last_reason == 'systematic_observed_survey'
    # No duration-based return at the old 600-second demo deadline.
    sweep.sample = RadiationSample(601., (0., 0., .44), 0, .5, 0., 0., 0., 0.)
    sweep.replan([0., 0., 0.], 601.)
    assert not sweep.returning


def test_radiation_exclusion_is_not_confused_with_disconnected_geometry(monkeypatch):
    hot = explorer(monkeypatch, hot=True)
    hot.replan([0., 0., 0.], 1.)
    assert hot.returning and hot.last_reason == 'radiation_frontiers_blocked'
    assert hot.survey_status[1] == RADIATION
    disconnected = explorer(monkeypatch, disconnected=True)
    disconnected.replan([0., 0., 0.], 1.)
    assert disconnected.survey_status[1] == DISCONNECTED
    assert disconnected.last_reason != 'radiation_frontiers_blocked'


def test_returning_or_running_out_of_viewpoints_does_not_claim_100_percent():
    assert outcome(False, 'returned', 'survey_frontiers_exhausted') == 'observed_frontiers_exhausted'
    assert outcome(False, 'returned', 'radiation_frontiers_blocked') == 'radiation_limited'
    assert outcome(False, 'returned', 'dose_return_reserve') == 'radiation_limited'
    assert outcome(False, 'stopped_unrecoverable', 'localization_unreliable', True) == 'navigation_stopped'
    assert outcome(False, 'exploring', 'time_limit') == 'time_limit'
    assert outcome(True, 'finished', 'frontiers_exhausted') == 'frontiers_exhausted'


def test_open_ended_survey_keeps_exploring_after_large_accumulated_dose(monkeypatch):
    planner = explorer(monkeypatch)
    planner.budget_enabled = False
    planner.sample = replace(planner.sample, dose_usv=500., dose_upper_usv=600.)
    planner.replan([0., 0., 0.], 1.)
    assert not planner.returning and planner.path
    assert planner.survey_status[1] == AVAILABLE
    assert planner.return_reserve == 0.  # no arbitrary percentage-of-budget margin
    # Disabling the cumulative cap must not remove instantaneous risk avoidance.
    hot = explorer(monkeypatch, hot=True)
    hot.budget_enabled = False
    hot.replan([0., 0., 0.], 1.)
    assert hot.returning and hot.survey_status[1] == RADIATION


def test_dose_minimization_can_finish_a_hot_viewpoint_without_rate_triggered_return(monkeypatch):
    planner = explorer(monkeypatch, hot=True)
    planner.budget_enabled = planner.rate_limited = False
    planner.sample = replace(planner.sample, dose_usv=500., dose_upper_usv=600.)
    planner.replan([0., 0., 0.], 1.)
    assert not planner.returning and planner.path
    assert planner.survey_status[1] == AVAILABLE
    assert planner.route_forecast['emergency_rate_override'] is False


def test_frontier_exhaustion_requires_three_distinct_fresh_scans(monkeypatch):
    planner = explorer(monkeypatch)
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain',
                        lambda terrain: SimpleNamespace(value=lambda key: 0, cache={}))
    planner.last_packet_sequence = 1
    planner.replan([0., 0., 0.], 1.)
    assert planner.frontier_count == 0 and not planner.frontier_complete
    # Replanning the same observation cannot confirm completion twice.
    planner.replan([0., 0., 0.], 1.)
    assert planner.frontier_confirmation_count == 1
    for sequence in (2, 3):
        planner.last_packet_sequence = sequence
        planner.sample = replace(planner.sample, stamp_s=float(sequence))
        planner.replan([0., 0., 0.], float(sequence))
    assert planner.frontier_complete and planner.state == 'finished'
    assert planner.last_reason == 'frontiers_exhausted' and not planner.path


def test_visiting_a_frontier_does_not_hide_remaining_unknown_space(monkeypatch):
    planner = explorer(monkeypatch)
    planner.visited.add(1)
    planner.replan([0., 0., 0.], 1.)
    assert planner.frontier_count == 1 and not planner.frontier_complete
    assert planner.path and planner.state == 'exploring'


def test_disconnected_frontiers_do_not_count_as_finished(monkeypatch):
    planner = explorer(monkeypatch, disconnected=True)
    planner.replan([0., 0., 0.], 1.)
    assert planner.frontier_count == 1 and not planner.frontier_complete
    assert planner.last_reason == 'unresolved_frontiers'
    assert outcome(False, 'returned', 'unresolved_frontiers') == 'frontiers_blocked'
