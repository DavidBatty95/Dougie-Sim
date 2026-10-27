from types import SimpleNamespace

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
    monkeypatch.setattr('dougie_sim.clustered_planner.ObservedGain', lambda terrain: SimpleNamespace(value=lambda key: 0, cache={}))
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
    assert outcome(True, 'survey_complete', '') == 'survey_complete'
