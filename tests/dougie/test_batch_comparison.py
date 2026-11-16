import json
import sys
import threading

import numpy as np
import pytest

from dougie_sim import batch_comparison as batch


def protocol():
    return dict(seeds=[1, 2], world='facility', seed_axis='detector', detector_seed=1,
                sensor_seed=0, layout_seed=0, vary_layout=False, duration=0.,
                localization='ground-truth', radiation_policy='minimize-dose', rate_limit=100.,
                milestones=[50., 90.], curve_step_s=10.)


def row(time, dose, floor):
    return dict(time_s=time, true_dose_usv=dose, explored_percent=floor,
                distance_m=time, frontier_count=3, frontier_complete=False)


def pair(seed, blind, aware, state='terminal', censored=False):
    return dict(seed=seed, settings=dict(seed=seed, sensor_seed=0, layout_seed=0), status=dict(state=state, censored=censored), runs={
        mode:dict(rows=rows, exit_code=0, slices={'0.00': rows[-1]['explored_percent']},
                  summary=dict(frontier_complete=False, end_state='returned', error=None))
        for mode, rows in (('blind', blind), ('aware', aware))})


def test_seed_axis_keeps_each_pair_matched_and_handles_nonconsecutive_seeds():
    p = protocol()
    assert batch.seed_settings(2, p) == dict(seed=2, sensor_seed=0, layout_seed=0)
    p.update(seeds=[8, 3], vary_layout=True, seed_axis='both')
    assert batch.seed_settings(3, p) == dict(seed=3, sensor_seed=3, layout_seed=1)


def test_endpoint_averages_keep_incomplete_runs_but_milestones_require_both():
    first = pair(1, [row(0, 0, 0), row(10, 2, 50)], [row(0, 0, 0), row(10, 1, 50)])
    second = pair(2, [row(0, 0, 0), row(10, 4, 60)], [row(0, 0, 0), row(10, .1, 20)])
    report = batch.aggregate([first, second], protocol())
    assert report['modes']['blind']['endpoint']['true_dose_usv']['mean'] == 3.
    assert report['modes']['aware']['endpoint']['true_dose_usv']['mean'] == .55
    assert report['modes']['aware']['confirmed_frontier_completions'] == 0
    milestone = report['milestones'][0]
    assert milestone['eligible_pairs'] == 1 and milestone['excluded_or_not_reached_pairs'] == 1
    assert milestone['paired_aware_minus_blind_dose']['mean'] == -1.
    assert milestone['paired_relative_reduction_percent']['mean'] == 50.
    assert report['milestones'][1]['eligible_pairs'] == 0


def test_mean_curves_never_pad_finished_trajectories():
    first = pair(1, [row(0, 0, 0), row(20, 20, 60)], [row(0, 0, 0), row(10, 1, 50)])
    second = pair(2, [row(0, 0, 0), row(30, 30, 60)], [row(0, 0, 0), row(30, 6, 50)])
    curves = batch.aggregate([first, second], protocol())['curves']
    at_ten = next(r for r in curves if r['time_s'] == 10.)
    at_twenty = next(r for r in curves if r['time_s'] == 20.)
    assert at_ten['paired_n'] == 2 and at_twenty['paired_n'] == 1
    assert at_twenty['aware']['true_dose_usv']['mean'] == 4.
    assert at_twenty['aware']['true_dose_usv']['mean_ci95'] is None


def test_failures_and_censoring_remain_in_denominators():
    failed = pair(1, [row(0, 0, 0), row(10, 2, 50)], [row(0, 0, 0), row(10, 1, 50)])
    failed['runs']['aware'].update(exit_code=1, summary=dict(error='fell', frontier_complete=False))
    timeout = pair(2, [row(0, 0, 0), row(10, 2, 50)], [row(0, 0, 0), row(10, 1, 50)], 'timeout', True)
    timeout['runs']['aware']['summary'].update(frontier_complete=True, end_state='finished')
    report = batch.aggregate([failed, timeout], protocol())
    assert report['terminal_pairs'] == 2 and report['censored_pairs'] == 1
    assert report['modes']['aware']['process_errors'] == 1
    assert report['modes']['aware']['confirmed_frontier_completions'] == 0
    assert report['modes']['aware']['completion_denominator'] == 2


def test_statistics_expose_missing_uncertainty_and_support_adverse_results():
    assert batch.statistics([])['mean'] is None
    assert batch.statistics([5.])['mean_ci95'] is None
    stats = batch.statistics([1., 2., 3.])
    assert stats['mean'] == 2. and stats['sd'] == 1.
    assert stats['mean_ci95'][0] <= 2. <= stats['mean_ci95'][1]
    np.testing.assert_allclose(stats['mean_ci95'], batch.statistics([1., 2., 3.])['mean_ci95'])


def test_wall_timeout_stops_both_child_sessions_and_retains_status(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, 'source_hashes', lambda: {})
    monkeypatch.setattr(batch, 'mission_command', lambda *args: [sys.executable, '-c', 'import time; time.sleep(30)'])
    batch.run_pair(1, tmp_path, protocol(), {}, threading.Event(), .1)
    status = json.loads((tmp_path/'seed_00001/pair_status.json').read_text())
    assert status['state'] == 'timeout' and status['censored']
    assert set(status['exit_codes']) == {'blind', 'aware'}
    assert all(isinstance(code, int) for code in status['exit_codes'].values())
    # Resumption must not rerun an already terminal timed-out pair and hide it.
    original = (tmp_path/'seed_00001/pair_status.json').read_bytes()
    batch.run_pair(1, tmp_path, protocol(), {}, threading.Event(), .1)
    assert (tmp_path/'seed_00001/pair_status.json').read_bytes() == original


def test_running_summary_is_not_read_while_the_worker_may_be_writing_it(tmp_path):
    directory = tmp_path/'seed_00001/aware'
    directory.mkdir(parents=True)
    (directory/'radiation_summary.json').write_text('{in progress')
    batch.write_json(directory.parent/'pair_status.json', dict(state='running', censored=False))
    pairs = batch.load_pairs(tmp_path, protocol())
    assert pairs[0]['runs']['aware']['summary'] is None
    assert pairs[1]['status']['state'] == 'pending'


def test_confirmed_completion_requires_valid_uncensored_frontier_evidence():
    completed = pair(1, [row(0, 0, 0), row(10, 2, 40)], [row(0, 0, 0), row(10, 1, 30)])
    completed['runs']['aware']['summary'].update(frontier_complete=True, end_state='finished',
        frontier_count=0, frontier_confirmation_count=3)
    report = batch.aggregate([completed], protocol())
    assert report['modes']['aware']['confirmed_frontier_completions'] == 1
    completed['runs']['aware']['summary']['frontier_count'] = 1
    assert batch.aggregate([completed], protocol())['modes']['aware']['confirmed_frontier_completions'] == 0


def test_resume_refuses_protocol_drift_before_launching_workers(tmp_path, monkeypatch, capsys):
    batch.write_json(tmp_path/'batch_config.json', dict(protocol={'world': 'reactor'}, source_sha256={}))
    monkeypatch.setattr(batch, 'source_hashes', lambda: {})
    monkeypatch.setattr(sys, 'argv', ['batch', '--output', str(tmp_path), '--resume'])
    with pytest.raises(SystemExit) as error:
        batch.main()
    assert error.value.code == 2
    assert 'resume protocol or runtime source hashes differ' in capsys.readouterr().err


def test_corrupt_terminal_artifacts_are_retained_and_counted(tmp_path):
    directory = tmp_path/'seed_00001/aware'
    directory.mkdir(parents=True)
    (directory/'radiation_summary.json').write_text('{interrupted write')
    (directory/'minimap_final.npz').write_bytes(b'partial file')
    batch.write_json(directory.parent/'pair_status.json', dict(state='failed', censored=False))
    pairs = batch.load_pairs(tmp_path, protocol())
    assert len(pairs[0]['runs']['aware']['artifact_errors']) == 2
    report = batch.aggregate(pairs, protocol())
    assert report['modes']['aware']['missing_summaries'] == 1
    assert report['modes']['aware']['artifact_errors'] == 1
    assert (directory/'minimap_final.npz').read_bytes() == b'partial file'
