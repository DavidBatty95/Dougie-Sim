"""Exercise the real mission lifecycle and portable multi-level artifacts."""
import json
import subprocess
import sys

import numpy as np
import pytest
from dougie_sim.spatial_gp import load_gp


@pytest.mark.parametrize('require_clean', [False, True])
@pytest.mark.parametrize('gp_mode', ['global', 'spatial'])
def test_short_mezzanine_run_archives_resolved_scene_and_observed_geometry(tmp_path, require_clean, gp_mode):
    output = tmp_path/'mission'
    run = subprocess.run([sys.executable, '-m', 'dougie_sim.radiation_run', '--world', 'mezzanine',
        '--headless', '--fast', '--duration', '1', '--fixed-plan-delay', '.5', '--gp-mode', gp_mode, '--output', str(output),
        *(['--require-clean'] if require_clean else [])],
        capture_output=True, text=True, timeout=45)
    assert run.returncode == int(require_clean), run.stdout+run.stderr  # A time-limited non-return must fail strict evaluation.
    summary = json.loads((output/'radiation_summary.json').read_text())
    config = json.loads((output/'run_config.json').read_text())
    scenario = json.loads((output/'source_scenario.json').read_text())
    evaluator = json.loads((output/'evaluator_field.json').read_text())
    assert config['world'] == 'mezzanine' and config['terrain_profile'] == 'layered'
    assert len(scenario['shields']) == 2 and scenario['shields'] == evaluator['shields']
    assert config['policy'] == summary['simulation']['policy']
    assert not summary['exploration_complete']
    assert summary['performance']['planner_execution'] == 'spawned_process'
    assert summary['gp']['updates'] == 2  # Includes the final pending detector window.
    gp = load_gp(output/'gp_posterior.npz')
    assert gp.updates == 2
    with np.load(output/'radiation_map_3d.npz') as volume:
        np.testing.assert_allclose(gp.predict(volume['xyz'])[0], volume['mean_usv_h'], rtol=1e-10)
    assert not summary['physical_witness']['vertical_round_trip_witness']
    assert not summary['round_trip_completed']
    assert 'missing_ordered_vertical_witness' in summary['round_trip_failures']
    assert 'access_ramp_0' in (output/'scene.xml').read_text()
    with np.load(output/'layered_geometry.npz') as geometry:
        assert int(geometry['schema_version']) == 2
        assert geometry['coarse_free_keys'].shape[1] == 3
        assert len(geometry['coarse_free_keys']) == len(geometry['coarse_free_evidence'])
        assert len(geometry['occupancy_keys']) == len(geometry['occupancy_evidence']) > 0
        assert geometry['support_returns'].shape[1] == 3
    with np.load(output/'motion_trace.npz') as motion:
        assert len(motion['rows']) > 0
        assert motion['rows'].shape[1] == len(motion['columns'])
        assert np.isfinite(motion['rows']).all()


def test_round_trip_cli_implies_clean_and_rejects_missing_witnesses(tmp_path):
    output = tmp_path/'round_trip'
    run = subprocess.run([sys.executable, '-m', 'dougie_sim.radiation_run', '--world', 'mezzanine',
        '--headless', '--fast', '--duration', '1', '--fixed-plan-delay', '.5',
        '--require-round-trip', '--output', str(output)], capture_output=True, text=True, timeout=45)
    assert run.returncode == 1, run.stdout+run.stderr
    config = json.loads((output/'run_config.json').read_text())
    summary = json.loads((output/'radiation_summary.json').read_text())
    assert config['require_clean'] and config['require_round_trip']
    assert not summary['round_trip_completed']
    assert 'missing_ordered_vertical_witness' in summary['round_trip_failures']


def test_landmark_mission_archives_observations_separately_from_truth(tmp_path):
    output = tmp_path/'landmarks'
    run = subprocess.run([sys.executable, '-m', 'dougie_sim.radiation_run', '--world', 'facility',
        '--headless', '--fast', '--duration', '1', '--fixed-plan-delay', '.5',
        '--landmark-count', '8', '--landmark-seed', '7', '--output', str(output)],
        capture_output=True, text=True, timeout=45)
    assert run.returncode == 0, run.stdout+run.stderr
    summary = json.loads((output/'radiation_summary.json').read_text())
    truth = json.loads((output/'landmark_evaluation.json').read_text())
    observed = json.loads((output/'landmark_observations.json').read_text())
    assert truth['total'] == 8 and truth['per_floor']['upper']['total'] == 4
    assert len(observed) == truth['discovered'] < truth['total']
    assert {o['identifier'] for o in observed} == {t['identifier'] for t in truth['targets'] if t['discovered']}
    assert not summary['landmark_mission_completed']
    assert 'visible landmark observations only' in summary['information_contract']
