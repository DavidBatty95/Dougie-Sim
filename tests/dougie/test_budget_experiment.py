import json

from scripts.benchmark_dougie_budget import score, paired_report


def test_physical_visitation_cannot_be_inflated_by_estimator_drift(tmp_path):
    (tmp_path/'radiation_summary.json').write_text(json.dumps(dict(
        simulation=dict(simulation_time_s=120.), true_dose_usv=.5, clean_return=False,
        path_length_m=0., decision_reason='waiting_for_support')))
    (tmp_path/'run_config.json').write_text(json.dumps(dict(dose_budget=10., duration=600., seed=3, sensor_seed=0)))
    (tmp_path/'localization_evaluation.jsonl').write_text('\n'.join(json.dumps(dict(
        stamp_s=float(t), position=[float(t), 0., .3], evaluator_true_position=[0., 0., .3])) for t in range(120)))
    result = score(tmp_path, 1)
    assert result['physical_visited_xy_cells'] == 1
    assert result['visitation_cells_per_usv'] == 2.
    assert result['visitation_cells_per_minute'] == .5
    assert not result['clean_return']
    assert result['returncode'] == 1


def test_missing_summary_is_retained_and_not_treated_as_success(tmp_path):
    result = score(tmp_path, 124)
    assert result['returncode'] == 124 and not result['clean_return']
    assert paired_report([result]) == []
