import pytest

from scripts.check_dougie_stop_screen import audit


def report():
    row = dict(angle_deg=8.53, mode='range_position_hold', sensor_seed=0, direction='down',
        max_horizontal_drift_m=.4, nonfoot_impulse_ns=0., nonfoot_peak_force_n=0., peak_motor_torque_nm=20.,
        scene_sha256='fixture', evaluator_approach_distance_m=.2, error=None, hold_entry_failed=False,
        hold_started_s=11.7, range_inhibited_ticks=100, max_inhibited_requested_command_m_s=0.,
        trace=[dict(time_s=31., range_feedback_ready=False, command=[0., 0., 0.])])
    return dict(cases=[[8.53, 'range_position_hold', 0, 'down']], results=[row], lidar_dropout_after_s=30.)


def test_stop_audit_refuses_missing_declared_cases_and_preserves_failures():
    data = report()
    data['cases'].append([8.53, 'range_position_hold', 1, 'down'])
    with pytest.raises(ValueError, match='predeclared'):
        audit(data)
    data = report()
    data['results'][0]['nonfoot_impulse_ns'] = 1.
    assert audit(data)[0]['contact_or_error_failures'] == 1
    with pytest.raises(ValueError, match='contact/error'):
        audit(data, require_contact_clean=True)


def test_stop_audit_checks_actual_fault_trace_not_only_summary_counter():
    data = report()
    assert audit(data, require_range_withheld=True)[0]['cases'] == 1
    data['results'][0]['trace'][0]['command'][0] = .01
    with pytest.raises(ValueError, match='post-dropout'):
        audit(data, require_range_withheld=True)
