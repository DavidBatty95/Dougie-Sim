from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.policy_timing import inference_timing,result_is_stale


def fixture():
    return (SimpleNamespace(generation=2,sequence=7,submitted_at=10.,completed_at=10.004),
            dict(key=(2,7),command=[.3,0.,0.],command_received_ros_s=1000.,
                 lowstate_receipt_age_at_submit_s=.002))


def test_timing_uses_monotonic_durations_and_explicit_ros_receipt_age():
    result,context = fixture()
    event = inference_timing(result,context,10.020,1000.035)
    assert event['submit_to_complete_ms'] == pytest.approx(4.)
    assert event['complete_to_publish_ms'] == pytest.approx(16.)
    assert event['submit_to_publish_ms'] == pytest.approx(20.)
    assert event['command_receipt_age_at_target_publish_s'] == pytest.approx(.035)
    assert event['lowstate_receipt_age_at_target_publish_s'] == pytest.approx(.022)
    assert event['role'] == 'policy_joint_target_publication_not_actuator_ack'


def test_old_generation_never_inherits_a_new_commands_context():
    result,context = fixture()
    context['key'] = (3,7)
    event = inference_timing(result,context,10.020,1000.035)
    assert not event['context_matched']
    assert 'command_at_submit' not in event
    assert 'command_receipt_age_at_target_publish_s' not in event


def test_clock_regression_is_not_clamped_into_a_fast_measurement():
    result,context = fixture()
    event = inference_timing(result,context,9.,999.)
    assert not event['timing_valid']
    assert 'submit_to_publish_ms' not in event
    assert event['command_receipt_age_at_target_publish_s'] is None


def test_completed_inference_cannot_be_applied_after_a_delayed_control_tick():
    result,context = fixture()
    assert not result_is_stale(result,10.02,.05)
    assert result_is_stale(result,10.1,.05)
    assert result_is_stale(result,9.,.05)
    assert result_is_stale(result,float('nan'),.05)
