import pytest

from unitree_g1_mujoco_bridge.transport_health import EvaluationHealth


def fixture():
    now = [0.]
    health = EvaluationHealth(['robot','people','box'],clock=lambda:now[0])
    for key in health.required:
        health.observe(key,0.)
    health.start()
    return health,now


def test_startup_wait_does_not_count_as_mission_outage():
    health = EvaluationHealth(['robot'])
    health.check()
    assert not health.report()['valid']
    assert not health.failures
    health.observe('robot',1.)
    health.start()
    assert health.report()['valid']


def test_missing_stream_at_mission_start_is_latched():
    health = EvaluationHealth(['people'])
    health.start()
    health.observe('people',1.)
    assert not health.report()['valid']


def test_outage_remains_invalid_after_recovery_even_without_timer_callback():
    health,now = fixture()
    now[0] = 2.
    for key in health.required:
        health.observe(key,2.)
    assert not health.report()['valid']
    assert 'people: receipt gap exceeded' in health.failures


def test_final_people_stream_must_be_fresh():
    health,now = fixture()
    now[0] = 1.1
    health.observe('robot',.5)
    health.observe('box',.5)
    assert 'people: receipt gap exceeded' in health.report()['failures']


def test_replayed_acquisition_stamps_do_not_refresh_coverage():
    health,now = fixture()
    now[0] = .8
    assert not health.observe('people',0.)
    now[0] = 1.1
    assert not health.report()['valid']


@pytest.mark.parametrize('stamp,reason',[
    (-1.,'acquisition clock regressed'),(2.,'acquisition gap exceeded'),
    (float('nan'),'nonfinite acquisition stamp'),
])
def test_bad_acquisition_history_is_latched(stamp,reason):
    health,now = fixture()
    now[0] = .1
    health.observe('people',stamp)
    assert 'people: '+reason in health.report()['failures']


def test_continuous_streams_remain_valid():
    health,now = fixture()
    for i in range(1,100):
        now[0] = i*.1
        for key in health.required:
            health.observe(key,now[0])
        health.check()
    assert health.report()['valid']
