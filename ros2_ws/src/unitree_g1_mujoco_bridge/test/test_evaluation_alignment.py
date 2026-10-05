import pytest

from unitree_g1_mujoco_bridge.evaluation_alignment import EvaluationAlignment


def person(x):
    return dict(id=1,x=x,y=0.,vx=-.3,vy=0.,radius_m=.42)


def test_bracketed_scoring_does_not_mix_robot_and_person_epochs():
    alignment = EvaluationAlignment()
    assert alignment.add_people(0.,1,[person(1.045)]) == []
    assert alignment.add_root(dict(sim_time=.05,stamp=100.,robot=dict(x=.25,y=0.,vx=-.4,vy=0.))) == []
    result, = alignment.add_people(.1,1,[person(1.015)])
    assert result['people'][0]['x']-result['robot']['x'] == pytest.approx(.78)
    # Pairing the latest person with the older robot would falsely cross .77 m.
    assert 1.015-.25 < .77
    assert result['robot']['vx'] == -.4


def test_alignment_refuses_missing_brackets_large_gaps_and_population_changes():
    alignment = EvaluationAlignment()
    alignment.add_people(0.,1,[person(2.)])
    alignment.add_root(dict(sim_time=.05,robot={}))
    assert alignment.add_people(.5,1,[person(1.85)]) == []
    assert alignment.dropped == 1
    alignment.add_root(dict(sim_time=.55,robot={}))
    assert alignment.add_people(.6,0,[]) == []
    assert alignment.dropped == 2


def test_reset_does_not_interpolate_across_old_simulation_history():
    alignment = EvaluationAlignment()
    alignment.add_people(10.,1,[person(2.)])
    alignment.add_root(dict(sim_time=10.1,robot={}))
    assert alignment.add_root(dict(sim_time=.1,robot={})) == []
    assert alignment.add_people(0.,1,[person(3.)]) == []
    result, = alignment.add_people(.2,1,[person(2.94)])
    assert result['people'][0]['x'] == pytest.approx(2.97)
