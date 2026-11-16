from copy import deepcopy
import json

import numpy as np
import pytest

from dougie_sim.planning_learning import DEFAULT_WEIGHTS,PlanningLearner,evaluate_attempt,planning_weights


def observations(total=16.,milestone=8.,progress=89.4,complete=False):
    def rows(dose,crossing,end_progress,finished=False):
        return [dict(time_s=0.,true_dose_usv=0.,distance_m=0.,explored_percent=0.,state='exploring'),
            dict(time_s=50.,true_dose_usv=crossing,distance_m=50.,explored_percent=75.1,state='exploring'),
            dict(time_s=100.,true_dose_usv=dose,distance_m=100.,explored_percent=end_progress,
                 state='finished' if finished else 'returned',frontier_complete=finished)]
    return dict(blind=rows(20.,10.,89.4),aware=rows(total,milestone,progress,complete))


def attempt(trial=None,number=1):
    return dict(id=number,seed=trial['seed'] if trial else 1,status='ended',exit_codes=dict(blind=0,aware=0),
        outcomes={m:dict(clean_return=True,error=None) for m in ('blind','aware')},learning_trial=trial,
        planning_weights=dict(blind=DEFAULT_WEIGHTS,aware=trial['weights'] if trial else DEFAULT_WEIGHTS),
        learning_protocol_sha256='unchanged-runtime-and-protocol')


def teach(learner,total=16.,milestone=8.,progress=89.4,**changes):
    record=attempt(learner.propose(),len(learner.entries)+1)
    record.update(changes)
    result=evaluate_attempt(record,observations(total,milestone,progress),learner.milestone)
    learner.observe(record,result)
    return record


def test_reward_uses_actual_total_dose_and_records_fixed_milestone_check():
    result=evaluate_attempt(attempt(),observations())
    assert result['eligible'] and result['reference_only']
    assert result['total_dose_ratio']==.8
    assert result['milestone_dose_ratio']==.8
    assert result['runs']['aware']['endpoint']['true_dose_usv']==16.


@pytest.mark.parametrize('mutation',[
    lambda r,p:r.update(status='rerun_interrupted'),
    lambda r,p:r['exit_codes'].update(aware=1),
    lambda r,p:r['outcomes']['aware'].update(clean_return=False),
    lambda r,p:r['outcomes']['aware'].update(error='worker error'),
    lambda r,p:r['outcomes']['aware'].update(artifact_error='missing summary'),
    lambda r,p:p['aware'][-1].update(explored_percent=80.),
    lambda r,p:[row.update(explored_percent=74.) for row in p['aware']],
    lambda r,p:p['aware'][1].update(explored_percent=79.),
    lambda r,p:p['blind'][-1].update(frontier_complete=True),
    lambda r,p:p['aware'][-1].update(state='interrupted'),
    lambda r,p:p['aware'][-1].update(true_dose_usv=float('nan')),
    lambda r,p:p['aware'][-1].update(time_s=1.),
])
def test_less_work_failures_interruptions_and_bad_data_cannot_win(mutation):
    record,pair=attempt(),observations(total=1.,milestone=.5)
    mutation(record,pair)
    assert not evaluate_attempt(record,pair)['eligible']


def test_seed_panels_repeat_incumbent_and_challenger_before_advancing():
    learner=PlanningLearner(base_seed=41,enabled=True)
    trials=[]
    for i in range(6):
        trial=learner.propose()
        trials.append((trial['seed'],trial['role'],trial['weights']))
        teach(learner,total=12. if trial['role']=='challenger' else 16.)
    assert [s for s,_,_ in trials]==[41,41,42,42,43,43]
    assert [r for _,r,_ in trials]==['incumbent','challenger']*3
    assert learner.rounds[0]['promoted']
    assert learner.best==dict(route_dose_weight=2.,frontier_dose_weight=2.)
    assert learner.propose()['seed']==44


def test_lower_milestone_dose_cannot_hide_worse_total_mission_dose():
    learner=PlanningLearner(enabled=True)
    for _ in range(3):
        teach(learner,total=16.,milestone=8.)
        teach(learner,total=25.,milestone=2.)
    assert not learner.rounds[0]['promoted']
    assert learner.best==DEFAULT_WEIGHTS


def test_candidate_must_improve_each_seed_and_not_trade_away_progress():
    learner=PlanningLearner(enabled=True)
    for candidate in (10.,10.,17.):
        teach(learner)
        teach(learner,total=candidate)
    assert not learner.rounds[0]['promoted']
    other=PlanningLearner(enabled=True)
    for _ in range(3):
        teach(other,progress=90.5)
        teach(other,total=10.,progress=89.4)
    assert not other.rounds[0]['promoted']


def test_changed_runtime_rejects_cross_trial_comparison():
    learner=PlanningLearner(enabled=True)
    for _ in range(3):
        teach(learner)
        teach(learner,total=10.,learning_protocol_sha256='changed-runtime')
    assert not learner.rounds[0]['promoted']
    assert 'runtime' in learner.rounds[0]['reason']


def test_a_better_ratio_cannot_reward_more_actual_dose_or_a_changed_blind_reference():
    learner=PlanningLearner(enabled=True)
    for _ in range(3):
        teach(learner,total=16.)
        record=attempt(learner.propose(),len(learner.entries)+1)
        pair=observations(total=20.)
        pair['blind'][-1]['true_dose_usv']=40.  # Ratio looks better: .5 versus .8.
        result=evaluate_attempt(record,pair)
        assert result['eligible'] and result['total_dose_ratio']==.5
        learner.observe(record,result)
    assert not learner.rounds[0]['promoted']
    assert learner.best==DEFAULT_WEIGHTS
    assert 'blind reference changed' in learner.rounds[0]['reason']


def test_seed_or_parameter_mislabel_cannot_teach_the_tuner():
    learner=PlanningLearner(enabled=True)
    record=teach(learner,seed=999)
    assert not learner.entries[0]['result']['eligible']
    assert not record['learning_result']['eligible']


def test_learning_state_survives_restart_mid_panel_and_pauses_without_losing_weights():
    learner=PlanningLearner(base_seed=40,enabled=True)
    teach(learner)
    restored=PlanningLearner.load(json.loads(json.dumps(learner.save())),enabled=True)
    assert restored.propose()==learner.propose()
    assert restored.propose()['role']=='challenger' and restored.propose()['seed']==40
    restored.set_enabled(False,next_seed=900)
    assert restored.propose() is None and restored.best==DEFAULT_WEIGHTS
    restored.set_enabled(True,next_seed=900)
    assert restored.propose()['seed']==40


def test_duplicate_results_do_not_advance_learning_twice():
    learner=PlanningLearner(enabled=True)
    record=teach(learner)
    learner.observe(record,record['learning_result'])
    assert learner.slot==1 and len(learner.entries)==1


def test_search_remains_bounded_across_promotions_and_rejections():
    learner=PlanningLearner(enabled=True)
    for _ in range(20):
        for _ in range(3):
            teach(learner,total=16.)
            teach(learner,total=10.)
        assert all(.25<=v<=16. for v in learner.best.values())
        assert all(.25<=v<=16. for v in learner.candidate.values())


def test_route_weight_changes_dose_time_tradeoff_without_altering_the_field():
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    value=ClusteredDoseExplorer()
    value.budget_enabled=value.rate_limited=False
    value.observed_rates.append(1.)
    terrain=value.terrain
    terrain.nodes={0:np.array([0.,0.,0.]),1:np.array([1.,0.,0.]),2:np.array([1.,1.,0.]),3:np.array([2.,0.,0.])}
    terrain.graph={0:[1,2],1:[0,3],2:[0,3],3:[1,2]}
    rates={0:1.,1:2.,2:1.,3:1.}
    value.route_dose_weight=.25
    short=terrain.routes(0,rates,rate_scale=value.effective_route_rate_scale)[3]
    value.route_dose_weight=4.
    cool=terrain.routes(0,rates,rate_scale=value.effective_route_rate_scale)[3]
    assert terrain.path(short,3)==[0,1,3]
    assert terrain.path(cool,3)==[0,2,3]
    assert rates=={0:1.,1:2.,2:1.,3:1.}


def test_blind_commands_remain_fixed_when_aware_parameters_are_tuned(tmp_path):
    from types import SimpleNamespace
    from dougie_sim.comparison import mission_command
    args=SimpleNamespace(world='reactor',duration=0.,seed=1,sensor_seed=0,layout_seed=0,
        dose_budget=None,rate_limit=100.,feed_fps=5,route_dose_weight=8.,frontier_dose_weight=4.)
    blind,aware=(mission_command(args,m,tmp_path) for m in ('blind','aware'))
    for flag,expected in (('--route-dose-weight',('1.0','8.0')),('--frontier-dose-weight',('2.0','4.0'))):
        assert (blind[blind.index(flag)+1],aware[aware.index(flag)+1])==expected


@pytest.mark.parametrize('route,frontier',[(0.,2.),(float('inf'),2.),(1.,20.),(.1,2.)])
def test_unbounded_planning_weights_are_rejected(route,frontier):
    with pytest.raises(ValueError):
        planning_weights(route,frontier)
