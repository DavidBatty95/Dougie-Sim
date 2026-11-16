from functools import partial
from http.server import ThreadingHTTPServer
import json
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from dougie_sim.comparison import ComparisonSession, DashboardHandler
from dougie_sim.comparison_history import session_averages


class Worker:
    def __init__(self, *args, **kwargs):
        self.returncode = None

    def poll(self):
        return self.returncode

    def send_signal(self, signal):
        self.returncode = 0

    def wait(self, timeout=None):
        return self.returncode


def settings(**kwargs):
    return SimpleNamespace(**dict(dict(world='facility', duration=0., seed=7,
        sensor_seed=2, layout_seed=3, dose_budget=None, rate_limit=100., feed_fps=5,
        localization='ground-truth', radiation_policy='minimize-dose', exposure_policy='responsive',
        gp_mode='windowed', gp_expert_capacity=32, gp_max_experts=16, replay=None,
        continuous=False), **kwargs))


def telemetry(session, time=10.):
    for i, mode in enumerate(('blind', 'aware')):
        directory = session.output/mode
        directory.mkdir(exist_ok=True)
        rows = [dict(time_s=0., true_dose_usv=0., distance_m=0., state='exploring'),
                dict(time_s=time, true_dose_usv=2.+i, distance_m=10.+i,
                     state='interrupted', frontier_complete=False)]
        (directory/'comparison.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))


def test_rerun_preserves_interrupted_pair_and_advances_only_detector_seed(tmp_path, monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen', Worker)
    session = ComparisonSession(settings(), tmp_path)
    telemetry(session)
    old_processes = dict(session.processes)
    session.request_rerun()
    with pytest.raises(RuntimeError):
        session.request_rerun()
    session.tick()
    assert session.output == tmp_path/'reruns/run_00002'
    assert all(p.poll() == 0 for p in old_processes.values())
    assert session.records[0]['status'] == 'rerun_interrupted'
    assert not session.records[0]['outcomes']['aware']['frontier_complete']
    assert json.loads((tmp_path/'comparison_status.json').read_text()) == dict(blind=0, aware=0)
    config = json.loads((session.output/'comparison_config.json').read_text())
    assert (config['seed'],config['sensor_seed'],config['layout_seed']) == (8,2,3)
    assert session.averages['ended_pairs'] == 1
    assert session.averages['curves'][-1]['aware']['distance_m']['mean'] == 11.
    assert session.state()['running'] and not session.state()['rerun_pending']
    session.close()


def test_continuous_waits_for_both_workers_and_can_be_disabled(tmp_path, monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen', Worker)
    clock = [100.]
    monkeypatch.setattr('dougie_sim.comparison.time.monotonic', lambda: clock[0])
    session = ComparisonSession(settings(), tmp_path)
    telemetry(session)
    session.set_continuous(True)
    session.processes['blind'].returncode = 0
    session.tick()
    assert len(session.records) == 1 and not session.samples
    session.processes['aware'].returncode = 1  # Failed attempts remain included.
    session.tick()
    assert len(session.samples) == 1
    clock[0] += 4.
    session.tick()
    assert len(session.records) == 2 and session.args.seed == 8
    session.set_continuous(False)
    for p in session.processes.values():
        p.returncode = 0
    session.tick()
    clock[0] += 4.
    session.tick()
    assert len(session.records) == 2
    assert session.records[0]['exit_codes']['aware'] == 1
    assert session.averages['ended_pairs'] == 2
    assert session.averages['paired_observations'] == 1
    session.close()


def test_replay_restores_history_and_reruns_with_recorded_settings(tmp_path, monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen', Worker)
    original = ComparisonSession(settings(), tmp_path)
    telemetry(original)
    original.request_rerun()
    original.tick()
    telemetry(original, 5.)
    original.close()
    replay = ComparisonSession(settings(replay=tmp_path, world='reactor', seed=100), tmp_path)
    assert replay.averages['ended_pairs'] == 2
    assert replay.output == tmp_path/'reruns/run_00002'
    assert replay.args.world == 'facility' and replay.args.seed == 8
    replay.request_rerun()
    replay.tick()
    assert replay.records[-1]['seed'] == 9
    assert replay.output == tmp_path/'reruns/run_00003'
    replay.close()


def test_partial_launch_failure_stops_already_started_worker(tmp_path, monkeypatch):
    workers = []
    def launch(*args, **kwargs):
        if workers:
            raise OSError('launch failed')
        worker = Worker()
        workers.append(worker)
        return worker
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen', launch)
    with pytest.raises(OSError):
        ComparisonSession(settings(), tmp_path)
    assert workers[0].poll() == 0
    record = json.loads((tmp_path/'comparison_session.json').read_text())['runs'][0]
    assert record['status'] == 'launch_failed'
    assert record['exit_codes'] == dict(blind=0, aware=1)


def test_control_api_queues_only_one_rerun_and_serves_history(tmp_path, monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen', Worker)
    session = ComparisonSession(settings(), tmp_path)
    server = ThreadingHTTPServer(('127.0.0.1',0),partial(DashboardHandler,session=session))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    def post(path, data):
        return urlopen(Request(base+path, data=json.dumps(data).encode(),headers={'Content-Type':'application/json'}))
    try:
        with post('/continuous',dict(enabled=True)) as response:
            assert json.load(response)['continuous']
        with post('/learning',dict(enabled=True)) as response:
            control_state=json.load(response)
            assert control_state['learning_enabled'] and control_state['continuous']
        with post('/rerun',{}) as response:
            assert response.status == 202 and json.load(response)['rerun_pending']
        with pytest.raises(HTTPError) as error:
            post('/rerun',{})
        assert error.value.code == 409
        with pytest.raises(HTTPError) as error:
            post('/continuous',dict(enabled='false'))
        assert error.value.code == 400
        with urlopen(base+'/api') as response:
            assert json.load(response)['_session']['seed'] == 7
        with urlopen(base+'/history') as response:
            history=json.load(response)
            assert history['averages']['ended_pairs'] == 0
            assert history['learning']['enabled']
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        session.close()


def test_means_are_time_aligned_paired_and_never_pad_short_attempts():
    def rows(end, dose, distance):
        return [dict(time_s=0.,true_dose_usv=0.,distance_m=0.),
                dict(time_s=end,true_dose_usv=dose,distance_m=distance)]
    pairs = [dict(blind=rows(10.,10.,20.),aware=rows(5.,10.,30.)),
             dict(blind=rows(10.,30.,60.),aware=rows(10.,40.,80.)),
             dict(blind=rows(10.,100.,100.),aware=[])]
    report = session_averages(pairs)
    middle = next(r for r in report['curves'] if r['time_s']==5.)
    assert middle['paired_n'] == 2
    assert middle['blind']['true_dose_usv']['mean'] == 10.
    assert middle['aware']['distance_m']['mean'] == 35.
    assert middle['blind']['true_dose_usv']['sd'] == pytest.approx(2**.5*5.)
    last = report['curves'][-1]
    assert last['time_s'] == 10. and last['paired_n'] == 1
    assert last['blind']['true_dose_usv']['mean'] == 30.
    assert last['aware']['distance_m']['mean'] == 80.
    assert report['ended_pairs'] == 3 and report['paired_observations'] == 2


def test_missing_intervals_are_explicit_gaps_in_mean_curves():
    def pair(start,end):
        rows = [dict(time_s=t,true_dose_usv=t,distance_m=t) for t in (start,end)]
        return dict(blind=rows,aware=rows)
    report = session_averages([pair(0.,2.),pair(8.,10.)])
    gap = next(r for r in report['curves'] if r['time_s']==5.)
    assert gap['paired_n'] == 0 and gap['blind']['distance_m']['mean'] is None


def learning_telemetry(session):
    trial=session.records[-1]['learning_trial']
    for mode in ('blind','aware'):
        directory=session.output/mode
        directory.mkdir(exist_ok=True)
        dose=20. if mode=='blind' else 12. if trial['role']=='challenger' else 16.
        rows=[dict(time_s=0.,true_dose_usv=0.,distance_m=0.,explored_percent=0.,state='exploring'),
            dict(time_s=50.,true_dose_usv=8.,distance_m=50.,explored_percent=75.1,state='exploring'),
            dict(time_s=100.,true_dose_usv=dose,distance_m=100.,explored_percent=89.4,state='returned',frontier_complete=False)]
        (directory/'comparison.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (directory/'radiation_summary.json').write_text(json.dumps(dict(clean_return=True,error=None)))
    for worker in session.processes.values():
        worker.returncode=0


def test_session_applies_trials_to_aware_only_and_promotes_after_full_panel(tmp_path,monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen',Worker)
    clock=[100.]
    monkeypatch.setattr('dougie_sim.comparison.time.monotonic',lambda:clock[0])
    session=ComparisonSession(settings(learn_planning=True,learning_seeds=2),tmp_path)
    assert session.continuous and session.state()['learning_enabled']
    seeds=[]
    for index in range(4):
        seeds.append(session.args.seed)
        config=json.loads((session.output/'comparison_config.json').read_text())
        assert config['planning_weights']['blind']==dict(route_dose_weight=1.,frontier_dose_weight=2.)
        assert config['planning_weights']['aware']['route_dose_weight']==(1. if index%2==0 else 2.)
        learning_telemetry(session)
        session.tick()
        if index<3:
            clock[0]+=4.
            session.tick()
    assert seeds==[7,7,8,8]
    assert session.learner.rounds[0]['promoted']
    assert session.learner.best['route_dose_weight']==2.
    assert len(session.history()['learning']['entries'])==4
    session.close()


def test_session_replay_resumes_same_learning_panel_without_losing_history(tmp_path,monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen',Worker)
    original=ComparisonSession(settings(learn_planning=True),tmp_path)
    learning_telemetry(original)
    original.tick()
    original.close()
    replay=ComparisonSession(settings(replay=tmp_path,learn_planning=True),tmp_path)
    assert replay.learner.slot==1
    assert len(replay.learner.entries)==1
    replay.request_rerun()
    replay.tick()
    assert replay.args.seed==7
    assert replay.records[-1]['learning_trial']['role']=='challenger'
    assert replay.records[-1]['planning_weights']['aware']['route_dose_weight']==2.
    replay.close()


def campaign_telemetry(session):
    for mode in ('blind','aware'):
        directory=session.output/mode
        directory.mkdir(exist_ok=True)
        for name in ('campaign_memory.npz','gp_posterior.npz','coverage_reference.npz','minimap_final.npz'):
            (directory/name).write_bytes(b'archive fixture')
        row=dict(time_s=2.,true_dose_usv=.1,distance_m=1.,explored_percent=40.,state='returned',frontier_complete=False,
            campaign_id=session.campaign.id,campaign_time_s=2.,campaign_dose_usv=.1,campaign_distance_m=1.)
        (directory/'comparison.jsonl').write_text(json.dumps(row)+'\n')
        (directory/'radiation_summary.json').write_text(json.dumps(dict(robot_lifetime=dict(alive=True),
            true_dose_usv=.1,distance_3d_m=1.,simulation=dict(simulation_time_s=2.),
            comparison_survey=dict(explored_percent=40.),end_state='returned',error=None,frontier_complete=False,
            localization_evaluation=dict(true_home_distance_m=.1))))
    for worker in session.processes.values():
        worker.returncode=0


def test_campaign_supervisor_retains_memory_and_blocks_in_field_reset(tmp_path,monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen',Worker)
    session=ComparisonSession(settings(campaign=True,lifetime_dose=2.,campaign_sorties=3),tmp_path)
    with pytest.raises(RuntimeError,match='in the field'):
        session.request_rerun()
    with pytest.raises(ValueError,match='worlds change'):
        session.set_learning(True)
    campaign_telemetry(session)
    session.tick()
    session.request_rerun()
    session.tick()
    inputs=json.loads((session.output/'aware_campaign_input.json').read_text())
    assert inputs['robot_id']==2 and inputs['prior_dose_usv']==.1
    assert inputs['previous_directory']==str((tmp_path/'aware').resolve())
    session.new_campaign(3.,2)
    session.tick()
    assert session.campaign.id==2 and session.campaign.limit==3.
    assert session.campaign.modes['aware']['total_dose_usv']==0.
    assert session.campaign_archives[0]['status']=='abandoned'
    assert json.loads((session.output/'aware_campaign_input.json').read_text())['previous_directory'] is None
    session.close()


def test_campaign_replay_preserves_ledger_without_duplicate_dose(tmp_path,monkeypatch):
    monkeypatch.setattr('dougie_sim.comparison.subprocess.Popen',Worker)
    original=ComparisonSession(settings(campaign=True,lifetime_dose=2.,campaign_sorties=3),tmp_path)
    campaign_telemetry(original)
    original.tick()
    original.close()
    restored=ComparisonSession(settings(replay=tmp_path,campaign=True,lifetime_dose=2.),tmp_path)
    assert restored.campaign.modes['aware']['total_dose_usv']==.1
    assert len(restored.campaign.history)==1
    restored.request_rerun()
    restored.tick()
    assert json.loads((restored.output/'aware_campaign_input.json').read_text())['robot_id']==2
    restored.close()
