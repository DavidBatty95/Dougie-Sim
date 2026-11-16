import json

from scripts.report_campaign_holdout import inspect


def test_holdout_report_keeps_censoring_physical_witness_and_actual_probe_arrival(tmp_path):
    config=dict(world='facility',layout_seed=2,sensor_seed=0,seed=212,duration=0.,
        campaign_sorties=2,lifetime_dose=2.,survival_fraction=.8,localization='ground-truth',campaign_strategy='agenda')
    (tmp_path/'comparison_config.json').write_text(json.dumps(config))
    state=dict(total_dose_usv=.2,total_time_s=20.,total_distance_m=3.,robots_lost=0,robots_recovered=1,
        sorties=1,explored_percent=52.,last_outcome='recovered_for_swap',wrecks=[])
    record=dict(id=1,directory='.',campaign_id=1,status='ended',exit_codes=dict(blind=-9,aware=0),learning_protocol_sha256='protocol')
    campaign=dict(id=1,status='halted',reason='second robot stranded',max_sorties=2,
        history=[dict(run_id=1)],modes=dict(blind=state,aware=state))
    (tmp_path/'comparison_session.json').write_text(json.dumps(dict(campaign=campaign,runs=[record])))
    for mode in ('blind','aware'):
        directory=tmp_path/mode
        directory.mkdir()
        telemetry=[dict(time_s=1.,campaign_time_s=1.,campaign_dose_usv=.01,explored_percent=25.),
            dict(time_s=20.,campaign_time_s=20.,campaign_dose_usv=.2,explored_percent=52.)]
        (directory/'comparison.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in telemetry))
        trace=[dict(time_s=1.,decision_reason='uncertainty_measurement_probe',selected_target=[1.,0.,0.],uncertainty_probe_count=1)]
        (directory/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trace))
        (directory/'measurements.jsonl').write_text(json.dumps(dict(stamp_s=2.,position=[1.,0.,.44],position_valid=True))+'\n')
        (directory/'radiation_summary.json').write_text(json.dumps(dict(end_state='returned',error=None,frontier_complete=False,
            frontier_count=8,physical_witness=dict(upper_landing_witness_s=10.),comparison_survey=dict(physically_reached_heights_m=[0.,2.4]))))
    report=inspect(tmp_path)
    assert report['censored'] and report['status']=='halted' and report['completed_sorties']==1
    aware=report['modes']['aware']
    assert aware['sortie_outcomes'][0]['frontier_complete'] is False
    assert aware['completion_censored']
    assert aware['sortie_outcomes'][0]['upper_landing_witness_s']==10.
    assert aware['sortie_outcomes'][0]['probes'][0]['reached_25cm']
    assert aware['milestones'][-1]['threshold']==50.
    assert aware['milestones'][-1]['observed_percent']==52.
    from scripts.monitor_campaign_holdout import render
    html=render(dict(updated_utc='test',studies=[report])).decode()
    assert 'exit -9' in html and 'halted' in html


def test_probe_arrival_after_abandonment_is_not_credited(tmp_path):
    (tmp_path/'comparison_config.json').write_text(json.dumps(dict(world='facility')))
    state=dict(total_dose_usv=0.,total_time_s=0.,total_distance_m=0.,robots_lost=0,robots_recovered=0,
        sorties=0,explored_percent=0.,last_outcome='not deployed',wrecks=[])
    record=dict(id=1,directory='.',campaign_id=1,status='running',exit_codes=dict(blind=None,aware=None))
    (tmp_path/'comparison_session.json').write_text(json.dumps(dict(campaign=dict(id=1,status='active',reason='',
        max_sorties=1,history=[],modes=dict(blind=state,aware=state)),runs=[record])))
    for mode in ('blind','aware'):
        directory=tmp_path/mode
        directory.mkdir()
        trace=[dict(time_s=1.,state='exploring',decision_reason='uncertainty_measurement_probe',selected_target=[1.,0.,0.]),
            dict(time_s=3.,state='returning',decision_reason='unresolved_frontiers',selected_target=[0.,0.,0.])]
        (directory/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trace))
        (directory/'measurements.jsonl').write_text(json.dumps(dict(stamp_s=5.,position=[1.,0.,.44],position_valid=True))+'\n')
    probe=inspect(tmp_path)['modes']['aware']['sortie_outcomes'][0]['probes'][0]
    assert not probe['reached_25cm'] and probe['minimum_distance_m'] is None


def test_live_worker_status_parsing_detects_signal_exit_without_retaining_telemetry(monkeypatch):
    import io
    from scripts import monitor_campaign_holdout
    # Deliberately put one JSON field across a 64 KiB read boundary.
    payload=json.dumps(dict(blind=dict(rows=[dict(state='x'*65515)],exit_code=-9,summary=None),
        aware=dict(rows=[dict(state='exploring')],exit_code=None,summary=None))).encode()
    monkeypatch.setattr(monitor_campaign_holdout,'urlopen',lambda *args,**kwargs:io.BytesIO(payload))
    assert monitor_campaign_holdout.worker_exit_codes('http://example.invalid')==dict(blind=-9,aware=None)
