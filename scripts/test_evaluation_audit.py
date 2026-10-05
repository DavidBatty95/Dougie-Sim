import json

import pytest

from audit_evaluation_alignment import audit, paired_clearance


def test_paired_clearance_uses_a_common_time_not_latest_independent_poses():
    states = [dict(sim_time=t,stamp=100.+t-1.,epoch=0,robot=dict(x=x,y=0.),
                   people=[dict(id=1,x=person_x,y=0.,radius_m=.42)])
              for t,x,person_x in ((1.,.27,1.045),(1.1,.23,1.015))]
    clearance,covered,_ = paired_clearance(states,1.05,1.1,.35)
    assert covered
    assert clearance == pytest.approx(.01)
    assert 1.015-.25-.35-.42 < 0.  # The unaligned sample would report contact.


def test_audit_reports_timebase_speed_difference_without_reclassifying_results(tmp_path):
    rows = [dict(topic='evaluation_aligned',stamp=wall+.01,data=dict(
        sim_time=sim,stamp=wall,epoch=0,robot=dict(x=x,y=0.,vx=.6,vy=0.),people=[],dropped_unaligned_samples=0))
        for sim,wall,x in ((1.,100.,0.),(1.1,100.05,.06))]
    rows.append(dict(topic='benchmark_metrics',stamp=100.06,data=dict(collisions=0,
        scoring_sample=dict(velocity_interval=[100.,100.05],robot_velocity=[1.2,0.]))))
    path = tmp_path/'trace.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result = audit(path)
    assert result['role'] == 'evaluation_diagnostic_only'
    example, = result['wall_time_only_speed_exceedances']
    assert example['legacy_speed'] == 1.2
    assert example['simulation_time_speed'] == pytest.approx(.6)
    assert result['collision_reports'] == []
