from analyze_avoidance_dataset import cohort_summary


def test_cohort_summary_keeps_validator_failures_and_invalid_contact_reports():
    records = [dict(role='scored',result=dict(valid_trial=valid,success=success,
                   goals_requested=3,goals_reached=goals,collisions=contacts),
                   execution=dict(return_code=code))
               for valid,success,goals,contacts,code in
               ((True,True,3,0,0),(True,True,3,0,1),(False,False,0,2,1))]
    assert cohort_summary({'records':records}) == dict(attempted=3,passed=1,invalid=1,
        goals_reached=6,goals_requested=9,reported_contacts=2)
