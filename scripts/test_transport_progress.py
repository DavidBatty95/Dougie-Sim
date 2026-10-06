import json

from summarize_transport_progress import summarize


def test_attempt_without_result_is_retained_and_marked_incomplete(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps(dict(world='lab',people=0,seed=42)))
    row = summarize(tmp_path)
    assert row['incomplete_artifacts']
    assert row['configuration']['seed'] == 42
    assert 'outcome' not in row


def test_verified_geometry_does_not_promote_failed_mission_to_success(tmp_path):
    (tmp_path/'manifest.json').write_text('{}')
    (tmp_path/'result.json').write_text(json.dumps(dict(success=False,mission_complete=False,
        error='post-release evidence unavailable',deliveries=[dict(verified=True)])))
    row = summarize(tmp_path)
    assert not row['incomplete_artifacts']
    assert not row['outcome']['success']
    assert row['outcome']['deliveries'][0]['verified']
