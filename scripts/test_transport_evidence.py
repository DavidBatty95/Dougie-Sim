import json
import zipfile

from transport_evidence import capture_source,compare_python_sources,digest,inventory


def test_inventory_excludes_results_secrets_and_outside_symlinks(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    (scripts/'mission.py').write_text('pass\n')
    (scripts/'.env').write_text('secret')
    (scripts/'results').mkdir()
    (scripts/'results/result.json').write_text('{}')
    (scripts/'outside').symlink_to('/etc/hosts')
    assert inventory(tmp_path,['scripts']) == {'scripts/mission.py':digest(scripts/'mission.py')}


def test_snapshot_retains_dirty_source_and_hashes_policy_without_archiving_it(tmp_path,monkeypatch):
    import transport_evidence
    (tmp_path/'scripts').mkdir()
    source = tmp_path/'scripts/mission.py'
    source.write_text('uncommitted = True\n')
    policy = tmp_path/'candidate.onnx'
    policy.write_bytes(b'policy fixture')
    run = tmp_path/'run'
    run.mkdir()
    monkeypatch.setattr(transport_evidence,'command',lambda *args,**kw:dict(returncode=0,stdout='fixture'))
    result = capture_source(tmp_path,run,policy)
    with zipfile.ZipFile(run/'source_snapshot.zip') as archive:
        assert archive.read('scripts/mission.py') == source.read_bytes()
        assert 'candidate.onnx' not in archive.namelist()
    assert result['source_sha256']['candidate.onnx'] == digest(policy)
    assert json.loads((run/'source_provenance.json').read_text()) == result


def test_stale_missing_and_extra_installed_modules_are_detected():
    source = {'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/mission.py':'new',
              'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/health.py':'health'}
    packages = {'unitree_g1_mujoco_bridge':{'files':{'mission.py':'old','obsolete.py':'old'}},
                'dynamic_obstacle_benchmarks':{'files':{}}}
    report = compare_python_sources(source,packages)
    assert not report['matched']
    assert len(report['mismatches']) == 3
    packages['unitree_g1_mujoco_bridge']['files'] = {'mission.py':'new','health.py':'health'}
    assert compare_python_sources(source,packages)['matched']
