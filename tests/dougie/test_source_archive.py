import hashlib
import io
import tarfile

import pytest

from scripts.report_dougie_terrain import source_snapshot


def test_existing_source_archive_is_verified_without_requiring_current_source(tmp_path):
    name = 'dougie_sim/historical_module_no_longer_present.py'
    content = b'# exact historical source\n'
    path = tmp_path/'source_snapshot.tar.gz'
    with tarfile.open(path, 'w:gz') as archive:
        member = tarfile.TarInfo(name)
        member.size = len(content)
        archive.addfile(member, io.BytesIO(content))
    manifest = dict(source_sha256={name: hashlib.sha256(content).hexdigest()})
    before = path.read_bytes()
    assert source_snapshot(tmp_path, manifest)['archive_reused']
    assert path.read_bytes() == before
    manifest['source_sha256'][name] = '0'*64
    with pytest.raises(ValueError, match='hash mismatch'):
        source_snapshot(tmp_path, manifest)
