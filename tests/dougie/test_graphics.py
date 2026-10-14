import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dougie_sim import graphics
from scripts.benchmark_dougie_rviz import frame_statistics


def test_headless_auto_does_not_probe_or_require_a_gpu(monkeypatch):
    def forbidden():
        raise AssertionError('headless execution must not probe graphics')
    monkeypatch.setattr(graphics, 'nvidia_available', forbidden)
    assert graphics.select_graphics('auto', needed=False) == 'system'


def test_interactive_auto_selects_nvidia_and_its_overlay(monkeypatch):
    monkeypatch.setattr(graphics, 'nvidia_available', lambda: True)
    mode = graphics.select_graphics('auto', needed=True)
    overlays, flags = graphics.compose_graphics('/workspace', mode)
    assert mode == 'nvidia'
    assert overlays == ['-f', '/workspace/docker-compose.gpu.yml'] and flags == []


def test_software_override_is_explicit_even_on_nvidia(monkeypatch):
    monkeypatch.setattr(graphics, 'nvidia_available', lambda: True)
    assert graphics.select_graphics('software', needed=True) == 'software'
    overlays, flags = graphics.compose_graphics('/workspace', 'software')
    assert overlays == [] and 'LIBGL_ALWAYS_SOFTWARE=1' in flags
    assert '__GLX_VENDOR_LIBRARY_NAME=mesa' in flags


def test_nvidia_selection_requires_both_device_and_docker_runtime(monkeypatch):
    monkeypatch.setattr(graphics.shutil, 'which', lambda name: '/usr/bin/'+name)
    replies = iter(['RTX test\n', json.dumps({'runc': {}})])
    monkeypatch.setattr(graphics.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=next(replies)))
    assert not graphics.nvidia_available()
    replies = iter(['RTX test\n', json.dumps({'nvidia': {}})])
    assert graphics.nvidia_available()


def test_dri_uses_compose_devices_and_numeric_permission_groups(monkeypatch, tmp_path):
    devices = tmp_path/'dri'
    devices.mkdir()
    (devices/'card0').touch()
    (devices/'renderD128').touch()
    monkeypatch.setenv('DOUGIE_DRI_RENDER_GID', 'test')
    monkeypatch.setenv('DOUGIE_DRI_CARD_GID', 'test')
    monkeypatch.setattr(graphics, 'Path', lambda p: devices if str(p) == '/dev/dri' else Path(p))
    overlays, flags = graphics.compose_graphics('/workspace', 'dri')
    assert overlays == ['-f', '/workspace/docker-compose.dri.yml'] and flags == []
    assert graphics.os.environ['DOUGIE_DRI_RENDER_GID'] == str((devices/'renderD128').stat().st_gid)


def test_frame_counter_selects_viewport_and_excludes_warmup(tmp_path):
    path = tmp_path/'frames.csv'
    # The 1x1 helper swaps much faster; only the actual large viewport counts.
    rows = [(t, 1, 1, 1) for t in np.arange(0., 3., .005)]
    rows += [(t, 2, 1000, 600) for t in np.arange(0., 3., .05)]
    np.savetxt(path, rows, delimiter=',')
    result = frame_statistics(path, 1., 3.)
    assert result['drawable'] == 2 and result['viewport_pixels'] == 600000
    assert result['presented_fps'] == pytest.approx(20.)
    assert result['frame_interval_p50_ms'] == pytest.approx(50.)
