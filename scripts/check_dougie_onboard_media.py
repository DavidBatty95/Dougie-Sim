#!/usr/bin/env python3
"""Decode every current GIF frame and verify portable provenance/window integrity."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT/'docs/media/dougie'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((MEDIA/'onboard-manifest.json').read_text())
    portable = MEDIA/'onboard-replay.json'
    assert digest(portable) == manifest['portable_sha256'], 'portable input checksum mismatch'
    assert digest(ROOT/'scripts/render_dougie_onboard.py') == manifest['renderer_sha256'], 'renderer identity mismatch'
    data = json.loads(portable.read_text())
    assert data['config']['localization'] == 'fused'
    assert data['config']['gp_mode'] == 'geometry'
    assert data['config']['detector_likelihood'] == 'window'
    assert data['summary']['clean_return'] is True
    assert data['summary']['localization_evaluation']['true_home_distance_m'] <= .4
    if data['config'].get('require_round_trip'):
        assert data['summary']['round_trip_completed'] is True
        assert not data['summary']['round_trip_failures']
        witness = data['summary']['physical_witness']
        assert witness['vertical_round_trip_witness']
        assert 0 < witness['upper_landing_witness_s'] < witness['lower_return_witness_s']
    for filename, key in (('onboard-mission.gif', 'native_recording'), ('onboard-fusion.gif', 'fusion_replay')):
        path, metadata = MEDIA/filename, manifest[key]
        assert digest(path) == metadata['sha256']
        assert path.stat().st_size == metadata['bytes']
        assert np.all(np.diff(metadata['frame_times_s']) > 0)
        frames = set()
        with Image.open(path) as image:
            assert image.n_frames == len(metadata['frame_times_s'])
            assert list(image.size) == metadata['size']
            assert image.info['loop'] == 0
            for index in range(image.n_frames):
                image.seek(index)
                assert image.info['duration'] == metadata['frame_duration_ms']
                frames.add(hashlib.sha256(image.convert('RGB').tobytes()).hexdigest())
        assert len(frames) > 10, 'animation does not contain enough distinct decoded frames'
        print(f'{filename}: {len(frames)} distinct decoded frames, checksum verified')
    for sample in data['measurements']:
        durations = np.asarray(sample['window_durations_s'])
        assert 1 <= len(durations) <= 8 and np.all(durations > 0)
        assert abs(durations.sum()-sample['integration_s']) < 1e-9
        np.testing.assert_allclose(np.average(sample['window_positions'], weights=durations, axis=0), sample['position'])
    print('Onboard media provenance, physical return and count-window consistency verified')


if __name__ == '__main__':
    main()
