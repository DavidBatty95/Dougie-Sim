#!/usr/bin/env python3
"""Publish unchanged native footage of a failed mission's terminal-stance trial."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT/'docs/media/dougie'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads((args.run/'run_config.json').read_text())
    summary = json.loads((args.run/'radiation_summary.json').read_text())
    capture = json.loads((args.run/'demo_manifest.json').read_text())
    if (config.get('terminal_hold') != 'gravity' or config.get('localization') != 'fused'
            or summary.get('round_trip_completed') is not False or not summary.get('terminal_stop')):
        raise ValueError('expected the explicitly failed, fused terminal-gravity experiment')
    if digest(args.run/'demo.gif') != capture['sha256']:
        raise ValueError('native capture checksum mismatch')
    destination = MEDIA/'facility-terminal-hold.gif'
    shutil.copyfile(args.run/'demo.gif', destination)
    with Image.open(destination) as image:
        assert image.n_frames == len(capture['frame_times_s'])
        assert list(image.size) == capture['size']
        for index in range(image.n_frames):
            image.seek(index)
            image.load()
            assert image.info['duration'] == capture['frame_duration_ms']
        image.convert('RGB').save(MEDIA/'facility-terminal-hold.png')
    names = ('run_config.json', 'radiation_summary.json', 'demo_manifest.json', 'scene.xml')
    manifest = dict(scope='native MuJoCo terminal-stance experiment; failed vertical mission, not successful recovery',
        origin=str(args.run), publisher_sha256=digest(Path(__file__)), capture=capture,
        bytes=destination.stat().st_size, sha256=digest(destination),
        input_sha256={name: digest(args.run/name) for name in names},
        runtime_source_sha256=config['source_sha256'],
        outcomes={key: summary.get(key) for key in ('end_state', 'error', 'round_trip_completed',
            'round_trip_failures', 'terminal_stop', 'true_dose_usv', 'post_stop_true_dose_usv', 'nonfoot_contacts')},
        terminal_stance=summary['simulation'].get('terminal_stance'))
    (MEDIA/'facility-terminal-hold.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    print(f'Verified and published {len(capture["frame_times_s"])} native frames: {destination}')


if __name__ == '__main__':
    main()
