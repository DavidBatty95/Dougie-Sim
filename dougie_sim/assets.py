"""Checksum-verified public assets, independent of Geoff's private submodules."""
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / '.dougie/assets'


def ensure_assets():
    manifest = json.loads((ROOT / 'config/dougie_assets.json').read_text())
    for item in manifest['files']:
        path = ASSETS / item['path']
        if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']:
            continue
        print(f"Downloading {item['path']}", flush=True)
        with urlopen(item['url'], timeout=90) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != item['sha256']:
            raise RuntimeError(f"Asset checksum mismatch: {item['path']}")
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.download')
        temporary.write_bytes(data)
        temporary.replace(path)
    return ASSETS


if __name__ == '__main__':
    print(ensure_assets())
