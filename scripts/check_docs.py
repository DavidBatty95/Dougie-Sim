#!/usr/bin/env python3
"""Check maintained Markdown's local links and required publication files."""
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def maintained_documents(root=ROOT):
    files = list(root.glob('*.md')) + list((root / 'docs').glob('*.md'))
    files += list((root / 'docs/literature_review').glob('*.md')) + list((root / 'profile').glob('*.md'))
    # Check the project-owned navigation surfaces, without treating historical
    # archive contents or upstream submodule documentation as maintained guides.
    for directory in ('scripts', 'patches', 'docker', 'tools', 'integrations',
                      'integrations/grove_g1', 'archive', 'docs/results', 'ros2_ws/src'):
        files.extend((root / directory).glob('README.md'))
    external = {'unitree_mujoco', 'unitree_ros2', 'unitree_sdk2'}
    files.extend(path for path in (root / 'ros2_ws/src').glob('*/README.md')
                 if path.parent.name not in external)
    return sorted(set(files))


def main():
    failures = []
    files = maintained_documents()
    for path in files:
        text = path.read_text()
        links = re.findall(r'\]\(([^)]+)\)', text) + re.findall(r'(?:href|src)="([^"]+)"', text)
        for link in links:
            url = urlsplit(link.split(' "', 1)[0])
            if url.scheme or url.netloc or not url.path:
                continue
            target = path.parent / unquote(url.path)
            if not target.exists():
                failures.append(f'{path.relative_to(ROOT)}: missing {link}')
        if re.search(r'bitbucket\.org|vec-uol', text, re.I):
            failures.append(f'{path.relative_to(ROOT)}: work-account reference')
    for directory in (ROOT / 'docs/media', ROOT / 'profile/assets'):
        for path in directory.rglob('*.svg'):
            try:
                ET.parse(path)
            except ET.ParseError as error:
                failures.append(f'{path.relative_to(ROOT)}: invalid SVG: {error}')
    if failures:
        raise SystemExit('\n'.join(failures))
    print(f'Local links checked in {len(files)} maintained/root documents')


if __name__ == '__main__':
    main()
