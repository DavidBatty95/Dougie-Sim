#!/usr/bin/env python3
"""Check maintained Markdown's local links and required publication files."""
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def main():
    failures = []
    files = list(ROOT.glob('*.md')) + list((ROOT / 'docs').glob('*.md'))
    files += list((ROOT / 'docs/literature_review').glob('*.md')) + list((ROOT / 'profile').glob('*.md'))
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
        for path in directory.glob('*.svg'):
            try:
                ET.parse(path)
            except ET.ParseError as error:
                failures.append(f'{path.relative_to(ROOT)}: invalid SVG: {error}')
    if failures:
        raise SystemExit('\n'.join(failures))
    print(f'Local links checked in {len(files)} maintained/root documents')


if __name__ == '__main__':
    main()
