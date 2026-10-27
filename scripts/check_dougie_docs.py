#!/usr/bin/env python3
"""Check local links across the maintained Go2 documentation and entry-point guides."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
files = [*(ROOT/name for name in ('README.md', 'CONTRIBUTING.md', 'LICENSES.md', 'SECURITY.md',
                                 'docker/README.md', 'scripts/README.md', 'tools/README.md')),
         *sorted((ROOT/'docs').rglob('*.md'))]
errors = []
for path in files:
    content = path.read_text()
    targets = re.findall(r'\]\(([^)]+)\)', content)+re.findall(r'(?:src|href)="([^"]+)"', content)
    for target in targets:
        if '://' in target:
            continue
        destination, _, fragment = target.partition('#')
        resolved = (path.parent/destination).resolve() if destination else path
        if not resolved.exists():
            errors.append(f'{path.relative_to(ROOT)}: missing {target}')
        elif fragment and resolved.suffix == '.md':
            headings = re.findall(r'^#+\s+(.+)$', resolved.read_text(), re.M)
            slugs = [re.sub(r'[^\w\- ]', '', h.lower()).replace(' ', '-') for h in headings]
            if fragment not in slugs:
                errors.append(f'{path.relative_to(ROOT)}: missing anchor {target}')
if errors:
    raise SystemExit('\n'.join(errors))
print(f'Active Dougie documentation links checked: {len(files)} files')
