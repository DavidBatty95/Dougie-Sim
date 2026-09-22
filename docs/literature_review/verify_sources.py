#!/usr/bin/env python3
"""Audit source identities, retain registry evidence, and generate BibTeX.

The default check is offline. --refresh contacts Crossref/arXiv and replaces
the evidence snapshot only after every source passes identity validation.
Bibliographic validation is not replication or peer-review verification.
"""

import argparse
from datetime import datetime, timezone
from difflib import SequenceMatcher
import hashlib
from html import unescape
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import time
import unicodedata
from urllib.parse import quote
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent


class MetadataParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.values = {}

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'meta' and attrs.get('name', '').startswith('citation_'):
            self.values.setdefault(attrs['name'], []).append(attrs.get('content', ''))


def normalize(value):
    value = unicodedata.normalize('NFKD', unescape(value)).casefold()
    return re.sub(r'[^a-z0-9]', '', value)


def retrieve(url):
    for attempt in range(4):
        try:
            req = Request(url, headers={'User-Agent': 'GeoffSim-LiteratureAudit/1.0'})
            with urlopen(req, timeout=60) as response:
                data = response.read()
                return data, response.geturl()
        except OSError:
            if attempt == 3:
                raise
            time.sleep(3 * (attempt + 1))


def fetch(source):
    if 'doi' in source:
        url = 'https://api.crossref.org/works/' + quote(source['doi'], safe='/')
        data, resolved = retrieve(url)
        m = json.loads(data)['message']
        if m['DOI'].casefold() != source['doi'].casefold():
            raise ValueError(f"DOI mismatch for {source['key']}")
        date = m.get('published', m.get('issued'))['date-parts'][0]
        item = {
            'title': m['title'][0],
            'authors': [a.get('family', a.get('name', '')) +
                        (', ' + a['given'] if a.get('given') else '')
                        for a in m['author']],
            'year': date[0], 'date_parts': date,
            'venue': '; '.join(m.get('container-title', [])),
            'type': m['type'], 'doi': m['DOI'],
            'url': 'https://doi.org/' + m['DOI'],
            'abstract': unescape(re.sub('<[^>]+>', '', m.get('abstract', ''))).strip(),
        }
    else:
        url = 'https://arxiv.org/abs/' + source['arxiv']
        data, resolved = retrieve(url)
        parser = MetadataParser()
        parser.feed(data.decode('utf-8'))
        m = parser.values
        ident = m['citation_arxiv_id'][0]
        if ident.split('v')[0] != source['arxiv'].split('v')[0]:
            raise ValueError(f"arXiv mismatch for {source['key']}")
        date = m['citation_date'][0]
        abstract = re.search(r'<blockquote[^>]*class="abstract[^>]*>(.*?)</blockquote>',
                             data.decode('utf-8'), re.S)
        item = {
            'title': m['citation_title'][0], 'authors': m['citation_author'],
            'year': int(date[:4]), 'date': date,
            'venue': 'arXiv (version cited; publication status not inferred)',
            'type': 'preprint-record', 'arxiv': source['arxiv'], 'url': url,
            'abstract': unescape(re.sub('<[^>]+>', '', abstract[1])).strip() if abstract else '',
        }
    item.update(key=source['key'], evidence_url=url, resolved_url=resolved,
                retrieved_at=datetime.now(timezone.utc).isoformat(),
                response_sha256=hashlib.sha256(data).hexdigest(),
                reading_level=source['reading_level'])
    validate_identity(source, item)
    return item


def validate_identity(source, item):
    score = SequenceMatcher(None, normalize(source['expected_title']), normalize(item['title'])).ratio()
    if score < 0.94:
        raise ValueError(f"Title mismatch: {source['key']}: {item['title']} ({score:.3f})")
    if not item['authors'] or not item['year'] or not item['response_sha256']:
        raise ValueError(f"Incomplete evidence: {source['key']}")
    if item['year'] > 2026:
        raise ValueError(f"Beyond review cutoff: {source['key']}")
    if 'doi' in source and source['doi'].casefold() != item.get('doi', '').casefold():
        raise ValueError(f"Snapshot DOI mismatch: {source['key']}")
    if 'arxiv' in source and source['arxiv'] != item.get('arxiv'):
        raise ValueError(f"Snapshot arXiv mismatch: {source['key']}")


def latex(value):
    value = unescape(value)
    return ''.join({'&': r'\&', '%': r'\%', '_': r'\_', '#': r'\#',
                    '$': r'\$', '{': r'\{', '}': r'\}'}.get(c, c) for c in value)


def bibliography(items):
    entries = ['% Generated from verified_sources.json by verify_sources.py --export.']
    for item in items:
        kind = 'article' if item['type'] == 'journal-article' else 'misc'
        fields = {'title': '{' + latex(item['title']) + '}',
                  'author': ' and '.join(latex(a) for a in item['authors']),
                  'year': str(item['year']), 'url': item['url']}
        if kind == 'article':
            fields['journal'] = latex(item['venue'])
        else:
            fields['howpublished'] = (r'arXiv:' + item['arxiv'] if 'arxiv' in item
                                      else latex(item['venue']))
        if 'doi' in item:
            fields['doi'] = item['doi']
        entries.append('@' + kind + '{' + item['key'] + ',\n' +
                       ',\n'.join('  ' + k + ' = {' + v + '}' for k, v in fields.items()) + '\n}')
    return '\n\n'.join(entries) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh', action='store_true', help='re-fetch all primary identity records')
    parser.add_argument('--export', action='store_true', help='regenerate BibTeX from the checked snapshot')
    args = parser.parse_args()
    sources = json.loads((ROOT / 'sources.json').read_text())
    if args.refresh:
        items = []
        for source in sources:
            item = fetch(source)
            items.append(item)
            print(f"Verified {item['key']}: {item['title']}", flush=True)
            time.sleep(0.5)
    else:
        items = json.loads((ROOT / 'verified_sources.json').read_text())
    by_key = {item['key']: item for item in items}
    if len(by_key) != len(items) or len({s['key'] for s in sources}) != len(sources):
        raise ValueError('Duplicate citation keys')
    if set(by_key) != {s['key'] for s in sources}:
        raise ValueError('Manifest and evidence snapshot differ')
    identifiers = [i.get('doi', i.get('arxiv', '')).casefold() for i in items]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('Duplicate source identifiers')
    for source in sources:
        validate_identity(source, by_key[source['key']])
    if args.refresh:
        (ROOT / 'verified_sources.json').write_text(json.dumps(items, ensure_ascii=False, indent=2) + '\n')
    expected_bib = bibliography(items)
    if args.export:
        (ROOT / 'references.bib').write_text(expected_bib)
    elif (ROOT / 'references.bib').read_text() != expected_bib:
        raise ValueError('Bibliography differs from snapshot; review changes and run --export')
    tex = '\n'.join(p.read_text() for p in ROOT.glob('*.tex'))
    cited = {key.strip() for group in re.findall(r'\\cite\w*\{([^}]+)\}', tex)
             for key in group.split(',')}
    if cited - by_key.keys():
        raise ValueError(f'Unknown citations: {sorted(cited - by_key.keys())}')
    uncited = by_key.keys() - cited
    print(f'{len(items)} source identities checked; {len(cited)} cited; {len(uncited)} uncited.')
    if uncited:
        print('Uncited:', ', '.join(sorted(uncited)))


if __name__ == '__main__':
    main()
