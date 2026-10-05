#!/usr/bin/env python3
"""Inspect the delivered PDF's bibliography links, bookmarks and page structure.

Requires the optional pypdf package. This checks document integrity, not whether
the scientific interpretations are correct. Use --write-report to save evidence.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from urllib.parse import unquote

from pypdf import PdfReader

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-report', action='store_true')
    args = parser.parse_args()
    pdf = ROOT/'literature_review.pdf'
    reader = PdfReader(pdf)
    records = json.loads((ROOT/'verified_sources.json').read_text())
    named = reader.named_destinations
    uris, destinations = set(), []
    errors = []
    for number, page in enumerate(reader.pages, 1):
        for reference in page.get('/Annots', []):
            annotation = reference.get_object()
            if annotation.get('/Subtype') != '/Link':
                continue
            if '/Rect' in annotation:
                x1, y1, x2, y2 = map(float, annotation['/Rect'])
                if min(x1, y1) < -1 or x2 > float(page.mediabox.width)+1 or y2 > float(page.mediabox.height)+1:
                    errors.append(f'Link outside page {number}')
            if '/A' in annotation:
                action = annotation['/A'].get_object()
                if '/URI' in action:
                    uris.add(unquote(str(action['/URI'])).rstrip('/').lower())
                if '/D' in action and isinstance(action['/D'], str):
                    destinations.append(str(action['/D']))
            if '/Dest' in annotation and isinstance(annotation['/Dest'], str):
                destinations.append(str(annotation['/Dest']))
    for destination in destinations:
        if destination not in named:
            errors.append('Unresolved internal destination: '+destination)
    for item in records:
        key = item['key']
        if 'cite.'+key not in named or 'cite.'+key not in destinations:
            errors.append('Missing linked bibliography entry/citation: '+key)
        if unquote(item['url']).rstrip('/').lower() not in uris:
            errors.append('Missing publication URL: '+key)

    outline = []
    def flatten(items):
        for item in items:
            if isinstance(item, list):
                flatten(item)
            else:
                outline.append({'title': item.title, 'page': reader.get_destination_page_number(item)+1})
    flatten(reader.outline)
    required = ('what makes navigation social', 'Deeper thematic synthesis',
                'ranked research gaps', 'A novelty roadmap', 'Detailed paper-by-paper', 'Linked bibliography')
    for title in required:
        if not any(title.lower() in item['title'].lower() for item in outline):
            errors.append('Missing bookmark: '+title)
    if not any(destination.startswith('page.') for destination in destinations):
        errors.append('No page back-reference links found')
    if errors:
        raise SystemExit('\n'.join(errors))
    text = '\n'.join(page.extract_text() or '' for page in reader.pages)
    report = {
        'checked_at': datetime.now(timezone.utc).isoformat(),
        'pdf': 'literature_review.pdf', 'pdf_sha256': hashlib.sha256(pdf.read_bytes()).hexdigest(),
        'pages': len(reader.pages), 'extracted_word_count': len(text.split()),
        'bibliographic_identities_with_linked_citations_and_publication_urls': len(records),
        'unique_external_urls': len(uris), 'internal_link_annotations': len(destinations),
        'bookmark_count': len(outline), 'outline': outline,
        'scope': 'PDF structure and link integrity; not independent scientific peer review',
    }
    if args.write_report:
        (ROOT/'pdf_validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(f"PDF verified: {report['pages']} pages; {len(records)} linked references; "
          f"{len(destinations)} resolved internal links; {len(outline)} bookmarks.")
    for item in outline:
        if any(term.lower() in item['title'].lower() for term in (*required, 'Gap 1:', 'Gap 2:', 'Gap 3:', 'Gap 4:', 'Gap 5:')):
            print(f"  p. {item['page']}: {item['title']}")


if __name__ == '__main__':
    main()
