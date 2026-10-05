#!/usr/bin/env python3
"""Validate the 50 structured appraisals and render their PDF/audit appendices."""
import argparse
from collections import Counter
import json
from pathlib import Path

from verify_sources import latex

ROOT = Path(__file__).resolve().parent


def outputs():
    appraisals = json.loads((ROOT/'paper_appraisals.json').read_text())
    identities = {item['key']: item for item in json.loads((ROOT/'verified_sources.json').read_text())}
    retrieval = json.loads((ROOT/'fulltext_retrieval.json').read_text())
    keys = [item['key'] for item in appraisals]
    if len(keys) != len(set(keys)) or set(keys) != set(identities):
        raise ValueError('Appraisals must cover each bibliographic identity exactly once')
    counts = Counter(item['inspection'] for item in appraisals)
    if set(counts) - {'selected-full-text', 'record-limited'}:
        raise ValueError('Unknown inspection level')
    tex = [r'''\section{Detailed paper-by-paper critical appraisals}
\label{app:appraisals}
The following structured records are part of the review, not a substitute for its
thematic synthesis. Each separates a method's contribution, the inspected evidence,
critical interpretation and implications for the project. ``Selected full text''
means the identified passages were inspected; it does not mean code replication,
verification of every appendix, or equivalence between a preprint and its later
version of record. Record-limited entries remain explicit evidence limitations.

''']
    audit = ['# Full-text access and expanded inspection audit\n\n',
             'Expansion date: **5 October 2026**. This supersedes the original reading-depth summary,\n'
             'while preserving the earlier identity/retrieval snapshot. The scientific corpus remains\n'
             '**50 unique works**; this is systematic extraction within a previously selected corpus.\n\n',
             f"**{counts['selected-full-text']} selected full-text appraisals; {counts['record-limited']} record-limited appraisals.**\n\n",
             'Full texts were located through existing arXiv records, author/institutional copies and\n'
             'DOI-matched OpenAlex location records. OpenAlex supplied locators, not primary results.\n'
             'All attempts and PDF/text fingerprints are in [fulltext_retrieval.json](fulltext_retrieval.json).\n'
             'Retrieved PDFs and extracted text remain in the ignored `build/fulltexts/` cache.\n'
             'Retrieval is not reading: the inspection anchors and critical appraisals below are the\n'
             'review record. No external algorithm or user study was replicated.\n\n',
             '| Key / publication | Inspection | Material and source anchor |\n| --- | --- | --- |\n']
    last_theme = None
    for item in appraisals:
        key = item['key']
        record = identities[key]
        for field in ('theme', 'short_title', 'loci', 'mechanism', 'evidence', 'critique', 'relevance'):
            if not isinstance(item.get(field), str) or not item[field].strip():
                raise ValueError(f'Missing {field}: {key}')
        if item['inspection'] == 'selected-full-text' and not retrieval[key].get('pdf_sha256'):
            raise ValueError(f'Full-text appraisal has no retrieved source: {key}')
        if item['theme'] != last_theme:
            tex.append('\\subsection{'+latex(item['theme'])+'}\n')
            last_theme = item['theme']
        tex.append('\\subsubsection{'+latex(item['short_title'])+'}\\label{paper:'+key+'}\n')
        tex.append('\\textbf{Publication:} \\citet{'+key+'}. \\emph{'+latex(record['title'])+'}. '
                   '\\href{'+record['url']+'}{Publication record}.\n\n')
        level = 'Selected full-text inspection' if item['inspection'] == 'selected-full-text' else 'Record-limited inspection'
        source_url = retrieval[key].get('pdf_url')
        tex.append('\\textbf{Evidence anchor:} '+latex(level+'. '+item['loci']) +
                   (' \\href{'+source_url+'}{Inspected source copy}.' if source_url else '')+'\n\n')
        for field, heading in (('mechanism', 'Method and contribution'), ('evidence', 'What the evidence establishes'),
                               ('critique', 'Critical appraisal'), ('relevance', 'Implication for novel work')):
            tex.append('\\paragraph{'+heading+'.} '+latex(item[field])+'\n\n')
        location = item['loci'].replace('|', '\\|')
        if source_url:
            location += f' [Inspected copy]({source_url}); SHA-256 `{retrieval[key]["pdf_sha256"][:16]}…`.'
        else:
            location += ' See the retained failed/unavailable locator attempts.'
        audit.append(f'| [`{key}`]({record["url"]}) | {level} | {location} |\n')
    audit.append('\n## Interpretation\n\n'
                 'Paper-specific numbers in the PDF are attributed to an inspected table or explicitly\n'
                 'labelled abstract. Results are not pooled across incompatible task, sensing, metric\n'
                 'or embodiment definitions. Critical interpretations and proposed experiments are\n'
                 'the review synthesis, not claims made by every cited author.\n\n'
                 'The four record-limited originals are MuJoCo (2012), sparse trajectory optimization\n'
                 '(2013), velocity obstacles (1998), and cooperation in dense crowds (2013).\n'
                 'Accessible software descriptions/institutional abstracts are labelled context, not\n'
                 'replacement full-text experimental evidence. Obtain those originals and extend\n'
                 'citation screening before claiming exhaustive state-of-the-art coverage.\n')
    return {'paper_appraisals.tex': ''.join(tex).rstrip()+'\n', 'FULLTEXT_AUDIT.md': ''.join(audit)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for name, expected in outputs().items():
        path = ROOT/name
        if args.check:
            if not path.exists() or path.read_text() != expected:
                raise SystemExit(f'Outdated review appendix: {name}')
        else:
            path.write_text(expected)
    print('50 structured paper appraisals and their inspection audit are consistent.')


if __name__ == '__main__':
    main()
