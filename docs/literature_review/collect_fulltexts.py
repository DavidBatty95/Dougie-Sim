#!/usr/bin/env python3
"""Locate accessible papers and cache PDFs/text for review, without claiming reading.

Publisher/author files stay in ignored build/fulltexts/. The tracked retrieval
ledger records URLs, hashes, access failures and extraction status, not PDF content.
Run --locate first, then --fetch; --keys selects an explicit subset.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time
from urllib.parse import quote
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
CACHE = ROOT / 'build/fulltexts'
LEDGER = ROOT / 'fulltext_retrieval.json'
AUTHOR_COPIES = {
    'mujoco': ['https://homes.cs.washington.edu/~todorov/papers/TodorovIROS12.pdf'],
    'teb': ['https://rst.etit.tu-dortmund.de/storages/rst-etit/r/SystemsEngineering/ECMR13_Roesmann.pdf'],
    'vo': ['https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/fiorini_vo.pdf'],
    'socnavbench': ['https://arxiv.org/pdf/2103.00047'],
    'parkour': ['https://arxiv.org/pdf/2306.14874'],
    'orca': ['https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf'],
    'dwa': ['https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf'],
    'sociallstm': ['https://openaccess.thecvf.com/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf'],
    'slam': ['https://www.theoj.org/joss-papers/joss.02783/10.21105.joss.02783.pdf'],
    'deepmimic': ['https://arxiv.org/pdf/1804.02717'],
    'orbslam': ['https://arxiv.org/pdf/2007.11898'],
    'safecontrol': ['https://arxiv.org/pdf/2108.06266'],
    'rough': ['https://arxiv.org/pdf/2201.08117'],
}


def get(url):
    request = Request(url, headers={'User-Agent': 'GeoffSim-LiteratureReview/1.0'})
    with urlopen(request, timeout=45) as response:
        data = response.read(128 * 1024 * 1024 + 1)
        if len(data) > 128 * 1024 * 1024:
            raise ValueError('Source exceeds the 128 MiB retrieval limit')
        return data, response.geturl()


def candidates(record):
    urls = list(record.get('candidate_urls', []))
    for location in record.get('locations', []):
        for field in ('pdf_url', 'landing_page_url'):
            url = location.get(field)
            if not url:
                continue
            if 'arxiv.org/abs/' in url:
                url = url.replace('http:', 'https:').replace('/abs/', '/pdf/')
            elif 'hal.science/' in url or 'hal.archives-ouvertes.fr/' in url:
                url = url.rstrip('/') + '/document' if not url.endswith('/document') else url
            elif field == 'landing_page_url':
                continue
            urls.append(url.replace('http:', 'https:'))
    return list(dict.fromkeys(urls))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--locate', action='store_true')
    parser.add_argument('--fetch', action='store_true')
    parser.add_argument('--keys', nargs='+')
    args = parser.parse_args()
    sources = json.loads((ROOT/'sources.json').read_text())
    supplemental = {item['key']: item['url'].replace('/abs/', '/pdf/')
                    for item in json.loads((ROOT/'supplemental_reading.json').read_text())}
    ledger = json.loads(LEDGER.read_text()) if LEDGER.exists() else {}
    CACHE.mkdir(parents=True, exist_ok=True)
    for source in sources:
        key = source['key']
        if args.keys and key not in args.keys:
            continue
        record = ledger.setdefault(key, {'title': source['expected_title'], 'candidate_urls': [], 'attempts': []})
        if key in AUTHOR_COPIES:
            record['candidate_urls'] = list(dict.fromkeys(AUTHOR_COPIES[key] + record['candidate_urls']))
        if key in supplemental:
            record['candidate_urls'] = list(dict.fromkeys([supplemental[key]] + record['candidate_urls']))
        if 'arxiv' in source:
            record['candidate_urls'] = ['https://arxiv.org/pdf/' + source['arxiv']]
        if args.locate and 'doi' in source and 'locations' not in record:
            url = 'https://api.openalex.org/works/https://doi.org/' + quote(source['doi'], safe='/')
            try:
                data, resolved = get(url)
                work = json.loads(data)
                if work['doi'].lower() != 'https://doi.org/' + source['doi'].lower():
                    raise ValueError('Locator DOI mismatch')
                record.update(locator_url=resolved, locator_title=work['title'],
                              locator_sha256=hashlib.sha256(data).hexdigest(),
                              located_at=datetime.now(timezone.utc).isoformat(),
                              locations=[{field: location.get(field) for field in
                                          ('pdf_url', 'landing_page_url', 'version', 'is_oa')}
                                         for location in work['locations']])
                print(key, 'located', len(record['locations']), flush=True)
            except Exception as error:
                record['locator_error'] = str(error)
                print(key, 'locator failed:', error, flush=True)
            time.sleep(.4)
        if args.fetch and not record.get('pdf_sha256'):
            for url in candidates(record)[:6]:
                attempt = {'url': url, 'retrieved_at': datetime.now(timezone.utc).isoformat()}
                try:
                    data, resolved = get(url)
                    attempt.update(resolved_url=resolved, sha256=hashlib.sha256(data).hexdigest())
                    if not data.startswith(b'%PDF'):
                        raise ValueError('Response is not PDF; landing page or access challenge retained as a failed attempt')
                    pdf, text = CACHE/(key+'.pdf'), CACHE/(key+'.txt')
                    pdf.write_bytes(data)
                    subprocess.run(['pdftotext', '-layout', str(pdf), str(text)], check=True, capture_output=True)
                    extracted = text.read_text(errors='replace')
                    if len(extracted.strip()) < 1000:
                        raise ValueError('Insufficient extractable text; requires manual inspection')
                    record.update(pdf_url=resolved, pdf_sha256=attempt['sha256'], text_sha256=hashlib.sha256(text.read_bytes()).hexdigest(),
                                  text_characters=len(extracted), retrieved_at=attempt['retrieved_at'],
                                  reading_status='Retrieved only; title and methods/evaluation require reviewer inspection')
                    attempt['status'] = 'PDF cached and text extracted'
                    print(key, 'PDF', len(data), 'bytes;', len(extracted), 'text characters', flush=True)
                except Exception as error:
                    attempt.update(status='failed', error=str(error))
                    print(key, 'failed:', url, str(error), flush=True)
                record['attempts'].append(attempt)
                LEDGER.write_text(json.dumps(ledger, indent=2, ensure_ascii=False)+'\n')
                time.sleep(1)
                if record.get('pdf_sha256'):
                    break
        LEDGER.write_text(json.dumps(ledger, indent=2, ensure_ascii=False)+'\n')


if __name__ == '__main__':
    main()
