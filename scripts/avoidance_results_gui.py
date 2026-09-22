#!/usr/bin/env python3
"""Local, dependency-free avoidance results dashboard and CSV export."""
import argparse
import csv
import io
import json
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ('id', 'role', 'world', 'world_size_m', 'complexity', 'seed', 'backend', 'pedestrian_scenario', 'onboard_only',
          'people_count', 'goals_requested', 'goals_reached', 'valid_trial',
          'success', 'invalid_reason', 'total_time_s', 'actual_path_length_m',
          'minimum_body_clearance_m', 'collisions', 'near_misses', 'fallen',
          'final_score', 'failure_reason', 'validation_return_code', 'source_artifact', 'source_sha256')


def rows(path):
    records = json.loads(path.read_text())['records']
    return [{key: (record.get('execution', record.get('suite_status', {})).get('return_code') if key == 'validation_return_code' else record.get(key) if key in record else record['result'].get(key))
             for key in FIELDS} for record in records]


def csv_bytes(data):
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=FIELDS)
    writer.writeheader()
    for row in data:
        # Prevent spreadsheet formula execution in imported text fields.
        writer.writerow({k: "'" + v if isinstance(v, str) and v.startswith(('=', '+', '-', '@'))
                         else v for k, v in row.items()})
    return output.getvalue().encode()


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, dataset, **kwargs):
        self.dataset = dataset
        super().__init__(*args, **kwargs)

    def do_GET(self):
        try:
            url = urlsplit(self.path)
            catalog = {path.name: path for path in (ROOT / 'docs/results').glob('*.json')
                       if isinstance(json.loads(path.read_text()).get('records'), list)}
            catalog[self.dataset.name] = self.dataset
            requested = parse_qs(url.query).get('dataset', [self.dataset.name])[0]
            if requested not in catalog:
                self.send_error(404)
                return
            dataset = catalog[requested]
            if url.path == '/api/catalog':
                body = json.dumps(dict(default=self.dataset.name, datasets=sorted(catalog))).encode()
                kind = 'application/json'
            elif url.path == '/api/results':
                body, kind = json.dumps(rows(dataset), allow_nan=False).encode(), 'application/json'
            elif url.path == '/api/run':
                identity = parse_qs(url.query).get('id', [''])[0]
                record = next((record for record in json.loads(dataset.read_text())['records']
                               if record['id'] == identity), None)
                if record is None:
                    self.send_error(404)
                    return
                body, kind = json.dumps(record, allow_nan=False).encode(), 'application/json'
            elif url.path == '/results.csv':
                body, kind = csv_bytes(rows(dataset)), 'text/csv; charset=utf-8'
            elif url.path == '/':
                body, kind = (ROOT / 'tools/avoidance_dashboard.html').read_bytes(), 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
        except (OSError, ValueError, KeyError) as error:
            self.send_error(500, 'Cannot load dataset: ' + str(error))
            return
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'docs/results/avoidance_dataset.json')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--export', type=Path, help='Export CSV and exit')
    args = parser.parse_args()
    data = rows(args.dataset)
    if args.export:
        args.export.parent.mkdir(parents=True, exist_ok=True)
        args.export.write_bytes(csv_bytes(data))
        print(f'Exported {len(data)} result records to {args.export}')
        return
    server = ThreadingHTTPServer(('127.0.0.1', args.port), partial(Handler, dataset=args.dataset))
    print(f'Avoidance results: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
