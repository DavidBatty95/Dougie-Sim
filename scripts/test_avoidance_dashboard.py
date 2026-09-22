import csv
import io
import json
import threading
import urllib.error
import urllib.request
from functools import partial
from http.server import ThreadingHTTPServer

from avoidance_results_gui import Handler, csv_bytes, rows


def test_measurement_missing_values_and_spreadsheet_escaping(tmp_path):
    source = tmp_path / 'data.json'
    source.write_text(json.dumps({'records': [dict(id='=BAD()', role='scored', result=dict(valid_trial=False))]}))
    data = rows(source)
    assert data[0]['collisions'] is None
    exported = list(csv.DictReader(io.StringIO(csv_bytes(data).decode())))
    assert exported[0]['id'] == "'=BAD()"
    assert exported[0]['collisions'] == ''
    assert exported[0]['valid_trial'] == 'False'


def test_server_refresh_and_route_restriction(tmp_path):
    source = tmp_path / 'data.json'
    source.write_text('{"records": []}')
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Handler, dataset=source))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        assert json.load(urllib.request.urlopen(base + '/api/results')) == []
        source.write_text(json.dumps({'records': [dict(id='new', role='visual', result={})]}))
        assert json.load(urllib.request.urlopen(base + '/api/results'))[0]['id'] == 'new'
        assert json.load(urllib.request.urlopen(base + '/api/run?id=new'))['result'] == {}
        assert b'Avoidance results' in urllib.request.urlopen(base).read()
        try:
            urllib.request.urlopen(base + '/../../.env')
        except urllib.error.HTTPError as error:
            assert error.code == 404
        else:
            raise AssertionError('Unexpected file route')
    finally:
        server.shutdown()
        server.server_close()
        worker.join()
