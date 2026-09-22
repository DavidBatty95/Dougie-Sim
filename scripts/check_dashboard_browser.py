#!/usr/bin/env python3
"""Optional real-Firefox dashboard check using WebDriver's HTTP API (no pip deps)."""
import argparse
import base64
import json
from pathlib import Path
import socket
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    profile_root = ROOT / 'results/browser-check'
    profile_root.mkdir(parents=True, exist_ok=True)
    driver = subprocess.Popen(['geckodriver', '--port', str(port), '--profile-root', str(profile_root)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    endpoint = f'http://127.0.0.1:{port}'
    session = None
    def request(method, path, payload=None):
        body = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request(endpoint+path, data=body, method=method,
                                     headers={'Content-Type': 'application/json'})
        return json.load(urllib.request.urlopen(req, timeout=30))['value']
    def wait(function, seconds=20):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            try:
                value = function()
                if value:
                    return value
            except (OSError, KeyError):
                pass
            time.sleep(.1)
        raise RuntimeError('Browser check timed out')
    try:
        wait(lambda: request('GET', '/status'))
        session = request('POST', '/session', {'capabilities': {'alwaysMatch': {
            'browserName': 'firefox', 'moz:firefoxOptions': {'args': ['-headless']}}}})['sessionId']
        prefix = f'/session/{session}'
        def script(code):
            return request('POST', prefix+'/execute/sync', {'script': code, 'args': []})
        request('POST', prefix+'/window/rect', {'width':1440, 'height':1500})
        request('POST', prefix+'/url', {'url':args.url})
        wait(lambda: script("return document.querySelectorAll('#runs tr').length > 0"))
        script("document.getElementById('dataset').value='avoidance_baseline_v1.json'; document.getElementById('dataset').dispatchEvent(new Event('change'))")
        wait(lambda: script("return document.querySelectorAll('#runs tr').length === 18 && document.getElementById('download').getAttribute('href').includes('avoidance_baseline_v1.json')"))
        script("document.getElementById('search').value='large_pillars'; document.getElementById('search').dispatchEvent(new Event('input'))")
        assert script("return document.querySelectorAll('#runs tr').length") == 3
        script("document.querySelector('#runs tr').click()")
        wait(lambda: script("return !document.getElementById('timeline').disabled"))
        before = script("return document.getElementById('replay').toDataURL()")
        script("const t=document.getElementById('timeline'); t.value=Math.floor(t.max/2); t.dispatchEvent(new Event('input'))")
        assert script("return document.getElementById('replay').toDataURL()") != before
        assert script("return document.getElementById('error').textContent") == ''
        if args.screenshot:
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            args.screenshot.write_bytes(base64.b64decode(request('GET', prefix+'/screenshot')))
        print('Firefox verified: dataset selection, filtering, run details and trajectory replay')
    finally:
        if session:
            request('DELETE', f'/session/{session}')
        driver.terminate()
        driver.wait(timeout=10)


if __name__ == '__main__':
    main()
