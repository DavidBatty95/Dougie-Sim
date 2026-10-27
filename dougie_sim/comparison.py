"""Run independent radiation-blind/aware missions with a live browser dashboard."""
import argparse
from datetime import datetime, timezone
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
MODES = ('blind', 'aware')


def mission_command(args, mode, output):
    return [sys.executable, '-u', '-m', 'dougie_sim.radiation_run',
            '--headless', '--fast', '--fixed-plan-delay', '0.5', '--comparison-stream',
            '--survey-until-complete',
            '--radiation-awareness', mode, '--world', args.world,
            '--duration', str(args.duration), '--seed', str(args.seed),
            '--sensor-seed', str(args.sensor_seed), '--layout-seed', str(args.layout_seed),
            '--dose-budget', str(args.dose_budget), '--rate-limit', str(args.rate_limit),
            '--feed-fps', str(args.feed_fps),
            '--output', str(output/mode)]


def read_rows(path):
    """Ignore an in-progress last line; retained JSONL is also usable offline."""
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            break
    return rows


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, output, processes, feed_fps=15, **kwargs):
        self.output, self.processes = output, processes
        self.feed_fps = feed_fps
        super().__init__(*args, directory=str(output), **kwargs)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/blind/stream.mjpg', '/aware/stream.mjpg'):
            self.stream_frames(path.split('/')[1])
            return
        if path == '/':
            payload = (ROOT/'dougie_sim/comparison.html').read_bytes()
            kind = 'text/html; charset=utf-8'
        elif path == '/api':
            payload = json.dumps({mode: dict(
                rows=read_rows(self.output/mode/'comparison.jsonl'),
                exit_code=self.processes[mode].poll(),
                summary=json.loads((self.output/mode/'radiation_summary.json').read_text())
                    if self.processes[mode].poll() is not None and (self.output/mode/'radiation_summary.json').exists() else None)
                for mode in MODES}).encode()
            kind = 'application/json'
        elif path in ('/blind/frame.jpg', '/aware/frame.jpg'):
            frame = self.output/path.lstrip('/')
            if not frame.exists():
                self.send_error(404, 'Waiting for first simulation frame')
                return
            payload, kind = frame.read_bytes(), 'image/jpeg'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *args):
        pass

    def stream_frames(self, mode):
        """One connection per panel; send latest complete frames, never a backlog."""
        self.connection.settimeout(5.)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        frame = self.output/mode/'frame.jpg'
        previous = None
        try:
            while True:
                finished = self.processes[mode].poll() is not None
                try:
                    # Open then fstat: atomic replacement cannot pair the old
                    # bytes with the new frame's identity.
                    with frame.open('rb') as source:
                        stat = os.fstat(source.fileno())
                        identity = (stat.st_ino, stat.st_mtime_ns)
                        payload = source.read() if identity != previous else None
                    if payload is not None:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                                         str(len(payload)).encode()+b'\r\n\r\n'+payload+b'\r\n')
                        self.wfile.flush()
                        previous = identity
                except FileNotFoundError:
                    pass  # Workers may still be loading their scenes/policies.
                if finished:
                    self.wfile.write(b'--frame--\r\n')
                    return
                time.sleep(1./self.feed_fps)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass  # Closing a tab must not affect either simulation.


def stop_processes(processes):
    for process in processes.values():
        if process.poll() is None:
            # A separate session includes the mission's spawned planner worker.
            import signal
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for process in processes.values():
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            import signal
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='reactor')
    parser.add_argument('--duration', type=float, default=0., help='optional simulated-time cap; 0 (default) surveys until completion or a reported limit')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--sensor-seed', type=int, default=0)
    parser.add_argument('--layout-seed', type=int, default=0)
    parser.add_argument('--dose-budget', type=float, default=2.)
    parser.add_argument('--rate-limit', type=float, default=100.)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--feed-fps', type=int, choices=range(1, 61), default=15, metavar='1..60')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--exit-on-complete', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or not all(math.isfinite(v) and v > 0 for v in (args.dose_budget, args.rate_limit)):
        parser.error('duration must be finite and nonnegative; dose budget and rate limit must be finite and positive')
    if not 1 <= args.port <= 65535 or min(args.seed, args.sensor_seed, args.layout_seed) < 0:
        parser.error('invalid port or negative seed')
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('comparison_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    processes, logs = {}, []
    server = ThreadingHTTPServer(('127.0.0.1', args.port),
        partial(DashboardHandler, output=output, processes=processes, feed_fps=args.feed_fps))
    serving = False
    try:
        commands = {mode: mission_command(args, mode, output) for mode in MODES}
        (output/'comparison_config.json').write_text(json.dumps(dict(
            **{k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            commands=commands,
            coverage_metric='physically_reached_3d_floor_v2',
            coverage_definition='Fixed 25 cm floor samples within 19 cm of fresh LiDAR returns and 15 cm of capture-time foot-support height; flat elevations require physical arrival within 35 cm in 3D and 8 cm vertically. Levels counted separately.',
            dose_definition='Integrated simulated true dose-equivalent proxy in micro-sieverts.',
            timing='Independent simulated clocks, fixed 0.5 s planning latency; curves end at each mission end.'), indent=2)+'\n')
        for mode in MODES:
            log = (output/(mode+'.log')).open('w')
            logs.append(log)
            processes[mode] = subprocess.Popen(commands[mode], cwd=ROOT, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, 'MUJOCO_GL': os.environ.get('MUJOCO_GL', 'egl'),
                     'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'})
        threading.Thread(target=server.serve_forever, daemon=True).start()
        serving = True
        print(f'Open http://localhost:{args.port} for the side-by-side comparison.\nResults: {output}\nCtrl+C stops both missions and closes the dashboard.', flush=True)
        reported = False
        while True:
            finished = all(p.poll() is not None for p in processes.values())
            if finished and not reported:
                codes = {mode: p.returncode for mode, p in processes.items()}
                (output/'comparison_status.json').write_text(json.dumps(codes, indent=2)+'\n')
                print(f'Missions finished: {codes}. Dashboard retained for inspection; logs: {output}', flush=True)
                reported = True
            if finished and args.exit_on_complete:
                break
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_processes(processes)
        if serving:
            server.shutdown()
        server.server_close()
        for log in logs:
            log.close()
    if args.exit_on_complete and any(p.returncode for p in processes.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
