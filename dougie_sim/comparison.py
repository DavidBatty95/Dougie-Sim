"""Run independent radiation-blind/aware missions with a live browser dashboard."""
import argparse
from copy import copy,deepcopy
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


class CompletedWorker:
    """Read-only dashboard replay of retained terminal process outcomes."""
    def __init__(self, code):
        self.returncode = code

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode


def mission_command(args, mode, output):
    return [sys.executable, '-u', '-m', 'dougie_sim.radiation_run',
            '--headless', '--fast', '--fixed-plan-delay', '0.5', '--comparison-stream',
            *(['--no-video'] if getattr(args, 'no_video', False) else []),
            '--survey-until-complete',
            '--radiation-awareness', mode, '--world', args.world,
            '--duration', str(args.duration), '--seed', str(args.seed),
            '--sensor-seed', str(args.sensor_seed), '--layout-seed', str(args.layout_seed),
            *(['--no-dose-budget'] if args.dose_budget is None else ['--dose-budget', str(args.dose_budget)]),
            '--localization', getattr(args, 'localization', 'fused'),
            '--radiation-policy', getattr(args, 'radiation_policy', 'rate-limited'),
            '--exposure-policy',getattr(args,'exposure_policy','standard'),
            '--gp-mode', getattr(args, 'gp_mode', 'windowed'),
            '--gp-expert-capacity', str(getattr(args, 'gp_expert_capacity', 32)),
            '--gp-max-experts', str(getattr(args, 'gp_max_experts', 16)),
            '--rate-limit', str(args.rate_limit),
            '--frontier-rate-limit',str(getattr(args,'frontier_rate_limit',None) if getattr(args,'frontier_rate_limit',None) is not None else args.rate_limit),
            '--frontier-confidence',str(getattr(args,'frontier_confidence',.95)),
            '--frontier-view-policy',getattr(args,'frontier_view_policy','raw'),
            '--campaign-strategy',getattr(args,'campaign_strategy','baseline'),
            '--route-dose-weight',str(getattr(args,'route_dose_weight',1.) if mode=='aware' else 1.),
            '--frontier-dose-weight',str(getattr(args,'frontier_dose_weight',2.) if mode=='aware' else 2.),
            *(['--lifetime-dose',str(args.lifetime_dose),'--survival-fraction',str(getattr(args,'survival_fraction',.8))]
                if getattr(args,'lifetime_dose',None) is not None else []),
            *(['--campaign-state',str(args.campaign_inputs[mode])] if getattr(args,'campaign_inputs',None) else []),
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
    def __init__(self, *args, output=None, processes=None, feed_fps=15, session=None, **kwargs):
        self.session = session
        if session is not None:
            output, processes = session.current()
        self.output, self.processes = output, processes
        self.feed_fps = feed_fps
        super().__init__(*args, directory=str(output), **kwargs)

    def do_GET(self):
        if self.session is not None:
            with self.session.lock:
                self.output, self.processes = self.session.current()
                session_state = self.session.state()
        path = self.path.split('?')[0]
        if path in ('/blind/stream.mjpg', '/aware/stream.mjpg'):
            self.stream_frames(path.split('/')[1])
            return
        if path == '/':
            payload = (ROOT/'dougie_sim/comparison.html').read_bytes()
            kind = 'text/html; charset=utf-8'
        elif path == '/comparison-metrics.js':
            payload = (ROOT/'dougie_sim/comparison_metrics.js').read_bytes()
            kind = 'application/javascript; charset=utf-8'
        elif path == '/api':
            result = {mode: dict(
                rows=read_rows(self.output/mode/'comparison.jsonl'),
                exit_code=self.processes[mode].poll(),
                summary=json.loads((self.output/mode/'radiation_summary.json').read_text())
                    if self.processes[mode].poll() is not None and (self.output/mode/'radiation_summary.json').exists() else None)
                for mode in MODES}
            if self.session is not None:
                result['_session'] = session_state
                if session_state.get('campaign'):
                    for mode in MODES:
                        result[mode]['campaign_rows']=self.session.campaign_rows(mode,result[mode]['rows'],
                            session_state['run_id'],session_state['campaign']['id'])
            payload = json.dumps(result).encode()
            kind = 'application/json'
        elif path in ('/blind/frame.jpg', '/aware/frame.jpg'):
            frame = self.output/path.lstrip('/')
            if not frame.exists():
                self.send_error(404, 'Waiting for first simulation frame')
                return
            payload, kind = frame.read_bytes(), 'image/jpeg'
        elif path == '/radiation':
            radiation = self.output/'aware/radiation_map.json'
            if not radiation.exists():
                self.send_error(503, 'Waiting for radiation map')
                return
            payload, kind = radiation.read_bytes(), 'application/json'
        elif path == '/config':
            payload, kind = (self.output/'comparison_config.json').read_bytes(), 'application/json'
        elif path == '/history' and self.session is not None:
            payload, kind = json.dumps(self.session.history(), allow_nan=False).encode(), 'application/json'
        elif path == '/learning' and self.session is not None:
            payload, kind = json.dumps(self.session.history()['learning'],allow_nan=False).encode(), 'application/json'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        if self.session is None:
            self.send_error(405)
            return
        try:
            if self.headers.get_content_type() != 'application/json':
                raise ValueError('Controls require application/json')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 1024:
                raise ValueError('Expected a small JSON control request')
            body = json.loads(self.rfile.read(length))
            if self.path == '/rerun' and body == {}:
                self.session.request_rerun()
            elif self.path == '/continuous' and isinstance(body, dict) and type(body.get('enabled')) is bool and len(body) == 1:
                self.session.set_continuous(body['enabled'])
            elif self.path == '/learning' and isinstance(body,dict) and type(body.get('enabled')) is bool and len(body)==1:
                self.session.set_learning(body['enabled'])
            elif self.path=='/campaign/new' and isinstance(body,dict) and set(body)=={'lifetime_dose','sorties'}:
                self.session.new_campaign(body['lifetime_dose'],body['sorties'])
            else:
                raise ValueError('Unknown control or invalid settings')
        except (ValueError, RuntimeError) as error:
            payload = json.dumps(dict(error=str(error))).encode()
            self.send_response(409 if isinstance(error, RuntimeError) else 400)
        else:
            payload = json.dumps(self.session.state()).encode()
            self.send_response(202)
        self.send_header('Content-Type', 'application/json')
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
    import signal
    for process in processes.values():
        if process.poll() is None:
            try:
                # Let the mission drain/finalize its planner and write artifacts.
                # Interrupting the whole group here kills that worker first and
                # turns a censored run into a missing-summary failure.
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
    deadline = time.monotonic()+30.
    for process in processes.values():
        try:
            process.wait(timeout=max(.1, deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


class ComparisonSession:
    """One serialized supervisor; HTTP controls only queue lifecycle changes."""
    def __init__(self, args, output):
        self.args, self.root = copy(args), output
        self.lock = threading.RLock()
        self.output, self.processes, self.logs = output, {}, []
        self.records, self.samples = [], []
        self.pending = self.transitioning = self.closed = False
        self.continuous = getattr(args, 'continuous', False) or getattr(args,'learn_planning',False)
        self.error = None
        self.ended_at = None
        self.revision = 0
        from .comparison_history import session_averages
        self.averages = session_averages([])
        self.learner = self._new_learner()
        self.campaign=self._new_campaign() if getattr(args,'campaign',False) else None
        self.campaign_archives=[]
        self.campaign_request=None
        if self.campaign:
            self.learner.enabled=False
        if args.replay:
            config = json.loads((output/'comparison_config.json').read_text())
            for name in ('world', 'duration', 'seed', 'sensor_seed', 'layout_seed', 'dose_budget',
                         'localization', 'radiation_policy', 'exposure_policy', 'gp_mode',
                         'gp_expert_capacity', 'gp_max_experts', 'rate_limit', 'feed_fps', 'no_video'):
                if name in config:
                    setattr(self.args, name, config[name])
            for name in ('frontier_rate_limit','frontier_confidence','frontier_view_policy','route_dose_weight','frontier_dose_weight',
                         'campaign','lifetime_dose','survival_fraction','campaign_sorties','campaign_strategy'):
                if name in config:
                    setattr(self.args,name,config[name])
            manifest = output/'comparison_session.json'
            saved = json.loads(manifest.read_text()) if manifest.exists() else None
            from .planning_learning import PlanningLearner,evaluate_attempt
            self.learner = (PlanningLearner.load(saved['learning'],enabled=getattr(args,'learn_planning',False))
                if saved and saved.get('learning') else self._new_learner())
            if saved and saved.get('campaign'):
                from .campaign import RadiationCampaign
                self.campaign=RadiationCampaign.load(saved['campaign'])
                self.campaign_archives=saved.get('campaign_archives',[])
            elif getattr(self.args,'campaign',False):
                self.campaign=self._new_campaign()
            if self.campaign:
                self.learner.enabled=False
            if saved:
                self.records = saved['runs']
                for record in self.records:
                    directory = self.root/record['directory']
                    if record['status'] == 'running':
                        record['status'] = 'server_interrupted'
                        record['exit_codes'] = {m: None for m in MODES}
                    self.samples.append({m: read_rows(directory/m/'comparison.jsonl') for m in MODES})
                    record['learning_result'] = evaluate_attempt(record,self.samples[-1],self.learner.milestone)
                    self.learner.observe(record,record['learning_result'])
                    if (self.campaign and record.get('campaign_id')==self.campaign.id
                            and not any(r['run_id']==record['id'] for r in self.campaign.history)):
                        self.campaign.finish(record,directory,self._summaries(directory))
                self.output = self.root/self.records[-1]['directory']
                self.args.seed = self.records[-1]['seed']
                self.processes = {m: CompletedWorker(self.records[-1]['exit_codes'].get(m) if self.records[-1]['exit_codes'].get(m) is not None else 1) for m in MODES}
                self._refresh_averages()
            else:
                codes = json.loads((output/'comparison_status.json').read_text())
                if not all(type(codes.get(m)) is int for m in MODES):
                    raise ValueError('Replay requires terminal integer exit codes')
                self.processes = {m: CompletedWorker(codes[m]) for m in MODES}
                self.records.append(self._record(output, self.args.seed))
                self._finish('ended', write_status=False)
            self.ended_at = time.monotonic()
            if not self.learner.started:
                self.learner.set_enabled(getattr(args,'learn_planning',False) and self.campaign is None,self.args.seed+1)
            if self.campaign and self.campaign.status!='active':
                self.continuous=False
        else:
            self._launch(output, args.seed)

    def _new_learner(self):
        from .planning_learning import PlanningLearner,planning_weights
        return PlanningLearner(planning_weights(getattr(self.args,'route_dose_weight',1.),getattr(self.args,'frontier_dose_weight',2.)),
            milestone=getattr(self.args,'learning_milestone',75.),seeds_per_round=getattr(self.args,'learning_seeds',3),
            base_seed=self.args.seed,enabled=getattr(self.args,'learn_planning',False))

    def _new_campaign(self,limit=None,sorties=None,identifier=1):
        from .campaign import RadiationCampaign
        return RadiationCampaign(limit if limit is not None else (getattr(self.args,'lifetime_dose',None) or 2.),
            sorties if sorties is not None else getattr(self.args,'campaign_sorties',6),
            getattr(self.args,'survival_fraction',.8),identifier)

    @staticmethod
    def _summaries(directory):
        result={}
        for mode in MODES:
            try:
                result[mode]=json.loads((Path(directory)/mode/'radiation_summary.json').read_text())
            except (OSError,ValueError):
                result[mode]=dict(artifact_error='missing or invalid mission summary')
        return result

    def _record(self, output, seed):
        return dict(id=len(self.records)+1, directory=str(output.relative_to(self.root)),
                    seed=seed, status='running', exit_codes={m: None for m in MODES})

    def _save(self):
        temporary = self.root/'comparison_session.tmp'
        temporary.write_text(json.dumps(dict(runs=self.records, continuous=self.continuous,learning=self.learner.save(),
            campaign=self.campaign.save() if self.campaign else None,campaign_archives=self.campaign_archives), indent=2)+'\n')
        temporary.replace(self.root/'comparison_session.json')

    def _launch(self, output, seed):
        run_args = copy(self.args)
        trial = self.learner.propose()
        if trial is not None:
            seed = trial['seed']
        weights = trial['weights'] if trial else self.learner.best
        run_args.route_dose_weight,run_args.frontier_dose_weight = weights['route_dose_weight'],weights['frontier_dose_weight']
        run_args.learning_milestone,run_args.learning_seeds = self.learner.milestone,self.learner.seeds_per_round
        run_args.learn_planning,run_args.continuous = trial is not None,self.continuous
        run_args.seed, run_args.replay, run_args.output = seed, None, output
        if self.campaign:
            run_args.lifetime_dose=self.campaign.limit
        import hashlib
        protocol = {k:getattr(run_args,k,None) for k in ('world','sensor_seed','layout_seed','duration','dose_budget',
            'localization','radiation_policy','exposure_policy','gp_mode','gp_expert_capacity','gp_max_experts',
            'rate_limit','frontier_rate_limit','frontier_confidence','frontier_view_policy','lifetime_dose','survival_fraction','campaign_strategy')}
        if self.campaign:
            protocol['campaign_weights']=weights
        digest = hashlib.sha256(json.dumps(protocol,sort_keys=True).encode())
        for source in sorted((ROOT/'dougie_sim').glob('*.py'))+sorted((ROOT/'config').glob('*.json')):
            digest.update(str(source.relative_to(ROOT)).encode())
            digest.update(source.read_bytes())
        protocol_sha256 = digest.hexdigest()
        if self.campaign:
            run_args.campaign_inputs=self.campaign.prepare(output,protocol_sha256)
        commands = {m: mission_command(run_args, m, output) for m in MODES}
        (output/'comparison_config.json').write_text(json.dumps(dict(
            **{k: str(v) if isinstance(v, Path) else {m:str(p) for m,p in v.items()} if k=='campaign_inputs' else v for k,v in vars(run_args).items()}, commands=commands,learning_trial=trial,
            campaign_package=self.campaign.payload() if self.campaign else None,
            planning_weights=dict(blind=dict(route_dose_weight=1.,frontier_dose_weight=2.),aware=weights),
            learning_protocol_sha256=protocol_sha256,
            completion_rule='Planner declares finished after zero observed-map frontier nodes on three fresh scans. Evaluator coverage percentage never controls completion.',
            coverage_metric='physically_reached_3d_floor_v2',
            coverage_definition='Fixed 25 cm floor samples, fresh returns and capture-time support heights; flat elevations require physical arrival.',
            dose_definition='Integrated simulated true dose-equivalent proxy in micro-sieverts.',
            timing='Independent simulated clocks, fixed 0.5 s planning latency; curves end at each mission end.'), indent=2)+'\n')
        with self.lock:
            self.output, self.processes, self.logs = output, {}, []
            self.args.seed = seed
            self.records.append(self._record(output, seed))
            self.records[-1].update(learning_trial=trial,learning_protocol_sha256=protocol_sha256,planning_weights=dict(
                blind=dict(route_dose_weight=1.,frontier_dose_weight=2.),aware=deepcopy(weights)))
            if self.campaign:
                self.records[-1].update(campaign_id=self.campaign.id,sortie=len(self.campaign.history)+1)
            try:
                for mode in MODES:
                    log = (output/(mode+'.log')).open('w')
                    self.logs.append(log)
                    self.processes[mode] = subprocess.Popen(commands[mode], cwd=ROOT, stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True,
                        env={**os.environ, 'MUJOCO_GL': os.environ.get('MUJOCO_GL', 'egl'),
                             'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'})
            except Exception:
                stop_processes(self.processes)
                for mode in MODES:
                    self.processes.setdefault(mode, CompletedWorker(1))
                self._finish('launch_failed')
                raise
            self.ended_at = None
            self.revision += 1
            self._save()

    def current(self):
        with self.lock:
            return self.output, dict(self.processes)

    def state(self):
        with self.lock:
            return dict(run_id=self.records[-1]['id'], seed=self.records[-1]['seed'],
                output=str(self.output), continuous=self.continuous,
                rerun_pending=self.pending or self.transitioning,
                running=any(p.poll() is None for p in self.processes.values()),
                ended_pairs=len(self.samples), revision=self.revision, error=self.error,
                learning_enabled=self.learner.enabled,learning_round=self.learner.generation+1,
                learning_trial=self.records[-1].get('learning_trial'),run_status=self.records[-1]['status'],
                campaign=self.campaign.payload() if self.campaign else None)

    def history(self):
        with self.lock:
            return dict(runs=[dict(r) for r in self.records], averages=self.averages, revision=self.revision,
                learning=self.learner.payload(),campaign=self.campaign.payload() if self.campaign else None)

    def campaign_rows(self,mode,current,run_id,campaign_id):
        with self.lock:
            rows=[row for record,pair in zip(self.records,self.samples)
                if record.get('campaign_id')==campaign_id and record['id']<run_id for row in pair[mode]]
        rows+=current
        return [dict(time_s=r['campaign_time_s'],true_dose_usv=r['campaign_dose_usv'],
            distance_m=r['campaign_distance_m'],explored_percent=r['explored_percent'],frontier_complete=r.get('frontier_complete',False))
            for r in rows if r.get('campaign_id')==campaign_id and 'campaign_time_s' in r]

    def request_rerun(self):
        with self.lock:
            if self.closed or self.pending or self.transitioning:
                raise RuntimeError('A rerun is already pending or the dashboard is stopping')
            if self.campaign and (any(p.poll() is None for p in self.processes.values()) or self.campaign.status!='active'):
                raise RuntimeError('A campaign cannot reset a robot in the field; wait for recovery/loss or start a new campaign')
            self.pending = True
            self.revision += 1

    def set_continuous(self, enabled):
        with self.lock:
            if self.closed:
                raise RuntimeError('The dashboard is stopping')
            if enabled and self.campaign and self.campaign.status!='active':
                raise ValueError('Campaign ended; start a new campaign')
            self.continuous = enabled
            self.revision += 1
            self._save()

    def set_learning(self,enabled):
        with self.lock:
            if self.closed:
                raise RuntimeError('The dashboard is stopping')
            if enabled and self.campaign:
                raise ValueError('Campaigns retain GP/map knowledge; controlled weight tuning is paused while worlds change')
            if enabled and getattr(self.args,'exposure_policy','responsive')=='pathwise':
                raise ValueError('Planning tuning supports the standard and responsive exposure profiles')
            self.learner.set_enabled(enabled,self.args.seed+1)
            if enabled:
                self.continuous = True
            self.revision += 1
            self._save()

    def _refresh_averages(self):
        from .comparison_history import session_averages
        pairs=[p for r,p in zip(self.records,self.samples) if self.campaign is None or r.get('campaign_id')==self.campaign.id]
        self.averages = session_averages(pairs)
        self.revision += 1

    def _finish(self, reason, write_status=True):
        if self.records[-1]['status'] != 'running':
            return
        codes = {m: self.processes[m].poll() for m in MODES}
        if write_status:
            (self.output/'comparison_status.json').write_text(json.dumps(codes, indent=2)+'\n')
        pair = {m: read_rows(self.output/m/'comparison.jsonl') for m in MODES}
        summaries = self._summaries(self.output)
        self.records[-1].update(status=reason, exit_codes=codes,
            outcomes={m: dict(frontier_complete=bool(pair[m] and pair[m][-1].get('frontier_complete')),
                state=pair[m][-1].get('state') if pair[m] else 'missing telemetry',
                clean_return=summaries[m].get('clean_return'),error=summaries[m].get('error'),
                artifact_error=summaries[m].get('artifact_error')) for m in MODES})
        from .planning_learning import evaluate_attempt
        self.records[-1]['learning_result'] = evaluate_attempt(self.records[-1],pair,self.learner.milestone)
        self.learner.observe(self.records[-1],self.records[-1]['learning_result'])
        if self.campaign and self.records[-1].get('campaign_id')==self.campaign.id:
            self.campaign.finish(self.records[-1],self.output,summaries)
            if self.campaign.status!='active':
                self.continuous=False
        self.samples.append(pair)
        for log in self.logs:
            log.close()
        self.logs = []
        self.ended_at = time.monotonic()
        self._refresh_averages()
        self._save()

    def new_campaign(self,limit,sorties):
        with self.lock:
            if self.campaign is None or self.closed or self.transitioning or self.pending:
                raise RuntimeError('Campaign reset is unavailable or already pending')
            self._new_campaign(limit,sorties)  # Validate before accepting the request.
            self.campaign_request=(limit,sorties)
            self.pending=True
            self.revision+=1

    def tick(self):
        with self.lock:
            finished = all(p.poll() is not None for p in self.processes.values())
            if finished:
                self._finish('ended')
            rerun = self.pending or (finished and self.continuous and time.monotonic()-self.ended_at >= 3.)
            if self.closed or not rerun:
                return
            self.pending, self.transitioning = False, True
            processes = dict(self.processes)
        try:
            stop_processes(processes)
            with self.lock:
                self._finish('rerun_interrupted')
                if self.campaign_request is not None:
                    if self.records[-1]['status']=='rerun_interrupted':
                        self.campaign.status,self.campaign.reason='abandoned','operator started a fresh campaign; partial outcomes retained'
                    self.campaign_archives.append(self.campaign.save())
                    self.campaign=self._new_campaign(*self.campaign_request,identifier=self.campaign.id+1)
                    self.campaign_request=None
                    self.continuous=True
                    self._refresh_averages()
                elif self.campaign and self.campaign.status!='active':
                    return
            directory = self.root/'reruns'/f'run_{len(self.records)+1:05d}'
            attempt=1
            while directory.exists():
                directory=self.root/'reruns'/f'run_{len(self.records)+1:05d}_attempt_{attempt}'
                attempt+=1
            directory.mkdir(parents=True, exist_ok=False)
            self._launch(directory, self.args.seed+1)
            self.error = None
        except Exception as error:
            with self.lock:
                self.error, self.continuous = str(error), False
                self._save()
        finally:
            with self.lock:
                self.transitioning = False

    def close(self):
        with self.lock:
            self.closed, self.continuous = True, False
        stop_processes(self.processes)
        with self.lock:
            self._finish('server_interrupted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('reactor', 'mezzanine', 'facility'), default='reactor')
    parser.add_argument('--duration', type=float, default=0., help='optional simulated-time cap; 0 (default) surveys until completion or a reported limit')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--sensor-seed', type=int, default=0)
    parser.add_argument('--layout-seed', type=int, default=0)
    parser.add_argument('--dose-budget', type=float, help='optional cumulative cap in µSv; omitted means open-ended dose minimization')
    parser.add_argument('--localization', choices=('ground-truth', 'fused'), default='ground-truth',
                        help='matched pose source; simulator pose isolates radiation planning from SLAM limits')
    parser.add_argument('--radiation-policy', choices=('minimize-dose', 'rate-limited'), default='minimize-dose')
    parser.add_argument('--exposure-policy',choices=('standard','responsive','pathwise'),default='responsive')
    parser.add_argument('--gp-mode', choices=('windowed', 'geometry', 'spatial', 'global'), default='windowed')
    parser.add_argument('--gp-expert-capacity', type=int, default=32)
    parser.add_argument('--gp-max-experts', type=int, default=16, help='resident GP windows; inactive models are retained on disk in windowed mode')
    parser.add_argument('--rate-limit', type=float, default=100.)
    parser.add_argument('--frontier-rate-limit',type=float,help='survey hotspot exclusion threshold in µSv/h; default rate-limit, 0 disables')
    parser.add_argument('--frontier-confidence',type=float,default=.95)
    parser.add_argument('--frontier-view-policy',choices=('raw','marginal'),default='raw')
    parser.add_argument('--route-dose-weight',type=float,default=1.)
    parser.add_argument('--frontier-dose-weight',type=float,default=2.)
    parser.add_argument('--learn-planning',action='store_true',default=os.environ.get('DOUGIE_COMPARE_LEARNING','0')=='1',help='tune aware planning weights over matched-seed paired reruns')
    parser.add_argument('--learning-milestone',type=float,default=75.)
    parser.add_argument('--learning-seeds',type=int,default=3,help='matched detector seeds per challenger round, 2..10')
    parser.add_argument('--campaign',action='store_true',default=os.environ.get('DOUGIE_COMPARE_CAMPAIGN','0')=='1')
    parser.add_argument('--campaign-strategy',choices=('baseline','agenda'),default=os.environ.get('DOUGIE_COMPARE_CAMPAIGN_STRATEGY','baseline'))
    parser.add_argument('--lifetime-dose',type=float,default=os.environ.get('DOUGIE_COMPARE_LIFETIME_DOSE') or None,help='indicative robot failure dose; campaign default 2 µSv-equivalent')
    parser.add_argument('--campaign-sorties',type=int,default=os.environ.get('DOUGIE_COMPARE_CAMPAIGN_SORTIES','6'))
    parser.add_argument('--survival-fraction',type=float,default=.8)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', default='127.0.0.1', help='listen address; use 0.0.0.0 behind a Docker localhost port mapping')
    parser.add_argument('--feed-fps', type=int, choices=range(1, 61), default=15, metavar='1..60')
    parser.add_argument('--no-video',action='store_true',help='retain metrics without rendering (campaign validation)')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--replay', type=Path, default=os.environ.get('DOUGIE_COMPARE_REPLAY') or None,
                        help='serve an existing completed comparison without rerunning physics')
    parser.add_argument('--exit-on-complete', action='store_true')
    parser.add_argument('--continuous', action='store_true', help='start the next paired detector seed after both workers end')
    args = parser.parse_args()
    if args.campaign:
        from .campaign import RadiationCampaign
        args.lifetime_dose=args.lifetime_dose if args.lifetime_dose is not None else 2.
        try:
            RadiationCampaign(args.lifetime_dose,args.campaign_sorties,args.survival_fraction)
        except ValueError as error:
            parser.error(str(error))
        if args.localization!='ground-truth' or args.exposure_policy=='pathwise' or args.learn_planning:
            parser.error('campaigns require ground-truth pose and standard/responsive planning; controlled weight tuning is separate')
        args.continuous=True
    elif args.lifetime_dose is not None:
        from .campaign import RadiationLifetime
        try:
            RadiationLifetime(args.lifetime_dose)
        except ValueError as error:
            parser.error(str(error))
    from .planning_learning import planning_weights,PlanningLearner
    try:
        PlanningLearner(planning_weights(args.route_dose_weight,args.frontier_dose_weight),args.learning_milestone,args.learning_seeds,args.seed)
    except ValueError as error:
        parser.error(str(error))
    if args.learn_planning:
        args.continuous = True
        if args.exposure_policy=='pathwise':
            parser.error('planning tuning supports standard and responsive exposure profiles')
    if args.frontier_rate_limit is None:
        args.frontier_rate_limit = args.rate_limit
    if not math.isfinite(args.frontier_rate_limit) or args.frontier_rate_limit < 0 or not math.isfinite(args.frontier_confidence) or not .5 < args.frontier_confidence < 1.:
        parser.error('frontier threshold must be finite and nonnegative; confidence must be between .5 and 1')
    if args.continuous and args.exit_on_complete:
        parser.error('--continuous and --exit-on-complete cannot be combined')
    if args.exposure_policy == 'pathwise' and (args.dose_budget is not None or args.radiation_policy != 'minimize-dose'):
        parser.error('pathwise policy requires no dose budget and minimize-dose routing')
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.rate_limit) or args.rate_limit <= 0 or (args.dose_budget is not None and (not math.isfinite(args.dose_budget) or args.dose_budget <= 0)):
        parser.error('duration must be finite and nonnegative; dose budget and rate limit must be finite and positive')
    if not 1 <= args.port <= 65535 or min(args.seed, args.sensor_seed, args.layout_seed) < 0:
        parser.error('invalid port or negative seed')
    output = args.replay or args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('comparison_%Y%m%dT%H%M%S_%fZ')
    if args.replay:
        if not all((output/name).is_file() for name in ('comparison_config.json', 'comparison_status.json')):
            parser.error('--replay requires a complete retained comparison')
    else:
        output.mkdir(parents=True, exist_ok=False)
    server = ThreadingHTTPServer((args.host, args.port),
        partial(DashboardHandler, output=output, processes={}))
    session = None
    serving = False
    try:
        session = ComparisonSession(args, output)
        server.RequestHandlerClass = partial(DashboardHandler, session=session, feed_fps=args.feed_fps)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        serving = True
        print(f'Open http://localhost:{args.port} for the side-by-side comparison.\nResults: {output}\nCtrl+C stops both missions and closes the dashboard.', flush=True)
        reported = None
        while True:
            session.tick()
            output, processes = session.current()
            finished = all(p.poll() is not None for p in processes.values())
            if finished and reported != output:
                codes = {mode: p.returncode for mode, p in processes.items()}
                print(f'Missions finished: {codes}. Dashboard retained for inspection; logs: {output}', flush=True)
                reported = output
            if finished and args.exit_on_complete:
                break
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        if session is not None:
            session.close()
        if serving:
            server.shutdown()
        server.server_close()
    if args.exit_on_complete and any(p.returncode for p in session.processes.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
