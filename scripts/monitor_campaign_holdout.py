#!/usr/bin/env python3
"""Serve a read-only, auto-refreshing report while uncapped campaigns run."""
import argparse
from datetime import datetime,timezone
from html import escape
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time
from urllib.request import urlopen

try:
    from .report_campaign_holdout import inspect
except ImportError:
    from report_campaign_holdout import inspect


def worker_exit_codes(url):
    """Read process statuses from the dashboard without retaining its large traces."""
    pattern=re.compile(rb'\],\s*"exit_code"\s*:\s*(null|-?\d+)')
    found=[]
    carry=b''
    with urlopen(url,timeout=15) as response:
        while len(found)<2:
            chunk=response.read(65536)
            if not chunk:
                break
            buffer=carry+chunk
            matches=list(pattern.finditer(buffer))
            for match in matches:
                value=match.group(1)
                found.append(None if value==b'null' else int(value))
            carry=buffer[matches[-1].end():][-128:] if matches else buffer[-128:]
    if len(found)<2:
        raise ValueError('live process statuses unavailable')
    return dict(zip(('blind','aware'),found[:2]))


def render(report):
    layouts=len({s['settings'].get('layout_seed') for s in report['studies']})
    sorties=sorted({s['requested_sorties'] for s in report['studies']})
    cap='no simulated-time cap' if all(s['settings'].get('duration')==0 for s in report['studies']) else 'declared simulated-time caps'
    lines=['<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
        '<meta http-equiv="refresh" content="30"><title>Dougie-Sim campaign holdout</title>',
        '<style>body{background:#101922;color:#eef5f5;font:16px system-ui;max-width:1250px;margin:32px auto;padding:0 20px}a{color:#60ddc9}table{border-collapse:collapse;width:100%;margin:20px 0}th,td{text-align:left;border-bottom:1px solid #354550;padding:10px}th{color:#a8bdc5}p{line-height:1.6}.scroll{overflow:auto}.small{color:#a8bdc5;font-size:14px}code{color:#60ddc9}</style>',
        '<h1>Dougie-Sim: uncapped facility holdout</h1>',
        f'<p>{layouts} fixed layout(s), matched detector seeds, {escape(", ".join(map(str,sorties)))} sortie(s) per policy. The physical robot lifetime remains limited; {escape(cap)}. '
        'Active values are snapshots, not completed missions. The two layouts are development evidence, not a general performance estimate.</p>',
        '<p><a href="/holdout_progress.json">Download live JSON and per-sortie outcomes</a> · <a href="http://localhost:8770">Main dashboard</a></p>',
        f'<p class="small">Last update: {escape(report["updated_utc"])} · refreshes every 30 seconds. '
        'An externally stopped or halted package remains censored; a sortie limit is not frontier completion.</p>',
        '<div class="scroll"><table><thead><tr><th>Layout / seed</th><th>Strategy</th><th>Package</th><th>Mode</th><th>Last worker outcome</th><th>Coverage</th><th>Campaign dose</th><th>Recovered / lost</th><th>Upper landing witnesses</th><th>Probe reached / selected</th><th>Frontier completions</th></tr></thead><tbody>']
    for study in report['studies']:
        cfg=study['settings']
        for mode in ('blind','aware'):
            data=study['modes'][mode]
            live=data['active_sortie']
            coverage=live['explored_percent'] if live else data['explored_percent']
            dose=live.get('campaign_dose_usv') if live else data['total_dose_usv']
            marker=('live' if live['telemetry_age_s']<60 else f"last telemetry {round(live['telemetry_age_s'])}s ago") if live else 'finalized ledger'
            codes=study.get('live_exit_codes') or {}
            if live and mode in codes and codes[mode] is not None:
                marker=f"worker exited {codes[mode]}; pair still finalizing"
            sorties=data['sortie_outcomes']
            latest=sorties[-1] if sorties else {}
            exit_code=codes.get(mode,latest.get('exit_code'))
            outcome=latest.get('end_state') or (live or {}).get('state') or 'waiting'
            if exit_code is not None:
                outcome+=f' · exit {exit_code}'
            if latest.get('error'):
                outcome+=' · '+str(latest['error'])[:200]
            witnesses=sum(s['upper_landing_witness_s'] is not None for s in sorties)
            probes=[p for s in sorties for p in s['probes']]
            completed=sum(s['frontier_complete'] is True and (s['frontier_confirmation_count'] or 0)>=3
                and s['end_state'] in ('returned','finished') and not s['error'] for s in sorties)
            attempt=' · retry 1' if '_retry1' in Path(study['directory']).name else ''
            values=[f"{cfg['layout_seed']} / {cfg['seed']}{attempt}",cfg['campaign_strategy'],
                f"{study['status']} · {study['completed_sorties']}/{study['requested_sorties']}",mode,outcome,
                f'{coverage:.2f}% ({marker})' if coverage is not None else 'waiting',
                f'{dose:.4f} µSv ({marker})' if dose is not None else 'waiting',
                f"{data['robots_recovered']} / {data['robots_lost']}",str(witnesses),
                f"{sum(p['reached_25cm'] for p in probes)} / {len(probes)}",str(completed)]
            lines.append('<tr>'+''.join('<td>'+escape(str(v))+'</td>' for v in values)+'</tr>')
    lines+=['</tbody></table></div><p class="small">Upper-landing and completion counts only include finalized sortie summaries. '
        'Robot counts likewise update after both workers finish the pair. A probe is counted as reached only from its post-selection detector trajectory before a goal change or return, using nominal sensor height; it is not an independent foot-contact witness. '
        'Dose/coverage comparisons should use common milestones in the JSON, including the actual crossing percentages. '
        'Different numbers of active sorties and unequal endpoint coverage are not equal-work comparisons.</p>',
        '</html>']
    return ''.join(lines).encode()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--studies',nargs='+',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--live-apis',nargs='*',default=[],help='optional per-study dashboard URLs in the same order')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8777)
    args=parser.parse_args()
    studies=[Path(p) for p in args.studies]
    output=Path(args.output)
    state={'report':None,'error':'Waiting for campaign manifests'}
    if output.exists():
        try:
            state['report']=json.loads(output.read_text())
        except (OSError,ValueError):
            pass  # Keep the last valid report visible while traces are scanned.
    lock=threading.Lock()
    def refresh():
        while True:
            try:
                data=[inspect(p) for p in studies]
                for item,url in zip(data,args.live_apis):
                    try:
                        item['live_exit_codes']=worker_exit_codes(url)
                    except (OSError,ValueError) as error:
                        item['live_exit_codes']=None
                        item['live_status_error']=str(error)
                report=dict(updated_utc=datetime.now(timezone.utc).isoformat(),
                    scope='Uncapped simulated campaigns. Active and halted outcomes remain visibly censored.',
                    studies=data)
                payload=json.dumps(report,indent=2,allow_nan=False)+'\n'
                temporary=output.with_suffix(output.suffix+'.tmp')
                temporary.write_text(payload)
                temporary.replace(output)
                with lock:
                    state.update(report=report,error=None)
            except (OSError,ValueError,KeyError,TypeError) as error:
                with lock:
                    state['error']=str(error)
            time.sleep(30.)
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            with lock:
                report,error=state['report'],state['error']
            if self.path.split('?')[0] not in ('/','/holdout_progress.json'):
                self.send_error(404)
                return
            if report is None:
                self.send_error(503,error)
                return
            is_json=self.path.split('?')[0].endswith('.json')
            payload=json.dumps(report,allow_nan=False).encode() if is_json else render(report)
            self.send_response(200)
            self.send_header('Content-Type','application/json' if is_json else 'text/html; charset=utf-8')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        def log_message(self,*args):
            pass
    threading.Thread(target=refresh,daemon=True).start()
    server=ThreadingHTTPServer((args.host,args.port),Handler)
    print(f'Holdout report: http://localhost:{args.port}',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__=='__main__':
    main()
