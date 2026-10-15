#!/usr/bin/env python3
"""Measure actual RViz GLX frame delivery with the verified Go2 mesh model.

Uses real standing/walking physics and a labelled synthetic 450-point GP display.
This isolates rendering from exploration quality. No mission result is inferred.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from dougie_sim.assets import ROOT, ensure_assets
from dougie_sim.graphics import rviz_command
from dougie_sim.ros_bridge import Bridge
from dougie_sim.rviz_visualization import Visualization
from dougie_sim.simulation import Simulation


def frame_statistics(path, start, end):
    data = np.loadtxt(path, delimiter=',', ndmin=2)
    selected = data[(data[:, 0] >= start) & (data[:, 0] < end)]
    if len(selected) < 2:
        raise RuntimeError('insufficient GLX presentations; inspect rviz.log')
    windows, counts = np.unique(selected[:, 1].astype(np.int64), return_counts=True)
    # RViz also swaps a tiny auxiliary drawable. Measure the large scene viewport,
    # never sum both windows or mistake an auxiliary context for display FPS.
    areas = [float(np.median(selected[selected[:, 1] == w, 2]*selected[selected[:, 1] == w, 3]))
             if selected.shape[1] >= 4 else 0. for w in windows]
    chosen = max(range(len(windows)), key=lambda i: (areas[i], counts[i]))
    window = windows[chosen]
    stamps = selected[selected[:, 1] == window, 0]
    intervals = np.diff(stamps)
    return dict(frames=len(stamps), measurement_wall_s=end-start,
                presented_fps=len(stamps)/(end-start),
                interval_fps=(len(stamps)-1)/(stamps[-1]-stamps[0]),
                frame_interval_p50_ms=float(np.percentile(intervals, 50)*1000),
                frame_interval_p95_ms=float(np.percentile(intervals, 95)*1000),
                drawable=int(window), observed_drawables=len(windows), viewport_pixels=int(areas[chosen]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=('room', 'reactor', 'facility'), default='facility')
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--warmup', type=float, default=8.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not all(math.isfinite(x) and x > 0 for x in (args.duration, args.warmup)):
        parser.error('duration and warmup must be positive finite seconds')
    ensure_assets()
    output = args.output or ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('rviz_benchmark_%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    library = output/'frame_probe.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-O2', str(ROOT/'tools/rviz_frame_probe.c'),
                    '-o', str(library), '-ldl', '-pthread'], check=True)
    sim = Simulation(args.world)
    bridge = Bridge(sim, publish_scan=False)
    visualization = Visualization(bridge, show_sources=True, cutaway=args.world == 'facility', field_display='all')
    x, y = np.meshgrid(np.linspace(-1., 9., 15), np.linspace(-2., 8., 10))
    xyz = np.concatenate([np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z))) for z in (.4, 1.2, 2.8)])
    mean = np.linspace(.2, 100., len(xyz))
    volume = xyz, mean, np.ones(len(xyz)), mean*1.5
    angles = np.linspace(0., 4*np.pi, 2000)
    trail = np.column_stack((4+3*np.cos(angles), 3+2*np.sin(angles), np.full(len(angles), .02))).tolist()
    metrics = dict(state='RENDER_BENCHMARK', decision_reason='synthetic map/trail; no exploration result',
                   planned_path=[[0., 0., 0.], [2., 0., 0.], [2., 2., 0.]],
                   return_path=[[2., 2., 0.], [0., 2., 0.], [0., 0., 0.]], selected_target=[2., 2., 0.])
    visualization.update(volume, metrics, trail)
    visualization.update_dashboard(metrics, None, 5., 100.)
    config = ROOT/'config'/('dougie_facility.rviz' if args.world == 'facility' else 'dougie_radiation.rviz')
    process = None
    subscribed = False
    started = time.monotonic()
    deadline = started+args.warmup+args.duration
    try:
        with (output/'rviz.log').open('w') as log:
            process = subprocess.Popen(rviz_command(config, output/'graphics.json'),
                env={**os.environ, 'LD_PRELOAD': f'{library} {os.environ.get("LD_PRELOAD", "")}'.strip(),
                     'DOUGIE_RVIZ_FRAME_LOG': str(output/'frames.csv')},
                stdout=log, stderr=subprocess.STDOUT)
            next_map = 0.
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'RViz exited early with {process.returncode}; see {output}/rviz.log')
                bridge.tick()
                visualization.tick()
                sim.step((0., 0., 0.))
                subscribed |= bool(bridge.pub_description.get_subscription_count() and
                                   visualization.scene.get_subscription_count() and visualization.mission.get_subscription_count())
                if sim.data.time >= next_map:
                    visualization.update(volume, metrics, trail)
                    visualization.update_dashboard(metrics, None, 5., 100.)
                    next_map = sim.data.time+2.
                time.sleep(max(0., started+sim.data.time-time.monotonic()))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        bridge.close()
    if not subscribed:
        raise RuntimeError('RViz did not subscribe to the Go2 model, scene and map; not a valid rendering benchmark')
    result = dict(world=args.world, graphics=json.loads((output/'graphics.json').read_text()),
                  **frame_statistics(output/'frames.csv', started+args.warmup, deadline),
                   warmup_s=args.warmup, requested_rviz_fps=30, gp_points=len(xyz), trail_points=len(trail),
                   cloud_points=len(visualization.highlights.cloud(volume, mode='all')[0]),
                   show_sources=True,
                  robot_links=len(bridge.robot_description.links),
                  urdf_sha256=hashlib.sha256(bridge.robot_description.xml.encode()).hexdigest(),
                  rviz_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                  synthetic_visualization=True, subscriptions_verified=subscribed,
                  simulated_time_s=float(sim.data.time), rviz_exit_code=process.returncode)
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), **result), indent=2))


if __name__ == '__main__':
    main()
