#!/usr/bin/env python3
"""Matched physical path-following benchmark, not exploration-quality evidence."""
from datetime import datetime, timezone
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from dougie_sim.assets import ROOT, ensure_assets
from dougie_sim.path_motion import RouteFollower, smooth_route
from dougie_sim.radiation_run import follow, support_pose
from dougie_sim.simulation import Simulation, rotation, scene_xml
from dougie_sim.terrain import SurfaceMap


class ConstantField:
    def predict(self, points):
        return np.ones(len(points)), np.full(len(points), .1), np.full(len(points), 2.)


def trial(mode, scenario):
    xml = ET.fromstring(scene_xml('room'))
    if scenario == 'obstacle':
        ET.SubElement(xml.find('worldbody'), 'geom', name='navigation_fixture', type='box',
                      pos='1 1 .075', size='.18 .18 .075')
    sim = Simulation(model_xml=ET.tostring(xml, encoding='unicode'))
    for _ in range(300):
        sim.step()
    terrain = SurfaceMap(radius=.4, max_step=.02, max_local_relief=.02)
    packet = sim.lidar_packet()
    terrain.update(packet.points, packet.supports, packet.origin)
    terrain.build_graph()
    root = terrain.nearest(support_pose(sim))
    if scenario == 'obstacle':
        goal = terrain.nearest([2., 2., 0.])
        parent = terrain.routes(root)[3]
        assert goal in parent, 'fixture must have a fully observed route'
        keys = terrain.path(parent, goal)
    else:
        keys = [root]
        for i in range(8):
            keys.extend(((root[0]+i+1, root[1]+i, root[2]), (root[0]+i+1, root[1]+i+1, root[2])))
    raw = [support_pose(sim), *[terrain.nodes[key].copy() for key in keys]]
    target = raw[-1]
    route = smooth_route(raw, terrain, ConstantField())
    follower = RouteFollower()
    follower.set_route(route.points, route.spans, support_pose(sim))
    route_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route.points, axis=0), axis=1))]
    route_offset, last_replan, replans = 0, sim.data.time, 0
    remaining = [p.copy() for p in raw[1:]]
    robot = set(np.flatnonzero(sim.model.geom_bodyid != 0))
    feet = {sim.model.geom(name).id for name in ('FR', 'FL', 'RR', 'RL')}
    start, wall = sim.data.time, time.perf_counter()
    travelled = variation = turning = impulse = peak = 0.
    previous_position = sim.data.qpos[:2].copy()
    previous_command = np.zeros(3)
    wrench = np.empty(6)
    reached, error = False, None
    try:
        while sim.data.time-start < 90.:
            position = support_pose(sim)
            if math.dist(position, target) < .16:
                reached = True
                break
            R = rotation(sim.data.qpos[3:7])
            yaw = math.atan2(R[1, 0], R[0, 0])
            if mode == 'smooth_replans' and sim.data.time-last_replan >= .5:
                # Resubmit the same certified route with a ~10 cm stale prefix.
                progress = route_arc[route_offset]+follower.progress
                route_offset = max(0, int(np.searchsorted(route_arc, max(0., progress-.1)))-1)
                spans = [[max(a, route_offset)-route_offset, b-route_offset]
                         for a, b in route.spans if b > route_offset]
                follower.set_route(route.points[route_offset:], spans, position)
                last_replan, replans = sim.data.time, replans+1
            command = follower.tick(position, yaw, .005) if mode != 'grid' else follow(remaining, position, yaw, .14)
            variation += abs(float(command[2]-previous_command[2]))
            turning += float(abs(command[0]) < .03 and abs(command[2]) > .1)*.005
            previous_command = command
            sim.step(command)
            travelled += float(np.linalg.norm(sim.data.qpos[:2]-previous_position))
            previous_position = sim.data.qpos[:2].copy()
            for i, contact in enumerate(sim.data.contact):
                a, b = int(contact.geom1), int(contact.geom2)
                if (a in robot) == (b in robot):
                    continue
                if (a if a in robot else b) not in feet:
                    mujoco.mj_contactForce(sim.model, sim.data, i, wrench)
                    force = max(0., float(wrench[0]))
                    if force > 1.:
                        impulse += force*.005
                        peak = max(peak, force)
    except RuntimeError as exc:
        error = str(exc)
    return dict(mode=mode, scenario=scenario, reached=reached, error=error, elapsed_sim_s=float(sim.data.time-start),
                distance_m=travelled, yaw_command_total_variation=variation, turning_in_place_s=turning,
                nonfoot_impulse_ns=impulse, nonfoot_peak_n=peak, wall_s=time.perf_counter()-wall,
                target=target.tolist(), final_position=sim.data.qpos[:3].tolist(),
                route=route.metrics, replans=replans, replan_rejections=follower.replan_rejections,
                follower_status=follower.status if mode != 'grid' else 'grid')


def main():
    ensure_assets()
    result = dict(scope='same observed route/policy/start/goal per scenario; one physical repeat per mode',
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (ROOT/'dougie_sim/path_motion.py', ROOT/'scripts/benchmark_dougie_motion.py')},
                  trials=[trial(mode, scenario) for scenario in ('zigzag', 'obstacle')
                          for mode in ('grid', 'smooth', 'smooth_replans')])
    output = ROOT/'results/dougie'/datetime.now(timezone.utc).strftime('motion_%Y%m%dT%H%M%S_%fZ.json')
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), **result), indent=2))
    if not all(t['reached'] and t['error'] is None and t['nonfoot_impulse_ns'] <= .1 and t['nonfoot_peak_n'] <= 25.
               for t in result['trials']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
