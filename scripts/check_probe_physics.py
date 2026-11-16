#!/usr/bin/env python3
"""Controlled probe fixture: real Go2/GP physics, injected blocked frontier.

This is a functional intervention, not a navigation efficacy comparison. The
fixture supplies one artificial pending frontier and budget-blocked status to
exercise the otherwise rare probe branch; support, motion and counts are real.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from dougie_sim.campaign_agenda import CampaignAgenda
from dougie_sim.clustered_planner import ClusteredDoseExplorer
from dougie_sim.path_motion import RouteFollower,RouteSmoother
from dougie_sim.radiation import Detector,RadiationField
from dougie_sim.radiation_run import support_pose
from dougie_sim.simulation import Simulation,rotation
from dougie_sim.survey_comparison import AVAILABLE,BUDGET
from dougie_sim.windowed_gp import WindowedGP


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    sim=Simulation('reactor')
    field=RadiationField.from_mujoco(sim.model,sim.data)
    detector=Detector(401)
    explorer=ClusteredDoseExplorer(mission_duration=1e12)
    explorer.budget_enabled=explorer.rate_limited=False
    explorer.survey_until_complete=explorer.smooth_execution=True
    explorer.campaign_agenda=CampaignAgenda()
    explorer.radiation_awareness='aware'
    explorer.gp=WindowedGP(capacity=16,max_experts=8,max_planes=0)
    samples=[]
    def advance(command):
        sensor=sim.data.qpos[:3]+[0.,0.,.12]
        sample=detector.integrate(sensor,field.rate(sensor),sim.model.opt.timestep,sim.data.time+sim.model.opt.timestep)
        if sample is not None:
            explorer.observe_radiation(sample)
            samples.append(sample)
        sim.step(command)
    while sim.data.time<1.5:
        advance(np.zeros(3))
    packet=sim.lidar_packet()
    explorer.observe_geometry(packet.points,packet.supports,packet.origin)
    nodes,_=explorer.terrain.build_graph()
    position=support_pose(sim)
    root=explorer.terrain.nearest(position)
    if root is None:
        raise RuntimeError('real scan did not provide starting support')
    _,_,length,parent=explorer.terrain.routes(root)
    keys=[k for k in parent if .6<length[k]<5.]
    _,sigma,_=explorer.gp.predict([nodes[k]+[0.,0.,.44] for k in keys])
    pairs=[(target,stand) for target,s in zip(keys,sigma) if s>1.
        for stand in keys if .35<=np.linalg.norm(nodes[target]-nodes[stand])<=1.0
        and .25<=length[stand]<length[target] and abs(nodes[stand][2]-nodes[target][2])<.18
        and explorer.utility.neighbourhood_seconds(nodes[stand]+[0.,0.,.44])<.25]
    if not pairs:
        raise RuntimeError('no observed-support uncertain target/stand-off fixture')
    target,stand=min(pairs,key=lambda pair:length[pair[1]])
    # Limit the injected available set to the selected genuinely supported node.
    # The actual planner still runs route, GP prediction, smoothing and follower.
    statuses={k:(BUDGET if k==target else AVAILABLE if k==stand or k==root else BUDGET) for k in nodes}
    observed=SimpleNamespace(values=lambda ks:np.asarray([int(k==target) for k in ks]),
        value=lambda k:int(k==target),cache={})
    with patch('dougie_sim.clustered_planner.ObservedGain',return_value=observed),\
         patch('dougie_sim.survey_comparison.survey_admission',return_value=statuses):
        explorer.start_yaw=0.
        explorer.replan(position,float(sim.data.time))
    if explorer.last_reason!='uncertainty_measurement_probe':
        raise RuntimeError(f'fixture failed to select probe: {explorer.last_reason}')
    motion=RouteSmoother(explorer.terrain,explorer.planning_gp,float('inf'),prediction='mean')([position,*explorer.path])
    follower=RouteFollower()
    if not follower.set_route(motion.points,motion.spans,position):
        raise RuntimeError('probe route was not trackable at start')
    goal=explorer.target.copy()
    before=explorer.gp.predict([goal+[0.,0.,.44]])[1][0]
    initial_samples=len(samples)
    minimum=float(np.linalg.norm(position-goal))
    while sim.data.time<35. and minimum>.25:
        p=support_pose(sim)
        matrix=rotation(sim.data.qpos[3:7])
        yaw=np.arctan2(matrix[1,0],matrix[0,0])
        advance(follower.tick(p,yaw,sim.model.opt.timestep))
        minimum=min(minimum,float(np.linalg.norm(support_pose(sim)-goal)))
        if follower.status=='waiting_for_route_reacquisition':
            break
    after=explorer.gp.predict([goal+[0.,0.,.44]])[1][0]
    result=dict(scope='Controlled intervention: artificial frontier/status, real measured support, MuJoCo locomotion and Poisson detector. Not an efficacy or safety claim.',
        selected_reason=explorer.last_reason,frontier_count=explorer.frontier_count,
        frontier_complete=explorer.frontier_complete,goal=goal.tolist(),target=nodes[target].tolist(),
        reached_within_25cm=minimum<=.25,minimum_distance_m=minimum,simulation_time_s=float(sim.data.time),
        actual_detector_windows=len(samples)-initial_samples,initial_sigma_log=float(before),final_sigma_log=float(after),
        follower_status=follower.status)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))
    if not result['reached_within_25cm'] or result['actual_detector_windows']<1:
        raise SystemExit(1)


if __name__=='__main__':
    main()
