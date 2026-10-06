#!/usr/bin/env python3
"""Launch the physical RGB-D transport demonstration and collect its evidence."""
import argparse
from datetime import datetime,timezone
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time

from avoidance_worlds import ROOT,SCENES
from demo_suite import case_catalog
from transport_worlds import TRANSPORT_WORLDS,INITIAL_KEYFRAME,generate
from transport_evidence import capture_source
from analyze_response_trace import write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',choices=TRANSPORT_WORLDS,default='transport_dual')
    parser.add_argument('--people',type=int,choices=range(9),default=0)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--duration',type=float,default=420.)
    parser.add_argument('--headless',action='store_true',help='Disable RViz; the native MuJoCo viewer may still open')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--fast-start',action='store_true')
    parser.add_argument('--camera-width',type=int,default=192,
                        help='Native raycast image width; 192 keeps dual-scene acquisition within the confirmation interval')
    parser.add_argument('--camera-height',type=int,default=108)
    parser.add_argument('--carry-posture',choices=('ik','payload_v1'),default='ik',
                        help='Explicit cycle-01 trained posture or historical Cartesian IK carry')
    parser.add_argument('--policy',type=Path,help='Repository-local 480/29 ONNX checkpoint; uses measured arm observations')
    args = parser.parse_args()
    if args.camera_width<=0 or args.camera_height<=0:
        parser.error('camera dimensions must be positive')
    if args.policy is not None:
        args.policy = args.policy.resolve()
        if not args.policy.is_file() or not args.policy.is_relative_to(ROOT):
            parser.error('--policy must be an existing file inside the repository')
    generate()
    run = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run.mkdir(parents=True)
    install = '/workspace/ros2_ws/.verification/install'
    if not args.fast_start:
        subprocess.run(['docker','compose','run','--rm','-e',f'COLCON_INSTALL_BASE={install}',
            '-e','COLCON_BUILD_BASE=/workspace/ros2_ws/.verification/build','-e','COLCON_LOG_BASE=/workspace/ros2_ws/.verification/log',
            'unitree-g1-sim','/workspace/scripts/build_ws.sh','--packages-select','g1_msgs','unitree_g1_mujoco_bridge',
            'unitree_g1_sim_bringup','unitree_g1_description','dynamic_obstacle_benchmarks'],cwd=ROOT,check=True)
        subprocess.run(['docker','compose','run','--rm','unitree-g1-sim','/workspace/scripts/build_unitree_mujoco.sh'],cwd=ROOT,check=True)
    command = case_catalog(run,args.seed,True,True,3,onboard_people=max(1,args.people))['onboard-waypoints'].command
    settings = dict(token.split(':=',1) for index,token in enumerate(command) if index and command[index-1]=='--sim-arg')
    container_run = '/workspace/'+str(run.relative_to(ROOT))
    settings.update(mujoco_scene=f'scene_29dof_transport_{args.world}.xml',mujoco_initial_keyframe=INITIAL_KEYFRAME,
        mujoco_pedestrians_enable='true',mujoco_pedestrians_count=str(args.people),mujoco_pedestrian_seed=str(args.seed),
        run_nav2='true',run_waypoint_explorer='false',run_waypoint_patrol='false',run_frontier_explorer='false',
        run_human_aware_motion_planner='true',run_human_collision_avoidance='true',run_dynamic_obstacle_tracker='true',
        run_arm_compositor='true',run_sim_manipulation_io='true',arm_compositor_goal_tolerance='0.08',
        bridge_arm_kp_override='160.0',bridge_arm_kd_override='4.0',
        bridge_arm_gains_require_authority='true',
        external_arm_observation_mask='true',
        run_rviz='false' if args.headless or args.no_rviz else 'true',publish_camera='true',
        camera_width=str(args.camera_width),camera_height=str(args.camera_height),camera_rate_hz='2.0',
        camera_mount_pitch_deg=str(TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.)),
        startup_current_pose_hold_time='0.0',startup_blend_time='0.0',startup_hold_time='0.0',
        startup_max_recovery_time='0.0',policy_action_ramp_time='0.0',mujoco_reset_on_controller_start='false',
        run_human_aware_run_logger='true',human_aware_run_log_path=container_run+'/trace.jsonl')
    if args.policy is not None:
        settings.update(policy_path='/workspace/'+str(args.policy.relative_to(ROOT)),external_arm_observation_mask='false')
    name = 'geoff-transport-'+str(os.getpid())
    env = dict(os.environ,COLCON_INSTALL_BASE=install,UNITREE_G1_CONTAINER_NAME=name)
    sources = [Path(__file__),ROOT/'scripts/transport_worlds.py',
               *[ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge'/name
                 for name in ('transport_demo.py','transport_perception.py','arm_kinematics.py','human_aware_motion_planner.py')]]
    if args.policy is not None:
        sources.append(args.policy)
    hashes = {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (run/'manifest.json').write_text(json.dumps(dict(world=args.world,people=args.people,seed=args.seed,
        duration_s=args.duration,build_reused=args.fast_start,carry_posture=args.carry_posture,
        camera_pitch_deg=TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.),
        settings=settings,source_sha256=hashes),indent=2)+'\n')
    capture_source(ROOT,run,args.policy)
    process = None
    try:
        with (run/'simulation.log').open('w') as log:
            process = subprocess.Popen([str(ROOT/'scripts/bringup_full_sim.sh'),*[f'{k}:={v}' for k,v in settings.items()]],
                                       cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(4.)
            pairs = '/workspace/'+str((SCENES/f'transport_pairs_{args.world}.json').relative_to(ROOT))
            shell = ('source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/.verification/install/setup.bash && '
                     f'python3 /workspace/scripts/transport_evidence.py {container_run}/runtime_provenance.json && '
                     'exec python3 -m unitree_g1_mujoco_bridge.transport_demo "$@"')
            result = subprocess.run(['docker','compose','run','--rm','--no-deps','--name',name+'-mission','unitree-g1-sim','bash','-lc',shell,'bash',
                '--pairs',pairs,'--output',container_run+'/result.json','--duration',str(args.duration),'--people',str(args.people),
                '--carry-posture',args.carry_posture,
                '--camera-pitch-deg',str(TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.))],cwd=ROOT)
            print(f'Transport artifacts: {run}',flush=True)
            return result.returncode
    finally:
        if process is not None:
            try:
                os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
            subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            subprocess.run(['docker','rm','-f',name+'-mission'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            if (run/'trace.jsonl').is_file():
                write_report(run/'trace.jsonl',run/'response_diagnostics.json')


if __name__=='__main__':
    raise SystemExit(main())
