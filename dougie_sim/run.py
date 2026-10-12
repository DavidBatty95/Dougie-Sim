"""Run the native Go2 simulation with an optional ROS bridge and passive viewer."""
import argparse
import json
import math
from pathlib import Path
import time

from .assets import ensure_assets
from .simulation import Simulation, WORLDS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=WORLDS, default='room')
    parser.add_argument('--people', type=int, choices=range(9), default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--fast', action='store_true', help='unpaced physics for headless tests')
    parser.add_argument('--duration', type=float, default=300.)
    parser.add_argument('--vx', type=float, default=0.)
    parser.add_argument('--vy', type=float, default=0.)
    parser.add_argument('--yaw-rate', type=float, default=0.)
    parser.add_argument('--controller', choices=('policy', 'stand'), default='policy')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.duration, args.vx, args.vy, args.yaw_rate)) or args.duration <= 0:
        parser.error('duration must be positive and all numeric values finite')
    if args.fast and (args.ros or not args.headless):
        parser.error('--fast requires --headless without --ros')
    ensure_assets()
    sim = Simulation(args.world, args.people, args.controller)
    bridge = None
    visualization = None
    viewer = None
    error = None
    if args.ros:
        from .ros_bridge import Bridge
        bridge = Bridge(sim, (args.vx, args.vy, args.yaw_rate))
        from .rviz_visualization import Visualization
        visualization = Visualization(bridge)
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        viewer.cam.distance = 3.
        viewer.cam.elevation = -25.
    start = time.monotonic()
    try:
        while sim.data.time < args.duration and (viewer is None or viewer.is_running()):
            command = (args.vx, args.vy, args.yaw_rate)
            if bridge:
                command = bridge.tick()
                visualization.tick()
            sim.step(command)
            if viewer and sim.counter % 4 == 0:
                viewer.cam.lookat[:] = sim.data.qpos[:3]
                viewer.sync()
            if not args.fast:
                time.sleep(max(0., start + sim.data.time - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        error = str(exc)
    finally:
        if bridge:
            bridge.close()
        if viewer:
            viewer.close()
        metrics = sim.metrics()
        metrics.update(wall_time_s=time.monotonic()-start, error=error,
                       world=args.world, people=args.people)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(metrics, indent=2)+'\n')
        print(json.dumps(metrics, indent=2))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
