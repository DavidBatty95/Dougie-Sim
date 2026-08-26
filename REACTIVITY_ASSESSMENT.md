# G1 Navigation Reactivity Assessment

This assessment separates command generation, policy response, and physical motion.
All live motion measurements use the official Unitree RL Lab 480-observation,
29-action policy in `scene_29dof_empty.xml`.

## Measured Policy Response

For a `0.35 m/s` forward step with the waypoint demo's `0.35 m/s^2` command slew:

- Motion onset above `0.03 m/s`: `0.707 s`
- 50% rise time: `1.205 s`
- 90% rise time: `1.411 s`
- Steady response: `0.317 m/s`
- Two-second post-stop net drift: `0.133 m`
- Maximum tilt: `0.067 rad`
- Fall: no

Raising command slew to `0.50 m/s^2` reduced the 90% rise time by about `0.28 s`,
but increased post-stop drift to `0.191 m` and maximum tilt to `0.079 rad`. It is
therefore not the default.

For a pure `0.08 m/s` lateral step:

- Eight-second lateral displacement: `0.0007 m`
- 50% and 90% rise: never reached
- Steady lateral response: `0.0005 m/s`

The deployed policy cannot execute a useful sidestep. The safety stack must not
assume that a lateral command is an available evasive action.

## End-to-End Bottlenecks

Before tuning, the person-to-command path included 10 Hz LiDAR, a tracker motion
confirmation gate, two 12 Hz social filters, and a 10 Hz Nav2 controller. In
Extreme Lab, 1,200-ray LiDAR averaged about `135 ms`, replaced queued frames, and
produced person clouds old enough to be rejected by the safety filter.

The bounded reactive configuration now uses:

- 600 simulated LiDAR rays at 10 Hz
- 20 Hz human-aware planner
- 20 Hz final collision filter
- 15 Hz Nav2 controller
- 10 Hz local costmap
- 4 Hz waypoint result/route checks
- Existing `0.18 m` tracker confirmation displacement, retained to avoid increasing
  false confirmed tracks

Both social command topics were measured at approximately 20 Hz with `40-58 ms`
intervals. Policy inference remains 50 Hz and is not the dominant delay.

## Safety Interpretation

The current policy can stop immediately at the command layer, but physical settling
from forward walking remains slow. Detection and replanning must happen early enough
to absorb roughly `0.7 s` locomotion onset and more than `0.1 m` post-stop travel.
The current safe strategy is early stop plus forward-arc replanning, not a last-moment
sidestep or reversal.

Before enabling nonzero evasive maneuvers around people, a replacement policy must
pass all of these gates:

- Bilateral lateral tracking at `0.05` and `0.10 m/s`
- Bilateral yaw and forward-arc tracking
- Motion onset below `0.35 s`
- 90% rise below `0.8 s`
- Stop settling below `0.6 s`
- Two-second post-stop drift below `0.05 m`
- No falls and no material tilt regression
- Person acquisition to safe command p95 below `0.4 s`
- No stale person-cloud rejection or sustained LiDAR frame replacement

Use `scripts/run_g1_fixed_command_test.sh` for repeatable forward, lateral, yaw, and
stop response measurements. Do not select a policy from training reward alone.
