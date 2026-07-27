# ROS 2 and MuJoCo Integration Verification

## Safety Architecture

- Only the MuJoCo physics thread accesses the primary `mjModel` and `mjData` for
  Unitree commands and state capture.
- DDS communication exchanges fixed-size command and state snapshots with the
  physics thread.
- Low-level commands expire after 100 ms by default. Missing, stale, invalid,
  and pre-reset commands produce zero actuator control.
- Reset and model reload invalidate cached commands and require a newer DDS
  command sequence.
- LiDAR and camera raycasts use separate latest-only workers, private copied
  MuJoCo models, and private data snapshots.
- Policy inference uses a single-flight worker and a 100 ms default monotonic
  deadline. Errors and deadline misses publish a safe hold target.

## Automated Verification

Run on 2026-07-27 in the project container:

| Check | Result |
| --- | --- |
| MuJoCo release build | Passed |
| MuJoCo debug build | Passed |
| Fresh full ROS workspace build | 9 packages passed |
| C++ CTest | 3/3 passed |
| Bridge Python tests | 36 passed |
| Policy worker tests | 7 passed |
| Sensor packet tests | 4 passed |
| Python syntax and diff checks | Passed |

The C++ tests cover watchdog boundaries, reset sequence invalidation,
phase-locked scheduling, latest-only replacement, worker restart, and clean
worker shutdown. Python tests cover policy single-flight behavior, bounded
shutdown, errors, deadlines, packet versions, and exact acquisition timestamps.

ThreadSanitizer binaries compiled successfully, but this Docker runtime could
not start them and reported `ThreadSanitizer: unexpected memory mapping` before
test execution. Debug tests with assertions enabled passed. Run TSAN on a host
or CI runner that supports its shadow-memory layout before hardware use.

## Live Simulation Results

Matched 10-second steady-state intervals used the hold controller and no
RViz/Nav2/SLAM:

| Configuration | Low-state packets | Approximate rate |
| --- | ---: | ---: |
| LiDAR/camera/pedestrians disabled | 6,163 | 616 Hz |
| LiDAR/camera/pedestrians enabled | 5,646 | 565 Hz |

Enabling all simulated sensors reduced observed low-state throughput by 8.4%,
within the 10% acceptance threshold. The enabled run delivered 105 LiDAR
packets with zero stale packet drops.

The initial asynchronous implementation still allocated an `mjData` snapshot
for every sensor frame. After replacing those allocations with a reusable
three-slot pool, a repeated sensor-enabled interval delivered 8,430 low-state
packets, approximately 843 Hz. Absolute rates vary with host scheduling, but
the pooled run removed the measured sensor throughput penalty and delivered
130 LiDAR packets while keeping the worker queue bounded.

A longer sensor-enabled reset test processed three asynchronous resets without
a crash. Low-state publication continued beyond 20,000 packets. The LiDAR
worker reported 252 submitted jobs, zero replacements, 251 completed jobs, and
53.4 ms mean worker time. This work is no longer part of the physics step.

## Remaining Limits

- Automated UI model drag/drop reload was not available in the headless test.
  The reload path builds and no longer leaves the bridge holding MuJoCo
  pointers, but repeated interactive reload should still be tested.
- The neutral hold controller did not maintain an upright G1 in these tests.
  This is controller/policy tuning, not a bridge race, but it prevents claiming
  improved locomotion stability from integration changes alone.
- Hardware deployment still requires torque/current limits, an emergency stop,
  liveliness monitoring, and controlled test-stand validation.
