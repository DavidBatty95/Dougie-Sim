# Reliable people-aware box transport: implementation plan

## End-state demonstration

The G1 repeatedly discovers a box on a table, selects its matching empty marked
square, approaches and picks it up, carries it through a furnished environment
with independently moving people, and places it on the correct square. The
results GUI reports each delivery and every failure, contact, drop and recovery.

**Current status:** navigation remains below acceptance; the last complete
published navigation cohort passed 4/18 strict trials. Subsequent troubleshooting
is in progress. The existing transport action server is an integration starting
point, not evidence that the complete mission works.

## Existing components and required changes

- Nav2, online SLAM, measured LiDAR tracking, local social planning and the final
  collision supervisor provide the navigation foundation.
- `g1_task_orchestration` already sequences navigation, approach, Pick/Place,
  retreat and arm authority. It currently uses base-pinning phases and expects
  supplied staging poses; these are not sufficient for the mobile acceptance run.
- `g1_manipulation` already offers MoveIt Pick/Place and named arm-posture actions.
- The arm compositor separates upper-body trajectories from locomotion commands.
- MuJoCo's existing grasp model activates a palm–object weld only inside a
  capture radius. It preserves the object's relative pose and carries its mass.
  This is a **simulation attachment model**, not demonstrated finger force closure.
- Existing `/objects` detections come from simulator state. They can support an
  explicitly labelled manipulation fixture, but final mission goals/perception
  must use measured RGB-D/LiDAR data and estimated transforms. Truth stays in
  scoring. The final demo must not pin the base or teleport boxes.
- The old transport scene has decorative, non-colliding tabletops and a single
  cube. Replace those with physical tables, real collision geometry, several
  matched source/destination pairs and measured object/surface perception.

## Milestones and acceptance gates

| Milestone | Implementation | Required evidence before completion |
| --- | --- | --- |
| M0: reproducible platform | Worlds, CLI, GUI, versioned datasets and source patches | Build/tests, browser check and honest baseline records; implemented in the existing milestone commit |
| M1: navigation reliability | Resolve arrival, command-response, route-progress, timing and avoidance failures | Every requested goal reached, zero contacts, valid sensor-only trials across the six-world/three-seed/four-person matrix; repeat on held-out seeds |
| M2: transport environments | Physical tables, lightweight boxes, colored/identified empty squares, furniture and clear aisles | All models load; every pairing is unique; boxes rest on supports; robot and loaded-footprint routes are connected |
| M3: verified manipulation | Measured target pose, observed-free docking pose, stable arm authority, grasp/lift/place verification | Repeated stationary pick/place for every object type, no pinning, no teleportation, no false-success attachment or wrong placement |
| M4: mobile transport | Shared navigation intent, precise docking, carry posture, payload-aware clearance and placement | Multiple complete deliveries with real physics in an empty-person control condition, without drops or falls |
| M5: dynamic missions | Integrate moving people, intrusion handling and bounded recovery | Complete all requested deliveries across representative worlds/seeds with zero person contacts, drops, wrong destinations or invalid sensor intervals |
| M6: release evidence | Updated GUI, JSON/CSV/replay, runbooks and convenience commands | Repeated acceptance runs from the published source snapshot and a clean-checkout build; commit and push verified milestones |

M1 remains a prerequisite for claiming M5. World construction and isolated
manipulation tests can proceed independently while navigation failures are fixed.
Acceptance thresholds must not be relaxed to make a failing run pass.

## Mission state machine

`DISCOVER → SELECT_PAIR → PLAN_STAGING → NAVIGATE_PICKUP → DOCK → VERIFY_CLEAR_WORKSPACE
→ ACQUIRE_ARM → PREGRASP → GRASP → VERIFY_HELD → LIFT → CARRY_POSTURE
→ NAVIGATE_DELIVERY → DOCK_DELIVERY → LOWER → RELEASE → VERIFY_PLACED
→ RETREAT → STOW → NEXT_PAIR → COMPLETE`

- Pair selection uses semantic IDs/colors and observed object/surface poses, not
  simulator coordinates. Initially use distinct red/blue/green box/pad pairs.
- Staging and docking poses must be reachable in observed free space. Keep the
  actual goal distinct from a planner's tolerated path endpoint.
- Share navigation intent with the social planner so task navigation, docking
  and holding cannot silently compete for `/cmd_vel` or suppress avoidance.
- Keep locomotion/balance active during manipulation. Use explicit arm ownership
  and a known carry posture; do not substitute a stationary hold controller for
  mobile walking.
- Before transport, update navigation and person-clearance geometry for the
  carried box. Restore the empty configuration only after confirmed release.
- A successful grasp requires actuator acknowledgement and held-state evidence.
  A successful delivery requires the correct box on the correct pad, released,
  supported and settled; an action server's success alone is insufficient.

## Failure handling

Retain the original mission deadline through retries. Handle stale detections,
blocked staging/docking, people entering the arm workspace, rejected grasps,
attachment loss, unstable posture, placement errors and action cancellation.
Retry only when the sensed physical state supports it. If attachment state is
uncertain, preserve arm authority and enter a recoverable stopped state rather
than silently releasing or declaring success. Every such transition is logged.

## World and perception design

Start with a furnished, wide-aisle room: two pairs of tables with a colored box
or a matching empty square. Expand to a workshop/lobby and the plaza, then to
narrower passages and occluding furniture. Table dimensions and approach
clearances must fit measured G1 reach and the loaded footprint.

Use original procedural assets. Begin measured perception with known-color
segmentation plus registered depth, geometric box/pad tests and stable temporal
IDs. Validate against simulator truth offline, including false detections on
people/furniture, occlusion and camera latency. Keep confidence/age gates explicit.

## Data and usability

Extend `./geoff` with transport-world listing, a one-command transport demo,
scored transport batches and results viewing. Record source/policy/world hashes,
seed, people count, object pairs, phase times, navigation goals, arm actions,
grasp/release acknowledgements, payload state, contacts, drops, placement errors,
timeouts and recovery reasons. Keep oracle fixtures, visual sessions and scored
sensor-only missions in separate datasets.

Each milestone is committed and pushed after its stated checks, with failures
and remaining gates documented. A single successful run does not establish
reliability; the release report must state exactly which matrix passed.
