# G1 Task Orchestration

[Workspace map](../README.md) · [Manipulation](../g1_manipulation/README.md) · [Project plan](../../../docs/PROJECT_PLAN.md)

`g1_transport_server` sequences the existing Nav2, base-approach, manipulation,
and arm-compositor interfaces. It does not publish velocity, joint, or LowCmd
commands.

The transport action is `/g1_transport_server/transport_object`. A mobile
bringup must provide Nav2, `/objects`, `g1_base_approach`, MoveIt,
`g1_manipulation_server`, and the arm compositor. Keep the compositor and policy
controller enabled, and do not run the stationary hold controller during a
transport mission.

The server rejects goals unless `allow_simulation_manipulation` is explicitly
enabled. This is not a hardware manipulation interface.

If a mission fails after Pick starts, the result conservatively reports that
the object may still be held and arm authority remains active. Operator or
higher-level recovery must establish the physical state before releasing it,
then call `/g1_transport_server/clear_recovery_latch` before another mission.

## Place in the project

This is the earlier action-based integration path. The newer experimental
RGB-D transport workflow is implemented in the bridge package and launched by
`scripts/run_transport_demo.py`. Its magnetic-fixture passes are documented in
the [payload report](../../../docs/PAYLOAD_POLICY_TRAINING.md). They should not
be interpreted as end-to-end validation of this orchestration package.

## Folder guide

| Path | Responsibility |
| --- | --- |
| [`g1_task_orchestration/`](g1_task_orchestration/) | Transport state sequencing and action ownership |
| [`config/`](config/) | Mission parameters and simulation opt-in |
| [`launch/`](launch/) | Server composition |
| [`test/`](test/) | Sequencing and failure-contract regressions |

## Development workflow

Read the [`TransportObject` action](../g1_msgs/action/TransportObject.action)
before writing a client. Check all prerequisites above and ensure command
authority is unambiguous. Package tests run in the supported ROS container:

```bash
colcon test --packages-select g1_task_orchestration
```

The [S4 plan](../../../docs/PROJECT_PLAN.md) calls for a consolidated, persistent
mission lifecycle with bounded retry/recovery and verified completion. Keep
uncertain possession, cancellation and arm-authority behavior explicit when
changing sequencing; a failed action is not proof that the object is released.
