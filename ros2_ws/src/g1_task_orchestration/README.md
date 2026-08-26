# G1 Task Orchestration

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
