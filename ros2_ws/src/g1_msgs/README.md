# g1_msgs

The stack's own interfaces: what `g1_locomotion` serves for the base approach, and what
manipulation serves to the behavior tree. Five actions, no messages.

```bash
colcon build --symlink-install --packages-select g1_msgs
```

## Interfaces

| Action | Goal |
|---|---|
| `ApproachObject` | `object_id`, `arm`, `working_yaw`, `use_current_heading`, `timeout_s` |
| `Retreat` | `distance_m`, `timeout_s` |
| `Pick` | `object_id`, `arm` |
| `Place` | `surface_object_id` or `pose`, `arm` |
| `SetArmPosture` | `group`, `named_target` |

All five are actions rather than services because each runs for seconds and has to be cancellable.

Ported from Grove-G1 commit `97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681` under the
BSD 3-Clause license. See `NOTICE` for attribution and license terms.
