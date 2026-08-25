# g1_locomotion

The step between navigation and manipulation. Nav2 parks the robot near an object and this package
closes the final gap into the arm's reach window.

One C++20 node, `g1_base_approach`, serves `~/approach_object` and `~/retreat`, consumes
`vision_msgs/msg/Detection3DArray` on `/objects`, and publishes `geometry_msgs/msg/Twist`.
`cmd_vel_topic` defaults to `/cmd_vel`, preserving this workspace's `/cmd_vel` to human-aware to
`/cmd_vel_safe` command pipeline.

Configuration is in `config/g1_base_approach.yaml`. The pure planner uses proportional commands
with measured linear speed floors, caps, simultaneous forward/lateral/yaw control, and a
recoverable reverse when the object is just past the reach window.

```bash
colcon build --symlink-install --packages-select g1_msgs g1_locomotion
colcon test --packages-select g1_msgs g1_locomotion
```

Ported from Grove-G1 commit `97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681` under the
BSD 3-Clause license. See `NOTICE` for attribution and license terms.
