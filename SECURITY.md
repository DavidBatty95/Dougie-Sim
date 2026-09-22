# Security and deployment scope

The supported target is the documented local MuJoCo simulation environment.
No hardware deployment or independent safety certification is claimed.

- Keep the simulation DDS interface on the configured local interface/domain.
  Do not connect tutorial command publishers to a live robot network.
- `.env`, credentials and private checkpoints are local files. Do not attach
  them to public issues or logs.
- The development container uses host networking/IPC for ROS integration and
  explicitly mapped graphics devices. It does not require `privileged: true`.
  Review host mounts and X11 access before using shared machines.
- Report sensitive issues through the team's private security channel. The new
  repository owner should configure GitHub private vulnerability reporting and
  an organization-owned contact as part of the handoff.

Only the current maintained branch is supported. Third-party vulnerabilities
should also be reported to the corresponding upstream project.
