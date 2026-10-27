# Security and deployment scope

[Project home](README.md) · [Container setup](docker/README.md)

The supported target is the documented local Go2/MuJoCo simulation environment.
No hardware deployment or independent safety certification is claimed.

- Keep ROS DDS on the configured local interface/domain. The browser comparison
  binds to localhost; ordinary ROS sessions use domain 43 by default.
- Do not connect simulation command publishers to a live robot network.
- Keep credentials, `.env` files and private checkpoints out of public issues,
  logs and commits.
- The container uses host networking for local ROS integration and explicitly
  mapped graphics devices. It does not require `privileged: true`.
- Report sensitive issues privately to the maintainers using GitHub private
  vulnerability reporting where available. Report third-party vulnerabilities
  to the corresponding upstream project as well.

Only the maintained Dougie profile is supported. Retired profiles are available
as historical source, not maintained deployment configurations.
