# Security and deployment scope

<!-- geoff-nav:start -->
<p align="center">
  <a href="README.md"><img src="docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="README.md#quick-start"><img src="docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="docs/DEMO_GUIDE.md"><img src="docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="docs/INDEX.md"><img src="docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="docs/REPOSITORY_GUIDE.md"><img src="docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="docs/PROJECT_STATUS.md"><img src="docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="docs/PROJECT_PLAN.md"><img src="docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="docs/RESULTS_REVIEW.md"><img src="docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="docs/literature_review/README.md"><img src="docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="docs/DEVELOPMENT.md"><img src="docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

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
