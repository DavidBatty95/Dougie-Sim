# Native Apple Silicon demo and Ubuntu desktop

Install Docker Desktop for Apple Silicon, then from a checkout run:

```bash
./demo
```

The launcher starts Docker Desktop if needed, builds the native ARM64 Ubuntu
22.04 / ROS 2 Humble / MuJoCo / CPU PyTorch image if missing, verifies/downloads
the pinned model and policy, starts the comparison, waits for its HTTP health
check, and opens <http://localhost:8770>. Everything lives in this checkout;
no second repository or hard-coded user directory is required.

The default comparison has no cumulative dose cap or time cap. Radiation-aware
Dougie uses predicted dose as a route cost; the blind ablation ignores it. Both
use simulator ground-truth pose to isolate planning from localization limits.
The dashboard labels these settings and includes truth/GP maps. Completion is
zero observed-map frontier nodes on three fresh scans, not a coverage percentage.

## Lifecycle and configuration

```bash
./dougie-mac compare-status
./dougie-mac compare-logs --tail 50
./dougie-mac compare-stop
# Reopen a completed experiment without rerunning physics:
./dougie-mac replay results/dougie/comparison_<timestamp>
# Launch a new experiment after stopping the previous one:
DOUGIE_COMPARE_WORLD=facility DOUGIE_COMPARE_SEED=3 ./demo
DOUGIE_COMPARE_LOCALIZATION=fused ./demo
DOUGIE_COMPARE_DURATION=600 DOUGIE_COMPARE_FPS=5 ./demo
```

Other environment options are `DOUGIE_COMPARE_PORT` (8770 by default) and
`DOUGIE_COMPARE_RADIATION_POLICY` (`minimize-dose` or `rate-limited`). Stop before
changing configuration. Repeated starts reuse the existing dashboard; completed
graphs remain available until stopped. Results are saved directly under
`results/dougie/comparison_*/` in the checkout. `compare-stop` does not delete them.

For a numeric cumulative cap, run the module with `--dose-budget` through Compose
or use the Linux `./dougie compare --dose-budget ...` command. See the
[comparison guide](DOUGIE_COMPARISON.md).

## Optional full Ubuntu desktop

```bash
./dougie-mac setup           # Build/update the image and start XFCE
./dougie-mac start demo      # Open the desktop and launch walking with RViz
./dougie-mac start radiation # Open a single-robot diagnostic radiation mission
./dougie-mac shell           # ROS-enabled Ubuntu shell
./dougie-mac down
```

The desktop is at <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>.
`/workspace` is this checkout, mounted read/write; results appear on both the Mac
and Ubuntu sides. The `dougie` user has passwordless sudo inside the container.
The home directory persists in a named Docker volume. Within Ubuntu, use
`dougie-desktop demo`, `dougie-desktop radiation`, or ordinary ROS commands.
Only run one ROS command-owning session per domain. The single-robot desktop
launch is a separate diagnostic profile with a 3600-s cap; the paired browser
demo has no cap. `DOUGIE_DESKTOP_PORT` changes the desktop's localhost port.

## Resources and graphics

Start with around 8 GB of Docker memory. Initial builds download several GB.
Standard Docker Desktop does not expose the Apple GPU to these Linux OpenGL
applications, so this profile uses Mesa CPU rendering (`llvmpipe`) without amd64
emulation. The paired feeds target 5 FPS; actual throughput depends on scene and
host load. RViz usually performs better than the direct interactive MuJoCo
viewer. No XQuartz is required. Only loopback HTTP ports are published; VNC stays
inside the desktop container.

After changing runtime requirements or the Dockerfile, rebuild:

```bash
docker compose -f docker-compose.mac.yml build
```

The comparison Compose file is standalone:
`docker compose -f docker-compose.compare.yml ...`. For Linux GPU/DRI launches,
use the original `./dougie` launcher and [graphics guide](DOUGIE_PERFORMANCE.md).
