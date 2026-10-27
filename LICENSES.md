# Licensing inventory

[Project home](README.md) · [Asset manifest](config/dougie_assets.json) · [Attribution](docs/DOUGIE_SIM.md#attribution)

Existing licenses remain authoritative. This cleanup does not assign a new
blanket root license or add a license grant to previously unlicensed original
scripts or documentation.

| Material | Existing declaration / authority |
|---|---|
| Go2 model and meshes | `unitreerobotics/unitree_mujoco`, BSD-3-Clause; pinned license downloaded with the assets |
| Go2 walking policy and observation contract | `fan-ziqi/rl_sar`, Apache-2.0; configuration and license downloaded beside the policy |
| MuJoCo, ROS 2, Nav2, SLAM Toolbox, NumPy, SciPy and PyTorch | Their upstream license notices |
| Optional MJLab/CUDA training dependencies | Their pinned distributions and upstream notices |
| Project code, documentation and generated evidence | Existing file notices; no additional blanket license grant |

Checksums establish artifact identity, not redistribution permission. Preserve
upstream copyright/license headers when modifying code or publishing derivatives.

The retired Geoff/G1 workspace and its package-specific notices remain together
in the [pre-cleanup history](docs/DOUGIE_HISTORY.md). They are no longer bundled
with the maintained Go2 checkout.
