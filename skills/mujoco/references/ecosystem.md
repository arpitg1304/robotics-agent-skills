# MuJoCo Ecosystem (MuJoCo 4.0.1, 2026-11-18)

Full component table and Menagerie model list, moved from SKILL.md.

## Components

| Component | Package / import | Status (2026-11-18) | Use it for |
|---|---|---|---|
| MuJoCo core | `pip install mujoco` (4.0.1), `import mujoco` | C engine + official bindings. Since 4.0 `mujoco` is a metapackage: `mujoco-core` (numpy only) + `mujoco-studio` + `glfw`/`pyopengl`. Python >= 3.11 | Single-env sim, control, MPC, teleop, rendering, data generation |
| Headless core | `pip install mujoco-core` (`[rendering]` for classic GL) | Same bindings, no Studio | Servers, RL workers, containers |
| CPU batch | `from mujoco import batch, rollout` | Multi-threaded, GIL released, float64, all solvers/plugins | Thousands of CPU rollouts, per-sim model fields, sysid |
| MJX | `pip install mujoco-mjx`, `from mujoco import mjx` | Two implementations: `impl="jax"` (default; any XLA device; differentiable) and `impl="warp"` (NVIDIA; not differentiable) | JAX pipelines, gradients through physics |
| MuJoCo Warp | `pip install mujoco-warp`, `import mujoco_warp as mjw` | NVIDIA GPU (CPU device for debugging). Maintained by Google DeepMind + NVIDIA as part of Newton. Not differentiable | Highest-throughput GPU batches, PyTorch training |
| MuJoCo Playground | `pip install playground` (1.0.1) | Rewritten as **PyTorch** tasks on `reaf-sim` + MuJoCo Warp, RSL-RL training, real-robot deploy (SO-101, YAM, Trossen WXAI, G1). The original JAX/MJX envs live in the repo's `rss2025/` directory | Ready-made envs, sysid and deploy examples |
| Menagerie | git clone, or `pip install mujoco-menagerie` (`import mujoco_menagerie as mm`) | Curated MJCF robots | Starting models; never hand-write a common arm |
| dm_control | `pip install dm_control` (1.0.52 requires `mujoco>=4.0.1`) | Maintained; Control Suite + PyMJCF | Existing dm_control code. For new procedural models use `mujoco.MjSpec`, not PyMJCF |
| Gymnasium | `pip install "gymnasium[mujoco]"` (1.4.0) | `*-v5` envs run on MuJoCo 4.0.1; `-v2`/`-v3` IDs raise a mujoco-py error | Classic locomotion benchmarks |
| Viewers | `mujoco.viewer` (managed/passive), MuJoCo Studio (`python -m mujoco.studio.viewer --model=x.xml`, `--gfx=web`), live.mujoco.org | Studio is the successor of `simulate`; `mujoco.viewer` still ships | Interactive debugging |

## Menagerie models

Menagerie arms relevant to low-cost manipulation (directory / main file):
`robotstudio_so101/so101.xml` (SO-101, 6 position actuators, `scene.xml`, `scene_box.xml`),
`trs_so_arm100/so_arm100.xml` (SO-ARM100 5-DoF), `low_cost_robot_arm/`, `i2rt_yam/`,
`trossen_wxai/`, `trossen_vx300s/`, `trossen_wx250s/`, `aloha/`, `agilex_piper/`,
`arx_l5/`. Industrial: `franka_emika_panda/`, `franka_fr3/`, `universal_robots_ur5e/`,
`kuka_iiwa_14/`, `kinova_gen3/`, `ufactory_xarm7/`. Some models ship `*_mjx.xml` variants
tuned for MJX. Each README states the minimum MuJoCo version and the derivation steps.
