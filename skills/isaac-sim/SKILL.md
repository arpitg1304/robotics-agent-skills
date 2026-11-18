---
name: isaac-sim
description: >
  NVIDIA Isaac Sim and Isaac Lab for robot learning: install (uv, pip, binary, Docker, cloud),
  RTX GPU and driver requirements, headless runs and livestreaming, OpenUSD assets, URDF/MJCF
  import, ArticulationCfg and actuators, manager-based and direct envs, configclass, gymnasium
  task registration, training and playing with rsl_rl, rl_games, skrl or sb3, parallel envs,
  tiled cameras, contact sensors, ray casters, IMU, frame transformers, domain randomisation via
  events, teleoperation, Isaac Lab Mimic, the ROS 2 bridge, PhysX/Newton presets, solver tuning
  and exploding articulations. Use when the user mentions Isaac Sim, Isaac Lab, Omniverse, Kit,
  isaacsim or isaaclab imports, omni.isaac modules, USD robots, URDF import, num_envs, Mimic,
  isaaclab train or play, ManagerBasedRLEnv, DirectRLEnv, PresetCfg, Newton or OvPhysX in Isaac
  Lab, or migrating from Isaac Gym or Isaac Lab 2.x.
---

# Isaac Sim and Isaac Lab

Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0, 2026-11-18. Not executed:
examples are checked against source, not run.

The 3.0 release is a break. Isaac Lab is now multi-backend: it installs **Newton** (Warp)
physics by default and treats Isaac Sim as an optional extra. Quaternions are XYZW, asset data
is a `ProxyArray`, and one `isaaclab train` command replaces the per-library scripts. Most
Isaac Lab code that agents learned before 2026 is wrong for 3.0. When a detail matters, trust
the installed checkout over memory, in this order: `--help` output, then source under
`source/`, then the docs under `docs/source/`.

An Isaac Lab checkout also ships NVIDIA-maintained agent skills in `.agents/skills/isaaclab-*`
(installing, migrating 2.x to 3.x, presets, sensors and actuators, events, multi-GPU). When
you work inside that repo, read the matching one. This skill is the portable overview.

## When to Use This Skill

- Installing Isaac Sim or Isaac Lab, or choosing between pip, binary, Docker and cloud
- Importing a URDF or MJCF robot, or writing an `ArticulationCfg` with actuators
- Writing a manager-based or direct RL environment and registering it with gymnasium
- Training or playing a policy with `isaaclab train` / `isaaclab play`
- Adding cameras, contact sensors, ray casters, IMUs or frame transformers to a task
- Domain randomisation, actuator models, observation noise and latency for sim-to-real
- Teleoperating, recording demonstrations and multiplying them with Isaac Lab Mimic
- Connecting Isaac Sim to ROS 2
- Fixing exploding articulations, slow training, black camera images or a broken upgrade

Out of scope: RL algorithm choice, reward design and sim-to-real theory
(`robot-reinforcement-learning`); policy architectures and BC/diffusion training
(`imitation-learning`); dataset format and QA (`lerobot`, `robot-data-quality`); MuJoCo and
MJX outside Isaac Lab (`mujoco`).

## Reference Files

Load these only when the task needs the detail:

- `references/install-and-setup.md`: non-uv installs, Docker gotchas, version pairing, first
  launch, visualizers and livestream.
- `references/assets-and-import.md`: USD, schema fragments, robot configs, importer gotchas.
  Load before converting or debugging an asset.
- `references/environment-examples.md`: full manager-based task with registration, direct
  skeleton. Load when writing an environment.
- `references/sensors-and-rendering.md`: sensor table, camera example, image memory. Load when
  adding a sensor or camera.
- `references/randomisation-and-actuators.md`: events example, PhysX vs Newton differences,
  actuator models, sim-to-real hooks.
- `references/debugging.md`: physics settings and the full symptom table. Load when a symptom is
  not in the short table below.
- `references/migration-2x-to-3x.md`: porting Isaac Lab 1.x/2.x, Isaac Gym or `omni.isaac.*`
  code; old-name to new-name tables; also when an error names a removed API.
- `references/config-reference.md`: config field tables, event functions, all train/play flags,
  training notes (logging, outputs, custom scripts, `num_envs`).
- `references/demonstrations-and-ros2.md`: teleop → Mimic → robomimic, HDF5 layout,
  GR00T/RLinf, ROS 2 bridge.

## Ecosystem Map and Requirements

| Component | Current (2026-11-18) | Notes |
|-----------|----------------------|-------|
| Isaac Sim | 6.2.0 (tag `v6.2.0`; `v7.0.0a1` alpha exists) | Omniverse Kit + OpenUSD app. Source on GitHub `isaac-sim/IsaacSim`, Apache-2.0; the build downloads the Kit SDK and extensions |
| Isaac Lab | 3.0.0 (tag `v3.0.0`, 2026-10-19) | BSD-3-Clause (Mimic is Apache-2.0). `main` targets Isaac Sim 6.2 and has already removed some 3.0 deprecated aliases |
| Newton | 1.6.1, on Warp 1.17 | Default Isaac Lab physics. Solvers: MJWarp, Kamino, VBD, MPM. Needs no Isaac Sim |
| OvPhysX / OVRTX | 0.6.4 / 0.5.1 | PhysX and RTX rendering without Kit. Cannot share a process with Isaac Sim |
| Python / PyTorch | 3.12 / torch 2.11.0 + CUDA 13.0 | CUDA 13 requires Turing or newer (compute 7.5+) |

Isaac Lab 3.0 rejects Isaac Sim 5.1 and older. Full version pairing:
`references/install-and-setup.md`.

**Requirements:**

- **OS:** Ubuntu 22.04+ x86_64, Linux aarch64 (DGX Spark), Windows 11 x86_64. **macOS is not
  supported** by Isaac Sim or Isaac Lab; a Mac can only edit code or drive a remote or cloud
  machine. Isaac Sim marks Ubuntu 22.04 as deprecated, and its pip wheels need GLIBC 2.35+.
  Below that (for example Ubuntu 20.04), use the downloaded binary package.
- **Driver:** Isaac Lab needs 580.65.06+ on Linux and 580.88+ on Windows, set by PyTorch
  CUDA 13. Isaac Sim 6.2 was tested with 595.58.03 (Linux) and 595.97 (Windows).
- **GPU for Isaac Sim:** RTX with RT cores. The minimum is an RTX 4080 with 16 GB VRAM and
  32 GB RAM. **A100/H100 are not supported** for Isaac Sim (no RT cores). Headless Newton
  simulation without cameras needs no RTX hardware.
- **Disk/network** (first-launch downloads, EULA env var): `references/install-and-setup.md`.

Check what is actually installed before giving version-specific advice:

```bash
uv run python -c "import isaaclab; print(isaaclab.__version__)"
uv run --extra isaacsim python -c "from isaaclab.utils.version import get_isaac_sim_version; print(get_isaac_sim_version())"
uv pip show isaacsim isaaclab newton warp-lang torch | grep -E '^(Name|Version)'
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
```

### Three choices you make separately

```
physics=   isaacsim_physx | physx (auto) | ovphysx | newton_mjwarp | newton_kamino ...
renderer=  newton_renderer | ovrtx | isaacsim_rtx | rtx (auto)        (camera tasks only)
--viz      none | newton_gl | newton_rtx | kit | rerun | viser       (what you look at)
```

Kit (Isaac Sim) is loaded only when something needs it: `physics=isaacsim_physx`,
`renderer=isaacsim_rtx`, `--viz kit`, livestreaming, or a direct `isaacsim.*` import. Defaults
are **per task**. Cartpole defaults to `newton_mjwarp`, and other tasks may default to
PhysX. Always list what a task supports:

```bash
uv run isaaclab list_envs --show_presets --keyword Cartpole
uv run isaaclab train --task Isaac-Cartpole --help   # groups physics= / renderer= / presets=
```

## Setup

### Recommended: uv from a checkout

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
git clone https://github.com/isaac-sim/IsaacLab.git && cd IsaacLab
git checkout v3.0.0                                  # pin; main moves daily

uv run isaaclab train --task Isaac-Cartpole --num_envs 16 --viz newton_gl physics=newton_mjwarp
uv run --extra isaacsim isaaclab train --task Isaac-Cartpole-Direct physics=isaacsim_physx
uv run --extra ovphysx  isaaclab train --task Isaac-Cartpole-Direct physics=ovphysx
```

`uv run` syncs the environment on every call. Put `--extra <name>` **before** `isaaclab`. The
extras are `isaacsim`, `ov`/`ovphysx`/`ovrtx`, `rsl-rl`, `rl-games`, `skrl`, `skrl-jax`,
`sb3`, `rlinf`, `torchrl`, `rerun`, `viser`, `mimic`, `teleop`, `video`, `importers`,
`tetrahedralization` and `leapp`. `--extra all` adds the curated `ov`, RL libraries, `rerun`
and `viser`, but **not** `isaacsim`, `mimic` or `teleop`.

Other install paths and Docker gotchas: `references/install-and-setup.md`.

### Verify, smallest layer first

```bash
uv run python -c "import isaaclab; print('ok')"                          # import
uv run isaaclab zero_agent --task Isaac-Cartpole --num_envs 4             # task builds and steps
uv run isaaclab train --task Isaac-Cartpole --max_iterations 1            # runner plumbing
uv run --extra isaacsim isaaclab -p scripts/tutorials/00_sim/create_empty.py --viz kit   # Kit opens
```

### Headless, visualizers, livestream

- **In 3.0, no `--viz` means headless.** `--headless` and `--enable_cameras` were **removed**:
  the 3.0 parser rejects them (see `test_kit_launcher.py`), and cameras enable themselves. A few
  3.0 doc pages still show these flags, so trust `--help`.

Visualizers and livestream: `references/install-and-setup.md`.

## Assets and Import

USD, schema fragments and SI units/scale: `references/assets-and-import.md`.

### Robot config

`rigid_props` and `articulation_props` are the schema-fragment lists from
`references/assets-and-import.md`.

```python
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

MY_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/assets/my_arm/my_arm.usda",
        activate_contact_sensors=True,          # required for any ContactSensor on this robot
        rigid_props=rigid_props,
        articulation_props=articulation_props,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),               # XYZW in 3.0 (was WXYZ in 2.x)
        joint_pos={"joint[1-6]": 0.0, "finger_.*": 0.04},   # regex keys, radians/meters
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-6]"],
            stiffness=400.0, damping=40.0, armature=0.01,
            joint_effort_limit=87.0,             # solver clamp (was effort_limit_sim)
            joint_velocity_limit=2.6,            # (was velocity_limit_sim)
        ),
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_.*"], stiffness=2e3, damping=1e2),
    },
    soft_joint_pos_limit_factor=0.95,
)
# in a scene: robot = MY_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

### URDF and MJCF import

Convert once, commit the USD, and spawn with `UsdFileCfg`. `UrdfFileCfg` and `MjcfFileCfg` can
convert at spawn time, but that hides conversion errors and costs startup time.

```bash
uv run --extra importers python scripts/tools/convert_urdf.py robot.urdf /out/dir \
    --fix-base --merge-joints --joint-stiffness 100.0 --joint-damping 1.0 --joint-target-type position \
    --ros_package_path my_pkg /path/to/my_pkg --viz newton_gl      # NAME PATH; repeatable
uv run --extra importers python scripts/tools/convert_mjcf.py robot.xml /out/robot.usd \
    --merge-mesh --self-collision --collision-type "Convex Hull"
```

Ready-made robot configs, importer gotchas and mesh conversion: load
`references/assets-and-import.md` before converting.

## Environment Design

| | Manager-based (`ManagerBasedRLEnv`) | Direct (`DirectRLEnv`) |
|---|---|---|
| Define | Config classes of terms: observations, actions, rewards, terminations, events, commands, curriculum | One class with `_pre_physics_step`, `_apply_action`, `_get_observations`, `_get_rewards`, `_get_dones`, `_reset_idx` |
| Best for | Most new tasks, ablations, Hydra overrides of single terms, sharing MDP terms | Custom step logic, fused reward kernels, MARL (`DirectMARLEnv`), Isaac Gym ports |
| Randomisation | `events` | `events` config also works in direct envs |

Both are driven by a `@configclass` config (a dataclass with mutable defaults handled for you;
use `cfg.replace(...)` or `isaaclab.utils.replace(cfg, ...)` to copy with changes). The scene is
declared on `cfg.scene` and accessed as `env.scene["name"]` in both workflows.

A complete manager-based task, its `gym.register` call and `isaaclab.tasks` entry point, and
a direct skeleton with the 3.0 `write_*_to_sim_index` and joint-target APIs: load
`references/environment-examples.md`.

Rules that example encodes:

- `sim.dt * decimation` is the control period. Set `render_interval = decimation` so cameras
  and visualizers don't render every physics step.
- Mark time limits `time_out=True` so they are truncations, not failures. The RL side is in
  `robot-reinforcement-learning`.
- Every term function returns a `(num_envs,)` or `(num_envs, k)` torch tensor on `env.device`.
  No Python loops over envs and no `.item()` in the step path, because each sync stalls the GPU.
- Resolve joint/body names once through `SceneEntityCfg(..., joint_names=...)`. Do not
  hard-code indices, because joint order can differ between PhysX and Newton.

## Training and Playing

```bash
uv run isaaclab train --task Isaac-Velocity-Rough-AnymalD --rl_library rsl_rl \
    --num_envs 4096 --seed 42 --max_iterations 1500                         # task default physics
uv run isaaclab play  --task Isaac-Velocity-Rough-AnymalD --checkpoint latest --num_envs 32 --viz newton_gl
uv run --extra isaacsim isaaclab train --task Isaac-Velocity-Rough-AnymalD physics=isaacsim_physx --viz kit
uv run isaaclab train --task Isaac-Cartpole --video --video_length 200 --video_interval 2000
uv run isaaclab train --task Isaac-Cartpole env.scene.num_envs=1024 env.sim.dt=0.005 agent.seed=7
uv run isaaclab train_multigpu --task Isaac-Cartpole                         # experimental
uv run isaaclab benchmark runtime --task Isaac-Cartpole                     # env-step throughput
```

- `--rl_library` takes `rsl_rl` (default for most tasks), `rl_games`, `skrl`, `sb3`, `rlinf` or
  `torchrl`. `rsl_rl` ships in the default uv env. The others need their extra
  (`--extra rl-games`, `skrl`, `sb3`, ...), and the task must register a `<lib>_cfg_entry_point`.
- `--checkpoint` takes a path, `latest`, `best` or `pretrained`. It replaces 2.x
  `--resume --load_run`. Keep the same `presets=` when loading, because an observation preset
  changes tensor shapes.
- Hydra overrides use `env.<path>=` and `agent.<path>=` (rl_games uses `agent.params.seed`).
  Overrides do **not** re-run `__post_init__`. If you change `env.sim.dt` or `env.decimation`,
  also override `env.sim.render_interval` and every sensor `update_period` derived from them.
- Outputs, TensorBoard/W&B, custom scripts, `num_envs` sizing: `references/config-reference.md`.

## Sensors and Rendering

Renderers: **Newton Warp** (`renderer=newton_renderer`) is fastest and lightest, with direct
lighting and no motion vectors. **OVRTX** (`renderer=ovrtx`) is the recommended kit-less RTX
path. **Isaac RTX** (`renderer=isaacsim_rtx`) is legacy, for runs that must stay in Isaac Sim.

Sensors, camera example, image memory and conventions: `references/sensors-and-rendering.md`.

## Randomisation and Actuators

Event modes (`EventTermCfg.mode`): `prestartup` authors USD before sim start (scale, some
materials). `startup` runs once after sim start (mass, friction buckets). `reset` runs per
episode. `interval` runs during an episode, with `interval_range_s`.

Events example, backend differences, actuator models (including the 3.0 double effort
clip), sim-to-real hooks: `references/randomisation-and-actuators.md`.

Which parameters to randomise, by how much, and why: `robot-reinforcement-learning`.

## Demonstrations and Imitation Learning

- **Mimic needs task-space (end-effector) actions.** It replays subtask segments under rigid
  transforms relative to a reference object, so convert joint-space data with FK first.
  Mimic task IDs still end in `-v0`.

Commands, HDF5 format, GR00T: `references/demonstrations-and-ros2.md`. Dataset QA:
`robot-data-quality`; conversion: `lerobot`; policy choice: `imitation-learning`.

## ROS 2 Bridge

Supported distros (Jazzy on 24.04, Humble on 22.04), DDS, OmniGraph, `/clock` and policy
deployment: `references/demonstrations-and-ros2.md`. ROS side: `ros2`, `docker-ros2-development`.

## Performance and Debugging

Physics settings and the full debug table: `references/debugging.md`.

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `unrecognized arguments: --headless` (or `--enable_cameras`) | Removed in 3.0 | Omit `--viz` (headless is the default), or use `--viz none`. Cameras enable themselves |
| Robot spawns rotated or upside down after an upgrade; goals wrong | WXYZ quaternions in 2.x code (3.0 is XYZW), or a USD `Gf.Quat` (real-first) passed straight in | `python scripts/tools/find_quaternions.py --path my_project/` (never on Isaac Lab itself), `--fix-identity-only` first, review the rest. `WARN_ON_TORCH_QUATF_ACCESS=1` flags runtime reads |
| `'ProxyArray' object has no attribute 'clone'` / `detach` | 3.0 data properties are `ProxyArray` | Use `.torch` (or `.warp`) |
| Articulation explodes or NaNs on the first steps | Spawn overlap, joint positions outside limits, zero/tiny link inertia, stiff explicit PD at large dt | Clamp resets to soft limits, fix inertias, use implicit PD or smaller dt, add armature, raise position iterations |
| Robot sags or ignores targets | Zero drive gains from import (`target_type none`), effort limit too low, wrong action term | Set actuator stiffness/damping, `joint_effort_limit >= actuator_effort_limit`, check the action term's joint regex |
| Contact sensor reads zeros | `activate_contact_sensors=False` on the spawner, or wrong body regex | Enable it on the spawner. Print `sensor.body_names` |
| Camera images black or garbage at scale | RTX atlas limit, wrong offset convention, no light | Smaller images or fewer cameras, or `newton_renderer`. Fix `convention`. Add a light. Inspect the last env |
| Training slow, low GPU use | Visualizer on, cameras rendering every physics step, too few envs, CPU device | `--viz none`, set `update_period`, `benchmark runtime` to size `num_envs`, `--device cuda:0` |

Debug order: reproduce with `--num_envs 1-4`, a fixed `--seed`, and `zero_agent`/`random_agent`.
Fix every `[PhysX]`/Newton warning before reading RL curves. Then profile with
`isaaclab benchmark` or Nsight Systems (`docs/source/how-to/profile_with_nsys.rst`). Kit logs
are written to the `kit_<timestamp>.log` path printed at startup. Add `--info`/`--verbose` for
more detail.

## Anti-Patterns

- **Copying 2.x snippets into 3.0.** That means `--headless`,
  `scripts/reinforcement_learning/rsl_rl/train.py`, `-v0` task IDs on core tasks, WXYZ
  quaternions, and `.clone()` on data. Grep for them after any migration.
- **Assuming Isaac Lab needs Isaac Sim.** In 3.0 the default install is Newton-only. Adding
  `--extra isaacsim` everywhere costs startup time and RTX requirements you may not need.
- **Assuming one physics backend.** Defaults are per task. Benchmark numbers and tuned gains
  from PhysX don't carry over to Newton unchanged.
- **Tuning RL before the asset is right.** Bad inertias, colliders and gains look like reward
  problems. Run `zero_agent` and look first.
- **Per-env Python loops, `.item()`, `.cpu()` in rewards or observations.** Each one stalls the
  GPU pipeline every step.
- **Rendering cameras at the physics rate** and requesting buffers the policy never reads.
- **Hard-coded joint indices, or time limits marked as failures.** Use `SceneEntityCfg` names
  and `time_out=True`.
- **Randomisation hidden in reward functions or the training script.** Put it in `events` so
  it is visible and overridable.
- **Mixing a binary Isaac Sim with conda/uv envs,** or OVRTX/OvPhysX with Kit in one process.
- **Training against `main` without pinning a tag.** APIs change weekly.

## Checklist

- [ ] Versions recorded: Isaac Lab tag/commit, Isaac Sim, Newton/Warp, torch, driver, GPU
- [ ] Platform supported (Linux/Windows 11, RTX for Isaac Sim, driver at least the minimum). Not macOS
- [ ] Asset converted to USD and inspected: scale, mass/inertia, colliders, joint limits, fixed base
- [ ] Actuator config covers every actuated joint, with effort/velocity limits and armature set
- [ ] All hard-coded quaternions are XYZW. Data reads use `.torch`
- [ ] `sim.dt`, `decimation`, `render_interval` and sensor `update_period` are consistent
- [ ] Physics backend chosen on purpose (`physics=`) and listed in `list_envs --show_presets`
- [ ] `zero_agent` and `random_agent` run warning-free at 4 envs, then at full `num_envs`
- [ ] Contact, camera and ray-caster outputs inspected for env 0 **and** the last env
- [ ] Randomisation in `events` with conservative ranges. Time-outs marked `time_out=True`
- [ ] Training run reproducible: seed, presets, Hydra overrides and the command line saved with the logs
- [ ] Policy exported (`exported/policy.onnx`) and checked in a second backend or a held-out randomisation
- [ ] For demos: task-space actions, annotations checked, Mimic success ratio reviewed, dataset QA passed

## Related Skills

- `robot-reinforcement-learning`: algorithms, reward design, randomisation strategy, sim-to-real theory
- `imitation-learning`: BC, diffusion and VLA policies trained on Isaac Lab or Mimic data
- `lerobot`: LeRobot dataset format, conversion and training stack
- `robot-data-quality`: validating demonstration and rollout datasets before training
- `mujoco`: MuJoCo/MJX/MJWarp outside Isaac Lab, and comparison with Newton's MJWarp solver
- `ros2`, `docker-ros2-development`: the ROS 2 side of the bridge and containerised deployment
- `robotics-testing`: CI smoke tests for environments and exported policies
- `robot-perception`: camera models and perception pipelines for sim-rendered images
