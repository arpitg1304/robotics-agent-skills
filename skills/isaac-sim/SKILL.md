---
name: isaac-sim
description: >
  NVIDIA Isaac Sim and Isaac Lab for robot learning: install (uv, pip, binary, Docker, cloud),
  RTX GPU and driver requirements, headless runs and livestreaming, OpenUSD assets, URDF/MJCF
  import, ArticulationCfg and actuator models, manager-based and direct environments,
  configclass, gymnasium task registration, training and playing with rsl_rl, rl_games, skrl or
  sb3, thousands of parallel envs, tiled cameras and renderers, contact sensors, ray casters,
  IMU, frame transformers, domain randomisation through events, teleoperation, Isaac Lab Mimic
  demonstration generation, the ROS 2 bridge, PhysX/Newton backend presets, solver tuning and
  exploding articulations. Use this skill when the user mentions Isaac Sim, Isaac Lab,
  Omniverse, Kit, isaacsim or isaaclab imports, omni.isaac modules, USD robots, URDF import,
  parallel envs, num_envs, tiled cameras, Mimic, isaaclab train or play, ManagerBasedRLEnv,
  DirectRLEnv, PresetCfg, Newton or OvPhysX in Isaac Lab, or migrating from Isaac Gym or
  Isaac Lab 2.x.
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

- `references/migration-2x-to-3x.md`: porting Isaac Lab 1.x/2.x, Isaac Gym or `omni.isaac.*`
  code; old-name to new-name tables; also when an error names a removed API.
- `references/config-reference.md`: field tables for `SimulationCfg`, `PhysxCfg`, scene and env
  configs, actuators, sensors, event functions, observation noise and delay, and the full
  train/play CLI flag list.
- `references/demonstrations-and-ros2.md`: the full teleop → Mimic → robomimic pipeline, what
  Mimic assumes, the HDF5 layout, GR00T/RLinf workflows, and ROS 2 bridge setup (distros,
  bundled libraries, DDS, OmniGraph nodes).

## Ecosystem Map and Requirements

| Component | Current (2026-11-18) | Notes |
|-----------|----------------------|-------|
| Isaac Sim | 6.2.0 (tag `v6.2.0`; `v7.0.0a1` alpha exists) | Omniverse Kit + OpenUSD app. Source on GitHub `isaac-sim/IsaacSim`, Apache-2.0; the build downloads the Kit SDK and extensions |
| Isaac Lab | 3.0.0 (tag `v3.0.0`, 2026-10-19) | BSD-3-Clause (Mimic is Apache-2.0). `main` targets Isaac Sim 6.2 and has already removed some 3.0 deprecated aliases |
| Newton | 1.6.1, on Warp 1.17 | Default Isaac Lab physics. Solvers: MJWarp, Kamino, VBD, MPM. Needs no Isaac Sim |
| OvPhysX / OVRTX | 0.6.4 / 0.5.1 | PhysX and RTX rendering without Kit. Cannot share a process with Isaac Sim |
| Python / PyTorch | 3.12 / torch 2.11.0 + CUDA 13.0 | CUDA 13 requires Turing or newer (compute 7.5+) |

Isaac Lab to Isaac Sim compatibility (from the Isaac Lab README):

- `v3.0.0`, `release/3.0.0` and `main` pair with 6.2. `v3.0.0-EA` and `-beta2` pair with 6.1
  and 6.0.
- `v2.3.x` pairs with 4.5/5.0/5.1, and `v2.0`-`v2.2` with 4.5 (2.2 also 5.0).
- Isaac Lab 3.0 rejects Isaac Sim 5.1 and older.

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
- **Disk/network:** the first Isaac Sim launch downloads extensions and compiles shaders, which
  can take more than 10 minutes. Set `OMNI_KIT_ACCEPT_EULA=yes` for non-interactive runs.

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

### Other install paths

| Path | When | Key commands |
|------|------|--------------|
| `isaaclab.sh -i` (legacy) | You manage the venv/conda env yourself | `./isaaclab.sh -i` (core + mimic + teleop + RL); `-i 'isaacsim'`, `-i 'ov[ovrtx]'` |
| pip Isaac Sim in your env | Editable checkout plus full Isaac Sim | `uv pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match --prerelease=allow`, then `./isaaclab.sh -i` |
| Isaac Lab wheel | Isaac Lab as a dependency of your project | `uv add --index-strategy unsafe-best-match --index nvidia=https://pypi.nvidia.com isaaclab==3.0.0`. Wheels exist only for major releases. The wheel has no `rl-games` and no robomimic |
| Binary Isaac Sim | Old GLIBC, or VS Code debugging inside Isaac Sim | Extract to `~/isaacsim`, `ln -s ~/isaacsim _isaac_sim`, `./isaaclab.sh -i`. Use the bundled `python.sh` only, never mixed with conda/uv |
| Docker | Clusters, CI, reproducibility | `./docker/container.py start [base\|ros2\|kitless]`, then `enter`, `copy`, `stop`. Prebuilt `nvcr.io/nvidia/isaac-lab:3.0.0` and `:3.0.0-kitless` |
| Cloud | No local RTX GPU (for example on a Mac) | Isaac Automator: `./run ./deploy-aws` (also `-gcp`, `-azure`, `-alicloud`) |

Docker gotchas: `container.py stop` deletes the named volumes, so run `copy` first. Images run
as uid/gid 1000, so `chown 1000:1000` bind-mounted cache dirs. Rendering needs
`--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all`.

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
- `--viz kit,newton_gl` opens several visualizers. `--viz none` overrides any configured in code.
- Livestream: `--livestream 1` (public WebRTC) or `2` (LAN), or set the `LIVESTREAM` env var.
  This forces the Kit visualizer and binds TCP 49100. An `NVST_R_BUSY` error means a stale
  process still holds that port.
- `--viz viser` serves a web UI on port 8080. It is the easiest remote view without Kit.
- Isaac Sim alone (no Isaac Lab) runs headless with `SimulationApp({"headless": True})`. The
  binary package's WebRTC launcher is `isaac-sim.streaming.sh`.

## Assets and Import

### USD essentials

- A **stage** is a tree of **prims** at paths like `/World/envs/env_0/Robot/base_link`. Isaac
  Lab clones `env_0` to `env_N`, and `{ENV_REGEX_NS}` in a `prim_path` expands to
  `/World/envs/env_.*`.
- **References/payloads** pull one USD into another. **Variant sets** select alternatives
  inside an asset, for example `variants={"Physics": "physx"}` on `UsdFileCfg`. **Instanceable**
  prims share one mesh in memory, and the importers produce instanceable assets by default.
- Physics comes from **applied schemas**: `RigidBodyAPI`, `CollisionAPI`, `MassAPI`,
  `ArticulationRootAPI`, joint `DriveAPI`, plus `physx*` and `newton*` namespaces. Isaac Lab
  authors them through schema **fragments**, one configclass per USD namespace, passed as a
  list:

```python
from isaaclab.sim.schemas import UsdPhysicsRigidBodyCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg
from isaaclab_newton.sim.schemas import NewtonArticulationCfg

rigid_props = [UsdPhysicsRigidBodyCfg(kinematic_enabled=False), PhysxRigidBodyCfg(max_depenetration_velocity=5.0)]
articulation_props = [
    PhysxArticulationCfg(enabled_self_collisions=False, solver_position_iteration_count=8,
                         solver_velocity_iteration_count=0),
    NewtonArticulationCfg(self_collision_enabled=False),
]
```

The 2.x `*PropertiesCfg` classes still work with a `DeprecationWarning` and are removed in 3.2.
Some universal properties (`disable_gravity`, `max_joint_velocity`, contact/rest offsets) only
exist in the `physx*` namespace, and Newton reads them from there too. So a Newton-only asset
still uses the PhysX fragment for those fields.

- Units are SI: meters, kilograms, seconds, radians, Z-up. A CAD mesh in millimeters spawns
  1000x too large. Fix the mesh, or set `scale=` on the spawner, and then re-check the mass.

### Robot config

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

`stiffness=None`/`damping=None` keep the USD-authored gains. Ready-made configs live in
`isaaclab_assets.robots` (`FRANKA_PANDA_CFG`, `UR10_CFG`, `ANYMAL_D_CFG`, `UNITREE_GO2_CFG`,
`G1_CFG`, `H1_CFG`, `SPOT_CFG`, `SO101_CFG` and others). Read the matching file before you copy
one.

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

The `importers` extra converts without Isaac Sim. Add `--extra isaacsim` and `--viz kit` to
preview in the Isaac Sim viewport. Importer gotchas in 3.0:

- **Output path is chosen for you.** The URDF output is `{usd_dir}/{robot_name}/{robot_name}.usda`.
  Re-running with a changed config writes `robot_1/`, `robot_2/` instead of overwriting, so
  delete stale folders.
- **Joint drives:** URDFs carry no gains. `--joint-target-type none` with zero gains suits effort
  control. Otherwise the drive adds PD effort on top of your torques. With an Isaac Lab actuator
  config, the actuator owns the gains. For PhysX, `ImplicitActuatorCfg` gains override USD. For
  Newton, the gain pattern picks the mode (stiffness only = position, damping only = velocity,
  both = PD, zero = effort).
- **Fixed base:** use `--fix-base` for arms. Floating-base legged robots must not be fixed. In
  configs, `fix_root_link` lives on the spawner cfg.
- **2.x options are gone or ignored** (`NaturalFrequencyGainsCfg`, `make_instanceable`, MJCF
  `--fix-base`). MJCF output now nests rigid bodies. See `references/migration-2x-to-3x.md`.
- **Collision approximation:** visual meshes are rarely good colliders. A convex hull of a
  concave part (gripper fingers, drawer handles) causes phantom contacts. Use convex
  decomposition or primitives. A "failed to cook GPU-compatible convex mesh" warning means the
  collider fell back to CPU, which hurts speed.
- **Self-collision** is off by default. Enable it for humanoids and hands, then check the
  initial pose has no overlaps.
- **Physics variant:** with no `Physics` variant selected, the asset composes with no joints
  or mass.
- **Mass and inertia:** URDFs with zero or placeholder inertia make links explode. Check that
  every link has a plausible mass and a positive-definite inertia.

For meshes (OBJ/STL/FBX/glTF), use `scripts/tools/convert_mesh.py`. It needs Isaac Sim, and
props such as `--mass` and `--collision-approximation` are passed explicitly.

## Environment Design

| | Manager-based (`ManagerBasedRLEnv`) | Direct (`DirectRLEnv`) |
|---|---|---|
| Define | Config classes of terms: observations, actions, rewards, terminations, events, commands, curriculum | One class with `_pre_physics_step`, `_apply_action`, `_get_observations`, `_get_rewards`, `_get_dones`, `_reset_idx` |
| Best for | Most new tasks, ablations, Hydra overrides of single terms, sharing MDP terms | Custom step logic, fused reward kernels, MARL (`DirectMARLEnv`), Isaac Gym ports |
| Randomisation | `events` | `events` config also works in direct envs |

Both are driven by a `@configclass` config (a dataclass with mutable defaults handled for you;
use `cfg.replace(...)` or `isaaclab.utils.replace(cfg, ...)` to copy with changes). The scene is
declared on `cfg.scene` and accessed as `env.scene["name"]` in both workflows.

### Compact manager-based task (verified against 3.0 source)

```python
# my_tasks/cartpole_upright/env_cfg.py
import math
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm, ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm, RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass, replace
from isaaclab.utils.math import wrap_to_pi
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_assets.robots.cartpole import CARTPOLE_CFG
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_tasks.utils import PresetCfg

POLE = SceneEntityCfg("robot", joint_names=["cart_to_pole"])
CART = SceneEntityCfg("robot", joint_names=["slider_to_cart"])


def pole_upright(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Custom term: (num_envs,) tensor in [0, 1]. Asset data is a ProxyArray -> use .torch."""
    q = wrap_to_pi(env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids])
    return torch.exp(-torch.sum(q**2, dim=1) / 0.1)


@configclass
class PhysicsCfg(PresetCfg):            # selectable with physics=<field name>
    isaacsim_physx: PhysxCfg = PhysxCfg()                                  # needs --extra isaacsim
    newton_mjwarp: NewtonCfg = NewtonCfg(solver_cfg=MJWarpSolverCfg())    # kit-less
    default: PhysxCfg = isaacsim_physx


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    robot: ArticulationCfg = replace(CARTPOLE_CFG, prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg(intensity=2000.0))


@configclass
class ActionsCfg:
    effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.05, n_max=0.05))

        def __post_init__(self):
            self.enable_corruption = True      # noise is applied only when this is True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_cart = EventTerm(func=mdp.reset_joints_by_offset, mode="reset",
                           params={"asset_cfg": CART, "position_range": (-1.0, 1.0), "velocity_range": (-0.5, 0.5)})
    reset_pole = EventTerm(func=mdp.reset_joints_by_offset, mode="reset",
                           params={"asset_cfg": POLE, "position_range": (-0.25 * math.pi, 0.25 * math.pi),
                                   "velocity_range": (-0.25 * math.pi, 0.25 * math.pi)})


@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    upright = RewTerm(func=pole_upright, weight=1.0, params={"asset_cfg": POLE})
    cart_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.01, params={"asset_cfg": CART})


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)    # truncation, not failure
    cart_out = DoneTerm(func=mdp.joint_pos_out_of_manual_limit, params={"asset_cfg": CART, "bounds": (-3.0, 3.0)})


@configclass
class CartpoleUprightEnvCfg(ManagerBasedRLEnvCfg):
    scene: SceneCfg = SceneCfg(num_envs=4096, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 2                 # policy at 60 Hz
        self.episode_length_s = 5.0
        self.sim.dt = 1 / 120               # physics at 120 Hz
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysicsCfg()
```

```python
# my_tasks/cartpole_upright/__init__.py
import gymnasium as gym

gym.register(
    id="MyOrg-Cartpole-Upright",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:CartpoleUprightEnvCfg",
        "rsl_rl_cfg_entry_point": "isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg:CartpolePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
```

Make `isaaclab train` find the task. Scaffold with `uv run isaaclab --new` (template
generator), or declare the entry point in your package's `pyproject.toml` and reinstall:

```toml
[project.entry-points."isaaclab.tasks"]
my_tasks = "my_tasks"
```

Then check with `uv run isaaclab list_envs --keyword Upright` and
`uv run isaaclab zero_agent --task MyOrg-Cartpole-Upright --num_envs 4`.

Rules this example encodes:

- `sim.dt * decimation` is the control period. Set `render_interval = decimation` so cameras
  and visualizers don't render every physics step.
- Mark time limits `time_out=True` so they are truncations, not failures. The RL side is in
  `robot-reinforcement-learning`.
- Every term function returns a `(num_envs,)` or `(num_envs, k)` torch tensor on `env.device`.
  No Python loops over envs and no `.item()` in the step path, because each sync stalls the GPU.
- Resolve joint/body names once through `SceneEntityCfg(..., joint_names=...)`. Do not
  hard-code indices, because joint order can differ between PhysX and Newton.

### Direct workflow essentials

```python
from isaaclab.envs import DirectRLEnv

class MyEnv(DirectRLEnv):
    cfg: "MyEnvCfg"          # skeleton: DirectRLEnvCfg with action_space, observation_space, scene, sim

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.robot = self.scene["robot"]
        self._joint_ids, _ = self.robot.find_joints(["slider_to_cart"])

    def _pre_physics_step(self, actions):  self.actions = actions.clone() * self.cfg.action_scale
    def _apply_action(self):                # called `decimation` times per env step
        self.robot.actuators.target_command.set_effort_index(value=self.actions, joint_ids=self._joint_ids)
    def _get_observations(self):            return {"policy": self.robot.data.joint_pos.torch.clone()}
    def _get_rewards(self):                 ...
    def _get_dones(self):                   return terminated, self.episode_length_buf >= self.max_episode_length
    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += self.scene.env_origins[env_ids]       # default poses are env-local
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
```

In 3.0, writes are `write_*_to_sim_index(..., env_ids=)` for partial data and
`write_*_to_sim_mask(..., env_mask=)` for full data. Joint targets go through
`robot.actuators.target_command.set_{position,velocity,effort}_index`. The old
`robot.set_joint_*_target*` still works but warns. The stock 3.0 Cartpole-Direct still calls
it, so don't copy that line.

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
- Outputs go to `logs/<library>/<experiment_name>/<timestamp>/`. RSL-RL `play` exports
  `exported/policy.pt` (TorchScript) and `exported/policy.onnx` into the run directory.
- Hydra overrides use `env.<path>=` and `agent.<path>=` (rl_games uses `agent.params.seed`).
  Overrides do **not** re-run `__post_init__`. If you change `env.sim.dt` or `env.decimation`,
  also override `env.sim.render_interval` and every sensor `update_period` derived from them.
- TensorBoard: `uv run python -m tensorboard.main --logdir logs/rsl_rl`. For W&B:
  `--logger wandb --log_project_name <proj>` with `--extra wandb`.
- Custom scripts: build the cfg with `isaaclab_tasks.utils.parse_env_cfg(task, device=,
  num_envs=, overrides=)`. Enter `with isaaclab.app.launch_simulation(env_cfg, args_cli):` and
  create the env with `isaaclab.utils.instantiate(env_cfg)` or `gym.make(task, cfg=env_cfg)`.
  Kit-dependent imports must happen after launch.
- `num_envs`: start at 16-64 to debug, then scale until env-steps/s stops improving
  (`benchmark runtime`). 4096 is typical for proprioceptive locomotion. Camera tasks fit far
  fewer.

## Sensors and Rendering

| Sensor (`isaaclab.sensors`) | Measures | Cost and gotchas |
|-----------------------------|----------|------------------|
| `CameraCfg` | `rgb`, `depth`, `normals`, segmentation, ... as `(N, H, W, C)` `ProxyArray` in `data.output` | Rendering usually dominates step time. Tiling is built into `Camera`. `TiledCameraCfg` is a deprecated alias |
| `ContactSensorCfg` | `net_forces_w`, `net_normal_forces_w`, `force_matrix_w` (with `filter_prim_paths_expr`), air time | Needs `activate_contact_sensors=True` on the spawner. Filtered matrices cost more |
| `RayCasterCfg` + `patterns.GridPatternCfg` / `LidarPatternCfg` | Height scans, lidar-like hits against `mesh_prim_paths` | Cheap. Hits only the listed meshes; use `MultiMeshRayCasterCfg` for moving targets |
| `ImuCfg` | `ang_vel_b`, `lin_acc_b` (includes gravity, like a real IMU) | The 2.x full-state "IMU" is now `PvaCfg` |
| `FrameTransformerCfg` | Poses of target frames relative to a source frame | Use it for end-effector or fingertip frames instead of hand-rolled transforms |
| `JointWrenchSensorCfg` | Joint reaction wrenches | Replaces `data.body_incoming_joint_wrench_b` |

```python
from isaaclab.sensors import CameraCfg
from isaaclab_newton.renderers import NewtonWarpRendererCfg

wrist_cam = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/hand/wrist_cam",
    update_period=1 / 30,                 # render at the policy/camera rate, not the physics rate
    height=96, width=96, data_types=["rgb", "depth"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, horizontal_aperture=20.955, clipping_range=(0.05, 5.0)),
    offset=CameraCfg.OffsetCfg(pos=(0.05, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0), convention="ros"),  # XYZW
    renderer_cfg=NewtonWarpRendererCfg(),  # or select with renderer= presets
)
# rgb = env.scene["wrist_cam"].data.output["rgb"].torch   -> (num_envs, 96, 96, 3)
```

Renderers: **Newton Warp** (`renderer=newton_renderer`) is fastest and lightest, with direct
lighting and no motion vectors. **OVRTX** (`renderer=ovrtx`) is the recommended kit-less RTX
path. **Isaac RTX** (`renderer=isaacsim_rtx`) is legacy, for runs that must stay in Isaac Sim.

Performance and correctness:

- Image memory scales with envs × H × W × channels × outputs, plus your vision encoder. Use
  small images (64-128 px), request only the buffers the policy consumes, and set
  `update_period` to the control rate.
- **Large RTX camera atlases can return black or corrupted images with no error** (about a 4 GiB
  Vulkan buffer limit, Isaac Lab issue #8412). Reduce resolution, camera count or SPP, or use
  `newton_renderer`. Save and inspect a frame from the **last** env before training
  (`env.write_image_to_file=true` on camera tasks that support it).
- Camera `offset.convention`: `world` is +X forward/+Z up, `ros` is +Z forward/-Y up, `opengl`
  is -Z forward/+Y up. A wrong convention points the camera at the floor.
- Physics-only sensors (contact, ray caster, IMU) are cheap. Give them `update_period=sim.dt`
  or the control period on purpose.

## Randomisation and Actuators

Event modes (`EventTermCfg.mode`): `prestartup` authors USD before sim start (scale, some
materials). `startup` runs once after sim start (mass, friction buckets). `reset` runs per
episode. `interval` runs during an episode, with `interval_range_s`.

```python
@configclass
class EventsCfg:
    friction = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": (0.6, 1.2),
                "dynamic_friction_range": (0.5, 1.0), "restitution_range": (0.0, 0.1), "num_buckets": 64})
    base_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "mass_distribution_params": (1 / 1.25, 1.25), "operation": "scale", "distribution": "log_uniform"})
    gains = EventTerm(func=mdp.randomize_actuator_gains, mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"), "stiffness_distribution_params": (0.8, 1.2),
                "damping_distribution_params": (0.8, 1.2), "operation": "scale"})
    push = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}})
```

The full list of built-in event functions and their parameters is in
`references/config-reference.md`. Confirm the parameter names in `isaaclab/envs/mdp/events.py`.

Backend differences that bite: PhysX material randomisation is bucketed and runs on CPU
tensors. Newton has one friction coefficient, so it ignores `dynamic_friction_range` and
`num_buckets`, and it writes position-only CoM changes. Give each backend its own event set
through a `PresetCfg` events config.

Actuator models (`isaaclab.actuators`):

| Cfg | Physics | Use |
|-----|---------|-----|
| `ImplicitActuatorCfg` | PD inside the solver (stable at high gains) | Default for position-controlled arms. Gains are the drive stiffness/damping |
| `IdealPDActuatorCfg` | Explicit PD torque, computed by Isaac Lab | When you need the computed torque, or torque-level effects |
| `DCMotorCfg` | Explicit PD plus a torque-speed curve (`saturation_effort`, `actuator_velocity_limit`) | Legged robots with real motor limits |
| `DelayedPDActuatorCfg` | Explicit PD with `min_delay`/`max_delay` in **physics steps** | Actuation latency randomisation |
| `RemotizedPDActuatorCfg` | Delayed PD with a joint-angle-dependent torque lookup | Linkage-driven joints (knees) |
| `ActuatorNetMLPCfg` / `ActuatorNetLSTMCfg` | Learned actuator model from `network_file` (TorchScript) | ANYmal-style sim-to-real with identified drives |

- `actuator_effort_limit` clips the explicit model output. `joint_effort_limit` is the solver
  clamp. In 3.0 an explicit group **keeps** the solver limit, so effort is clipped twice. Set
  `joint_effort_limit >= actuator_effort_limit`.
- Explicit actuators need a small `sim.dt`. The stock legged tasks use 0.005 s. **Heuristic:**
  a stiff explicit PD at 1/60 s oscillates and explodes, while implicit PD tolerates larger steps.
- Add `armature` (reflected rotor inertia) for geared joints. Its absence is a common cause of
  jitter and of sim-to-real mismatch.

Sim-to-real hooks:

- Observation noise per term (`noise=`, active only with `enable_corruption=True`).
- Observation latency (`ObsTerm(delay_min_lag=, delay_max_lag=)`, in observation samples) and
  action latency (`DelayedPDActuatorCfg`).
- Direct envs: `observation_noise_model` and `action_noise_model`
  (`NoiseModelWithAdditiveBiasCfg`).
- Actuator nets for identified drives.
- PhysX↔Newton policy transfer as a cheap sim-to-sim check before hardware
  (`docs/source/how-to/transfer_policies_between_physx_and_newton.rst`).

Which parameters to randomise, by how much, and why: `robot-reinforcement-learning`.

## Demonstrations and Imitation Learning

```bash
# Teleop-record -> annotate subtasks -> multiply with Mimic (keyboard | spacemouse; XR: --xr on IK-Abs)
uv run --extra teleop,isaacsim isaaclab teleop record --task IsaacContrib-Stack-Cube-Franka-IK-Rel \
    --viz kit --teleop_device spacemouse --dataset_file ./datasets/dataset.hdf5 --num_demos 10
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --viz kit --auto \
    --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --num_envs 300 --generation_num_trials 1000 \
    --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5
```

- **Mimic needs task-space (end-effector) actions.** It replays subtask segments under rigid
  transforms relative to a reference object, so convert joint-space data with FK first.
  Mimic task IDs still end in `-v0`.
- Only successful generated trials are kept. **Heuristic:** a success/attempt ratio below about
  10% usually means bad annotations or subtasks that are too long.
- **Format:** robomimic-style HDF5 (`data/demo_<i>/actions`, `obs/...`, `attrs["success"]`).
  Train BC with `scripts/imitation_learning/robomimic/train.py`. Merge and convert with
  `scripts/tools/` (`merge_hdf5_datasets.py`, `hdf5_to_mp4.py`, `mp4_to_hdf5.py` for Cosmos
  augmentation).
- **GR00T N1.5:** the humanoid locomanipulation SDG workflow fine-tunes it in a separate uv env
  and converts HDF5 to LeRobot format. RLinf (`--rl_library rlinf`) does RL post-training of
  VLAs.
- Full pipeline (replay, visuomotor, SkillGen, XR, GR00T steps): load
  `references/demonstrations-and-ros2.md`. For dataset QA use `robot-data-quality`, for
  conversion `lerobot`, and for policy choice `imitation-learning`.

## ROS 2 Bridge

- `isaacsim.ros2.bridge` in Isaac Sim 6.2 supports **Jazzy on Ubuntu 24.04 (recommended)** and
  **Humble on 22.04**. Humble on 24.04 is unsupported.
- Libraries: source your ROS 2 before launching, or leave `ROS_DISTRO` unset to use the bundled
  libraries.
- Match `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on both sides. Containers and remote hosts
  need a UDP-only FastDDS profile.
- OmniGraph publishers and subscribers (`ROS2PublishClock`, `ROS2PublishJointState`,
  `ROS2SubscribeJointState`, `ROS2CameraHelper`, ...) run **only while the sim is playing**.
  Publish `/clock` and use `use_sim_time:=true`.
- Deploy learned policies as an ONNX export inside a ROS 2 node at the training control rate,
  and use the bridge for single-robot integration tests, not for parallel RL. Details:
  `references/demonstrations-and-ros2.md`; ROS side: `ros2`, `docker-ros2-development`.

## Performance and Debugging

Physics settings that matter:

- `sim.dt` with `decimation`: stock tasks use 1/120 s (Cartpole) to 1/200 s (legged).
- PhysX scene iteration ranges (`min/max_position_iteration_count`), and per articulation
  `solver_position_iteration_count`. Stock robots use 4 (ANYmal) to 8 (Franka); raise it for
  grasping only if a fixed reproduction shows penetration.
- `solver_type=1` (TGS) for articulations. GPU buffers: `gpu_max_rigid_patch_count`,
  `gpu_max_rigid_contact_count` and others are fixed, so overflow **drops contacts**.
- Newton MJWarp: `njmax`/`nconmax` at least the expected constraints and contacts per env,
  `num_substeps`, `integrator="implicitfast"`.
- `env_spacing` large enough that envs never touch. `filter_collisions=True` (the default)
  stops cross-env collisions. `replicate_physics=True` needs identical assets per env; use
  `MultiAssetSpawnerCfg` for variety.

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `unrecognized arguments: --headless` (or `--enable_cameras`) | Removed in 3.0 | Omit `--viz` (headless is the default), or use `--viz none`. Cameras enable themselves |
| Robot spawns rotated or upside down after an upgrade; goals wrong | WXYZ quaternions in 2.x code (3.0 is XYZW), or a USD `Gf.Quat` (real-first) passed straight in | `python scripts/tools/find_quaternions.py --path my_project/` (never on Isaac Lab itself), `--fix-identity-only` first, review the rest. `WARN_ON_TORCH_QUATF_ACCESS=1` flags runtime reads |
| `'ProxyArray' object has no attribute 'clone'` / `detach` | 3.0 data properties are `ProxyArray` | Use `.torch` (or `.warp`) |
| `ModuleNotFoundError: omni.isaac.lab` / `omni.isaac.core` | Pre-2.0 / pre-Isaac Sim 4.5 code | See `references/migration-2x-to-3x.md` |
| `ModuleNotFoundError: isaacsim.core.experimental...` | 3.0 no longer preloads Isaac Sim extensions | `from isaaclab.sim.utils import enable_extension; enable_extension("<ext>")` after Kit launch, then import |
| Articulation explodes or NaNs on the first steps | Spawn overlap, joint positions outside limits, zero/tiny link inertia, stiff explicit PD at large dt | Clamp resets to soft limits, fix inertias, use implicit PD or smaller dt, add armature, raise position iterations |
| Robot sags or ignores targets | Zero drive gains from import (`target_type none`), effort limit too low, wrong action term | Set actuator stiffness/damping, `joint_effort_limit >= actuator_effort_limit`, check the action term's joint regex |
| Joints move in PhysX but not Newton | Joint not covered by an actuator config, or gains pick the wrong Newton mode | Cover every actuated joint with an actuator cfg. Check the gain pattern |
| Contact sensor reads zeros | `activate_contact_sensors=False` on the spawner, or wrong body regex | Enable it on the spawner. Print `sensor.body_names` |
| `[PhysX]` buffer overflow warning; objects fall through each other at scale | Fixed GPU buffer too small | Raise only the named `gpu_*` capacity in `PhysxCfg` |
| "Failed to cook GPU-compatible mesh", slow steps | Convex mesh fell back to CPU | Use primitive or convex-decomposition colliders |
| Fingers or handles collide with air | Convex hull of a concave mesh | Use convex decomposition or SDF colliders. Inspect the collision visualisation |
| Neighbouring envs interact | `env_spacing` too small, or a global prim spawned per env | Increase spacing. Global props go under `/World`, per-env props under `{ENV_REGEX_NS}` |
| Camera images black or garbage at scale | RTX atlas limit, wrong offset convention, no light | Smaller images or fewer cameras, or `newton_renderer`. Fix `convention`. Add a light. Inspect the last env |
| Training slow, low GPU use | Visualizer on, cameras rendering every physics step, too few envs, CPU device | `--viz none`, set `update_period`, `benchmark runtime` to size `num_envs`, `--device cuda:0` |
| `env.sim.dt=` override broke sensors or timing | Hydra does not re-run `__post_init__` | Override `render_interval` and sensor `update_period` too |
| Checkpoint fails to load: shape mismatch | Different `presets=` / observation config from training | Use the same task, presets and agent config as the run |
| Task not found by `isaaclab train` | External package not imported | Add the `isaaclab.tasks` entry point, run `uv sync`, check `list_envs` |
| Crash in `libusd_tf` with OVRTX or OvPhysX | Kit/Isaac Sim USD in the same env or process | Separate envs. Don't combine `--extra isaacsim` workflows with ov runtimes in one process |

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
