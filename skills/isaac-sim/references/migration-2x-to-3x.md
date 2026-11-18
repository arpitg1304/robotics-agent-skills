# Isaac Lab / Isaac Sim Migration Notes

Verified against Isaac Lab `v3.0.0` (`docs/source/isaaclab/migrating.rst`, which is about 5,000
lines and authoritative) and Isaac Sim 6.2.0, 2026-11-18. Not executed. Read the guide in your
checkout for the full text, because `main` has already removed some 3.0 deprecated aliases
(for example `effort_limit_sim`, scheduled for removal in 3.1).

Migration order that works:

1. Fresh 3.0 checkout and a Python 3.12 environment. Never upgrade a 2.x env in place.
2. Package and backend boundaries.
3. Asset, actuator and sensor APIs.
4. Data access and quaternions.
5. RL entrypoints.
6. Visualizers and recording.
7. Importers.

Smoke-test after each step with `isaaclab zero_agent --num_envs 4`.

## Naming eras at a glance

| Era | Isaac Sim modules | Isaac Lab modules | Train command |
|-----|-------------------|-------------------|---------------|
| Isaac Gym Preview / IsaacGymEnvs (deprecated) | `isaacgym.gymapi`, `gymtorch` | n/a | `train.py task=...` |
| Isaac Sim ≤ 4.2, Isaac Lab 1.x (Orbit successor) | `omni.isaac.core`, `omni.isaac.kit.SimulationApp`, `omni.importer.urdf` | `omni.isaac.lab`, `omni.isaac.lab_tasks`, `omni.isaac.lab_assets` | `./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py` |
| Isaac Sim 4.5-5.1, Isaac Lab 2.x | `isaacsim.core.api`, `isaacsim.core.prims`, `isaacsim.core.utils`, `isaacsim.SimulationApp`, `isaacsim.asset.importer.urdf` | `isaaclab`, `isaaclab_tasks`, `isaaclab_assets`, `isaaclab_rl`, `isaaclab_mimic` | `./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-...-v0 --headless` |
| Isaac Sim 6.x, Isaac Lab 3.0 | `isaacsim.core.experimental.*`, `isaacsim.sensors.experimental.*` (old `isaacsim.core.*` deprecated) | plus `isaaclab_physx`, `isaaclab_newton`, `isaaclab_ov`, `isaaclab_visualizers`, `isaaclab_teleop` | `uv run isaaclab train --rl_library rsl_rl --task Isaac-... [physics=...]` |

Isaac Sim 4.5 renames (from Isaac Lab 2.0 release notes):

| Old | New |
|-----|-----|
| `omni.isaac.cloner` | `isaacsim.core.cloner` |
| `omni.isaac.core.prims` | `isaacsim.core.prims` |
| `omni.isaac.core.simulation_context` | `isaacsim.core.api.simulation_context` |
| `omni.isaac.core.utils` | `isaacsim.core.utils` |
| `omni.isaac.core.world` | `isaacsim.core.api.world` |
| `omni.isaac.kit.SimulationApp` | `isaacsim.SimulationApp` |
| `omni.isaac.ui` | `isaacsim.gui.components` |
| `omni.importer.urdf` / `omni.importer.mjcf` | `isaacsim.asset.importer.urdf` / `.mjcf` |
| `omni.isaac.lab`, `omni.isaac.lab_tasks`, `omni.isaac.lab_assets` | `isaaclab`, `isaaclab_tasks`, `isaaclab_assets` |
| `source/standalone/workflows/*` | `scripts/reinforcement_learning/*`, `scripts/imitation_learning/*` |

Isaac Sim 6.0 deprecations. Prefer the Isaac Lab in-tree API, and fall back to `experimental`:

| Deprecated (Isaac Sim) | Replacement |
|------------------------|-------------|
| `isaacsim.core.utils.stage` | `isaaclab.sim.utils.stage` (`get_current_stage`, `create_new_stage`, `open_stage`, `save_stage`, ...) |
| `isaacsim.core.utils.prims` | `isaaclab.sim.utils.prims` |
| `isaacsim.core.utils.queries` / `.transforms` / `.semantics` | `isaaclab.sim.utils.queries` / `.transforms` / `.semantics` |
| `isaacsim.core.utils.extensions.enable_extension` | `isaaclab.sim.utils.enable_extension` (needs running Kit) |
| `isaacsim.core.utils.viewports.set_camera_view` | `isaacsim.core.rendering_manager.ViewportManager.set_camera_view` |
| `isaacsim.core.prims.XFormPrim(View)` | `isaaclab.sim.views.FrameView`; for articulations/rigid prims `isaacsim.core.experimental.prims` |
| `isaacsim.core.simulation_manager.SimulationManager` | `isaaclab_physx.physics.PhysxManager` or `isaaclab_newton.physics.NewtonManager` |
| `isaacsim.core.cloner` | `isaaclab.cloner` |
| `isaacsim.sensors.<name>` | `isaacsim.sensors.experimental.<name>` |
| `isaacsim.robot.wheeled_robots` | `isaacsim.robot.experimental.wheeled_robots` |
| `omni.physics.tensors.impl.api` | `omni.physics.tensors.api` |

In 3.0, stock Kit experiences no longer preload Isaac Sim extensions, so enable them before
any direct import:

```python
from isaaclab.sim.utils import enable_extension      # NOT from isaacsim.core.experimental.utils.app
enable_extension("isaacsim.core.experimental.prims")  # after Kit has launched
from isaacsim.core.experimental.prims import XformPrim
```

Custom `.kit` files: drop the `extsDeprecated` search path, list the experimental extensions
explicitly, and remove extensions that pull in `isaacsim.core.api`.

## Install and CLI (2.x → 3.0)

| 2.x | 3.0 |
|-----|-----|
| `conda create ... python=3.11`; `./isaaclab.sh --install rsl_rl` | `uv run [--extra isaacsim] isaaclab ...` (Python 3.12); `./isaaclab.sh -i` still exists |
| `scripts/reinforcement_learning/<lib>/train.py` / `play.py` | `isaaclab train` / `isaaclab play --rl_library <lib>`; direct script `scripts/reinforcement_learning/train.py --rl_library <lib>` |
| `--resume --load_run RUN` | `--checkpoint <path>\|latest\|best` |
| `--use_pretrained_checkpoint` | `--checkpoint pretrained` |
| `--headless` | Removed (the parser rejects it). Omit `--viz` for headless |
| `--enable_cameras` | Removed. Cameras enable themselves |
| `--viz newton` | `--viz newton_gl` (or `newton_rtx`); `newton` is a deprecated alias |
| Task IDs with `-v0` (`Isaac-Cartpole-v0`) | Suffixless for core tasks (`Isaac-Cartpole`, `Isaac-Velocity-Rough-AnymalD`). Mimic tasks keep `-v0`. Contributed tasks use the `IsaacContrib-` prefix |
| `isaaclab_tasks.manager_based.*` / `.direct.*` | `isaaclab_tasks.core.<task>` and `isaaclab_tasks.contrib.<task>`, with `_manager_` / `_direct_` file infixes |
| `scripts/benchmarks/benchmark_rsl_rl.py` | `isaaclab benchmark training --rl_library rsl_rl` |
| `presets=newton` broad selection | Typed selectors `physics=`, `renderer=`; `presets=` for task modes |
| `fold_preset_tokens(...)` | Removed; pass `physics=` / `renderer=` / `presets=` tokens through unchanged |

## Backends and schema configs

- `sim.physics = sim_utils.PhysxCfg(...)` → `from isaaclab_physx.physics import PhysxCfg`.
  Multi-backend tasks wrap the alternatives in `isaaclab_tasks.utils.PresetCfg` with a
  `default` field. `isaaclab.physics.PhysxAutoCfg(isaacsim_physx=..., ovphysx=...)` picks Kit
  PhysX or OvPhysX automatically.
- `SurfaceGripper(Cfg)` moved to `isaaclab_physx.assets`. `Articulation`, `RigidObject`,
  `RigidObjectCollection`, `DeformableObject` and the common sensors stay importable from
  `isaaclab.*`. Factories dispatch to the active backend.
- Schema cfgs: the 2.x `*PropertiesCfg` classes are deprecated (removed in 3.2) in favour of
  per-namespace **fragments** passed as a list. Replacement sets:

| Deprecated | Fragments |
|------------|-----------|
| `RigidBodyPropertiesCfg` | `UsdPhysicsRigidBodyCfg` + `PhysxRigidBodyCfg` |
| `CollisionPropertiesCfg` | `UsdPhysicsCollisionCfg` + `PhysxCollisionCfg` (+ `NewtonCollisionCfg`) |
| `JointDrivePropertiesCfg` | `UsdPhysicsDriveCfg` + `PhysxJointCfg` (+ `MujocoJointCfg`) |
| `ArticulationRootPropertiesCfg` | `PhysxArticulationCfg` (+ `NewtonArticulationCfg`) |
| `MassPropertiesCfg` | `MassCfg` |
| `ConvexHull/ConvexDecomposition/TriangleMesh/SDFMeshPropertiesCfg` | `PhysxConvexHullCfg`, `PhysxConvexDecompositionCfg`, `PhysxTriangleMeshCfg`, `PhysxSDFMeshCfg` (+ `UsdPhysicsMeshCollisionCfg`) |

USD-namespace fragments live in `isaaclab.sim.schemas`, PhysX ones in
`isaaclab_physx.sim.schemas`, and Newton/MuJoCo ones in `isaaclab_newton.sim.schemas`.
`fix_root_link` and `ensure_drives_exist` move to the spawner cfg. Drive field renames:
`max_velocity` → `max_joint_velocity`, `max_effort` → `max_force`.

## Assets, actuators, sensors

| 2.x | 3.0 |
|-----|-----|
| `XformPrimView` | `FrameView` (`isaaclab.sim.views`) |
| `root_physx_view` | `root_view` (backend-specific type) |
| Actuator `effort_limit` / `effort_limit_sim` / `velocity_limit_sim` | `actuator_effort_limit` / `joint_effort_limit` / `joint_velocity_limit` (old names removed in 3.1) |
| Explicit actuator groups raised the solver effort limit to 1e9 | Solver keeps `joint_effort_limit`, so set it ≥ `actuator_effort_limit` |
| `robot.set_joint_position_target(...)` | `robot.actuators.target_command.set_position_index(value=..., joint_ids=..., env_ids=...)` (keyword-only) |
| `data.joint_pos_target`, `data.applied_torque`, `data.computed_torque` | `actuators.target_command.position`, `actuators.applied_effort`, `actuators.computed_effort` |
| Group properties `armature`, `friction`, `effort_limit_sim` | Read `data.joint_armature` etc.; write `write_joint_armature_to_sim_index(...)` etc. |
| `ActuatorBase._parse_joint_parameter` | `isaaclab.actuators.resolve_joint_parameter` |
| `write_actuator_stiffness_to_sim` | `mdp.randomize_actuator_gains` event, or `isaaclab.actuators.newton.write_group_parameter` |
| `RigidObjectCollection.write_object_*` / `find_objects` | `write_body_*` / `find_bodies` |
| `Imu` (full state) | `Pva` / `PvaCfg`; new `Imu` has only `ang_vel_b`, `lin_acc_b` (gravity included); `gravity_bias` removed |
| `mdp.imu_orientation`, `mdp.imu_projected_gravity` | `mdp.pva_orientation`, `mdp.pva_projected_gravity` |
| `data.body_incoming_joint_wrench_b` | `JointWrenchSensorCfg` → `sensor.data.force` / torque |
| `ContactSensorData.pos_w/quat_w/pose_w` | Deprecated; use `FrameTransformer` |
| `net_forces_w` (normal only, PhysX) | `net_forces_w` = total (normal + friction) on Isaac Sim PhysX 6.2+/Newton/OvPhysX; explicit `net_normal_forces_w`, `net_friction_forces_w`, `normal_force_matrix_w`, `friction_force_matrix_w` |
| `TiledCamera` / `TiledCameraCfg` | `Camera` / `CameraCfg` (tiling built in; old names are deprecated aliases) |
| `ArticulationData.gear_ratio` | Removed; read gear ratio from your actuator cfg |

## Data access and quaternions

- **Quaternions are XYZW** everywhere in Isaac Lab 3.0 (configs, tensors, `isaaclab.utils.math`).
  The identity is `(0, 0, 0, 1)`. `convert_quat` is only for external WXYZ data. USD `Gf.Quat*`
  and some Isaac Sim APIs are still real-first, so reorder at those boundaries.
- Find hard-coded quaternions in **your** code (never run it on Isaac Lab's own packages):

```bash
python scripts/tools/find_quaternions.py --path my_project/            # report
python scripts/tools/find_quaternions.py --path my_project/ --fix-identity-only
python scripts/tools/find_quaternions.py --path my_project/ --fix      # interactive, review each
WARN_ON_TORCH_QUATF_ACCESS=1 uv run python my_script.py                 # flags runtime .torch reads of quats
```

  The tool can mistake RGBA colours and bounding boxes for quaternions, so review every match.
- **ProxyArray:** every `.data.*` property returns `isaaclab.utils.warp.ProxyArray`. Use
  `.torch` for a cached zero-copy tensor or `.warp` for the `wp.array`. `torch.*` functions
  and `wp.launch` accept it directly (torch emits a one-time deprecation warning).
  `wp.to_torch(proxy)` is a temporary shim. Methods like `.clone()`, `.detach()`, indexing
  and `torch.zeros_like` need `.torch` first.
- **Writes split by selection:** `write_*_to_sim_index(data, env_ids=)` takes data shaped
  `(len(env_ids), ...)`. `write_*_to_sim_mask(data, env_mask=)` takes full `(num_envs, ...)`
  data plus a bool mask. The old `write_*_to_sim` forwards to `_index` with a warning.
- `TimestampedBufferWarp` is removed. Use `TimestampedBuffer(wp.zeros(...))`.

## Visualizers, camera view, video

| 2.x | 3.0 |
|-----|-----|
| `env_cfg.viewer.eye/lookat` (`ViewerCfg`) | `env_cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=..., lookat=...)` (`isaaclab.visualizers`) |
| `viewer.origin_type="asset_root"` | `sim.visualizer_cfgs = [KitVisualizerCfg(origin_type="asset", origin_track_path="robot")]` |
| `ViewportCameraController` | Deprecated; tracking lives in `KitVisualizer` |
| `NewtonVisualizerCfg` | `NewtonGLVisualizerCfg` / `NewtonRTXVisualizerCfg` (`isaaclab_visualizers.newton`) |
| `tiled_cam_view/num/prim_path/eye/renderer` | `streaming_view/streaming_envs/streaming_sensor_prim_path/streaming_cam_eye/streaming_cam_renderer_cfg` |
| `gym.wrappers.RecordVideo(env, ...)` | `env_cfg.video_recorders = [VideoRecorderCfg(source="visualizer", output_dir=..., video_length=...)]` from `isaaclab.envs.utils.video_recorder_cfg`; sources `visualizer`, `visualizer:kit`, `visualizer:newton_gl`, `sensor:<name>` |
| `isaaclab.envs.utils.recording_hooks` | `SimulationContext.add_render_callback()` |

On `main` after 3.0, at least Cartpole assigns `sim.visualizer_cfgs = VisualizerCfg(...)` where
3.0.0 used `default_visualizer_cfg`. Check your checkout's task files before copying.

## Importers

- **URDF importer 3.0** (Isaac Sim 6) is a Python pipeline built on `urdf-usd-converter` plus
  the asset transformer. Isaac Lab no longer pins the importer extension version.
  - `usd_file_name` is chosen for you (`{usd_dir}/{robot}/{robot}.usda`) and the output folder
    dedupes.
  - Ignored settings: `convert_mimic_joints_to_normal_joints`, `replace_cylinders_with_capsules`,
    `root_link_name`, `make_instanceable` (instanceable by default).
  - `merge_fixed_joints` works as a URDF pre-processing step.
  - `NaturalFrequencyGainsCfg` is deprecated and leaves the importer gains in place. Use
    `PDGainsCfg`.
  - CLI: `convert_urdf.py in.urdf OUT_DIR --fix-base --merge-joints --joint-stiffness
    --joint-damping --joint-target-type`.
- **MJCF importer** is built on `mujoco-usd-converter` with the pure-Python `MJCFImporter`.
  - Output has nested rigid bodies.
  - `import_inertia_tensor` and `import_sites` are removed (always on).
  - New settings: `merge_mesh`, `collision_from_visuals`, `collision_type`. `self_collision`
    maps to the importer's `allow_self_collision`.
  - Old CLI flags `--fix-base`, `--make-instanceable`, `--import-sites` are gone.
- Both importers author a `Physics` variant set. `physics_variant` on the converter cfg selects
  `physics` (portable), `physx` or `mujoco`.

## Isaac Gym / IsaacGymEnvs → Isaac Lab

- Port to a **direct** workflow first. The maintained Cartpole direct env
  (`isaaclab_tasks/core/cartpole/cartpole_direct_env.py`) is the reference.
- YAML task config → `@configclass` (`DirectRLEnvCfg` + `SimulationCfg` + scene cfg). Actors
  become `ArticulationCfg`/`RigidObjectCfg` entries in `cfg.scene`, reached via
  `self.scene["robot"]`.
- No `acquire_*_tensor`/`refresh_*`. Read `robot.data.joint_pos.torch`. Write
  `write_joint_position_to_sim_index(position=..., env_ids=...)`. `dof` is renamed to `joint`.
- Isaac Gym tensors used XYZW, older Isaac Lab used WXYZ, and 3.0 uses XYZW. Check every
  copied quaternion against the identity.
- Joint and body ordering can differ from Isaac Gym, and between PhysX and Newton. Look up
  indices by name.
- `pre_physics_step` → `_pre_physics_step` + `_apply_action` (called `decimation` times).
  `compute_observations` → `_get_observations` returning `{"policy": ...}`. `reset_idx` →
  `_reset_idx`, calling `super()._reset_idx(env_ids)` first and adding
  `scene.env_origins[env_ids]` to root positions.
- Launch: `uv run isaaclab train --rl_library rl_games --task <id>` (IsaacGymEnvs used
  rl_games).

In an Isaac Lab checkout, `.agents/skills/isaaclab-migrating-2x-to-3x` and
`isaaclab-migrating-from-isaac-gym` route through the same official guide with
evaluation scenarios.
