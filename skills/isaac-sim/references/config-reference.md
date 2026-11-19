# Isaac Lab 3.0 Configuration and CLI Reference

Fields and defaults were read from Isaac Lab `v3.0.0` source on 2026-11-18. Not executed. Each
field links back to a file under `source/`. Before relying on a default, re-read that file in
your checkout, because `main` changes quickly. Units are SI (m, kg, s, rad, N, N·m).

## Contents

- CLI: `isaaclab` subcommands and flags, plus training workflow notes
- Simulation and physics backends
- Scene and environment configs
- Assets and actuators
- Sensors
- Manager terms (observations, events, rewards, terminations)
- Built-in event functions

## CLI

`isaaclab` subcommands (`source/isaaclab/isaaclab/cli/__init__.py`): `train`,
`train_multigpu`, `play`, `zero_agent`, `random_agent`, `list_envs`, `benchmark`,
`microbenchmark`, `demo`, `example`, `leapp` and `teleop` (`run`, `record`, `replay`). Setup
flags on the bare command: `-i/--install [selectors]`, `--new` / `-n` (template generator),
`--editor`, `--isaacsim_source <path>`, `-p <script.py>` (run a Python script in the env).

Launcher flags (KitLauncher, accepted by every Isaac Lab script that calls `add_launcher_args`):

| Flag | Meaning |
|------|---------|
| `--viz / --visualizer a,b` | `kit`, `newton_gl`, `newton_rtx`, `rerun`, `viser`, `none`. Omitted means headless (unless cfg sets `sim.visualizer_cfgs`) |
| `--device` | `cpu`, `cuda`, `cuda:N` |
| `--livestream 0/1/2` | Off / public WebRTC / private WebRTC (`LIVESTREAM` env var too). Adds the `kit` visualizer |
| `--xr` | XR mode (teleoperation) |
| `--experience <file.kit>` | Custom Kit experience file |
| `--kit_args "..."` | Raw Kit args, for example `--kit_args "--ext-folder=/path"` |
| `--deterministic` | Request reproducible rendering |
| `--max_visible_envs N` | Cap envs drawn by visualizers |
| `--info`, `--verbose` | Kit log level |
| `--headless`, `--enable_cameras`, `--cpu` | **Rejected in 3.0** (removed) |

Train/play flags (`source/isaaclab_rl/isaaclab_rl/entrypoints/common.py` and
`backends/cli_args_*.py`):

| Flag | Train | Play | Meaning |
|------|-------|------|---------|
| `--task` | ✓ | ✓ | Registered gym ID |
| `--rl_library` | ✓ | ✓ | `rsl_rl`, `rl_games`, `skrl`, `sb3`, `rlinf`, `torchrl`. Defaults to the task's `default_agent` |
| `--agent` | ✓ | ✓ | Agent cfg entry-point key, for example `rsl_rl_with_symmetry_cfg_entry_point` |
| `--num_envs`, `--seed` | ✓ | ✓ | Override the env cfg |
| `--max_iterations` | ✓ | | Training iterations |
| `--checkpoint` | ✓ (resume) | ✓ | Path, `latest`, `best`, `pretrained` |
| `--video`, `--video_length`, `--video_interval` | ✓ | ✓ | Record clips (lengths in env steps) |
| `--distributed` | ✓ | | Multi-GPU (set by `train_multigpu`) |
| `--capture_env_sensors[_length/_interval/_format]` | ✓ | | Dump camera-like sensor frames (`tensorboard` or `file`) |
| `--disable_fabric` | | ✓ | Use USD I/O instead of Fabric (slow; debugging) |
| `--real-time` | | ✓ | Throttle to wall-clock |
| `--train_env_cfg` | | ✓ | Skip the task's play-mode overrides |
| `--logger`, `--log_project_name`, `--experiment_name`, `--run_name` | ✓ | | RSL-RL logging (`tensorboard`, `wandb`, `neptune`). Check `--help` per library |

Hydra tokens go last, with no dashes:

- `physics=<name>`, `renderer=<name>`, `presets=a,b`.
- `env.<dotted.path>=<value>` and `agent.<dotted.path>=<value>`. rl_games uses
  `agent.params.*`.
- `env.<path>=null` removes a term.
- `env.observations.policy.x.func=isaaclab.envs.mdp:joint_pos` swaps a function.

Overrides apply after `__post_init__`, so derived values (sensor `update_period`,
`render_interval`) must be overridden explicitly.

### Training workflow notes

Moved from SKILL.md (Training and Playing).

- Outputs go to `logs/<library>/<experiment_name>/<timestamp>/`. RSL-RL `play` exports
  `exported/policy.pt` (TorchScript) and `exported/policy.onnx` into the run directory.
- TensorBoard: `uv run python -m tensorboard.main --logdir logs/rsl_rl`. For W&B:
  `--logger wandb --log_project_name <proj>` with `--extra wandb`.
- Custom scripts: build the cfg with `isaaclab_tasks.utils.parse_env_cfg(task, device=,
  num_envs=, overrides=)`. Enter `with isaaclab.app.launch_simulation(env_cfg, args_cli):` and
  create the env with `isaaclab.utils.instantiate(env_cfg)` or `gym.make(task, cfg=env_cfg)`.
  Kit-dependent imports must happen after launch.
- `num_envs`: start at 16-64 to debug, then scale until env-steps/s stops improving
  (`benchmark runtime`). 4096 is typical for proprioceptive locomotion. Camera tasks fit far
  fewer.

## Simulation and physics

`isaaclab.sim.SimulationCfg` (`sim/simulation_cfg.py`):

| Field | Default | Notes |
|-------|---------|-------|
| `device` | `"cuda:0"` | |
| `dt` | `1/60` | Physics step [s] |
| `gravity` | `(0, 0, -9.81)` | |
| `render_interval` | `1` | Physics steps per render. Set it to `decimation` |
| `use_fabric` | `True` | GPU-side USD mirror. Disabling it is slow |
| `physics` | `None` → `PhysxCfg()` | Or a `PresetCfg` of backend cfgs |
| `physics_material` | `RigidBodyMaterialBaseCfg()` | Default material |
| `use_newton_actuators` | `True` | Explicit actuators run natively (Newton) or through a shared adapter |
| `visualizer_cfgs` / `default_visualizer_cfg` | `[]` / `None` | Visualizer setup and camera eye/lookat |
| `create_stage_in_memory` | `False` | Faster startup |
| `logging_level`, `save_logs_to_file`, `log_dir` | `"WARNING"`, `True`, `None` | |

`isaaclab_physx.physics.PhysxCfg` (`isaaclab_physx/physics/physx_manager_cfg.py`). These are
scene-level settings. Per-articulation iteration counts go on `PhysxArticulationCfg`.

| Field | Default | Notes |
|-------|---------|-------|
| `solver_type` | `1` (TGS) | `0` = PGS |
| `min/max_position_iteration_count` | `1` / `255` | Clamp per-actor iterations |
| `min/max_velocity_iteration_count` | `0` / `255` | |
| `enable_ccd` | `False` | Not supported on GPU dynamics (forced off) |
| `enable_stabilization` | `False` | Only for dt > ~1/30. Distorts contact-force readings |
| `bounce_threshold_velocity` | `0.5` m/s | Tasks often use 0.2 |
| `friction_offset_threshold` / `friction_correlation_distance` | `0.04` / `0.025` m | |
| `enable_enhanced_determinism` | `False` | |
| `solve_articulation_contact_last` | `False` | Try for stiff grasps |
| `gpu_max_rigid_contact_count` | `2**23` | Fixed capacity. Overflow drops contacts |
| `gpu_max_rigid_patch_count` | `5 * 2**15` | Rough-terrain tasks use `10 * 2**15` |
| `gpu_found_lost_pairs_capacity` | `2**21` | |
| `gpu_found_lost_aggregate_pairs_capacity` | `2**25` | |
| `gpu_total_aggregate_pairs_capacity` | `2**21` | |
| `gpu_collision_stack_size` / `gpu_heap_capacity` | `2**26` / `2**26` | The heap grows; the others don't |

`isaaclab_newton.physics.NewtonCfg` (`newton_manager_cfg.py`):

| Field | Default | Notes |
|-------|---------|-------|
| `solver_cfg` | `None` | `MJWarpSolverCfg`, `KaminoPADMMSolverCfg`, `VBDSolverCfg`, `MPMSolverCfg`, `XPBDSolverCfg`, `FeatherstoneSolverCfg` |
| `num_substeps` | `1` | Solver substeps per physics step |
| `use_cuda_graph` | `True` | Capture the step as a CUDA graph |
| `deterministic_mode` | `"not_guaranteed"` | `run_to_run`, `gpu_to_gpu` |
| `collision_cfg` | `None` | `NewtonCollisionPipelineCfg(max_triangle_pairs=...)` for terrain |
| `default_shape_cfg` | `NewtonShapeCfg()` | `margin=0.0`, `gap=0.01`, `ke=2.5e3`, `kd=100`, `mu=1.0` |
| `debug_mode` | `False` | |

`MJWarpSolverCfg` (`mjwarp_manager_cfg.py`):

| Field | Default |
|-------|---------|
| `njmax` | 300 |
| `nconmax` | `None` |
| `iterations` | 100 |
| `ls_iterations` | 50 |
| `solver` | `"newton"` |
| `integrator` | `"euler"` (tasks use `"implicitfast"`) |
| `cone` | `"pyramidal"` (`"elliptic"` is more accurate) |
| `impratio` | 1.0 |
| `use_mujoco_contacts` | `True` |
| `tolerance` | 1e-6 |

Undersized `njmax`/`nconmax` silently drops constraints.

## Scene and environment

`InteractiveSceneCfg` (`scene/interactive_scene_cfg.py`):

| Field | Default | Notes |
|-------|---------|-------|
| `num_envs`, `env_spacing` | required | Spacing in m. Envs must not touch |
| `replicate_physics` | `True` | Identical per-env assets. Use `MultiAssetSpawnerCfg` for variety |
| `filter_collisions` | `True` | No cross-env collisions |
| `clone_in_fabric` | `False` | Faster cloning (Cartpole 3.0.0 enables it) |
| `lazy_sensor_update` | `True` | Sensors compute only when read |

Asset fields on a scene cfg subclass become `env.scene["<field>"]`. Per-env prims use
`{ENV_REGEX_NS}/...`; global ones use `/World/...`.

`ManagerBasedRLEnvCfg` / `ManagerBasedEnvCfg` (`envs/manager_based_*_cfg.py`):

- Required: `sim`, `decimation`, `scene`, `observations`, `actions`, `episode_length_s`,
  `rewards`, `terminations`.
- Optional: `events`, `commands`, `curriculum`, `recorders`, `seed`, `video_recorders`,
  `teleop_devices`, `xr`, `is_finite_horizon`, `rerender_on_reset`, `wait_for_textures`.
- `viewer` is deprecated. Use `sim.default_visualizer_cfg`.

`DirectRLEnvCfg` (`envs/direct_rl_env_cfg.py`):

- `decimation`, `episode_length_s`, `scene`, `sim`.
- `action_space`, `observation_space`, `state_space` (int or gymnasium space).
- `events`, `observation_noise_model`, `action_noise_model`.
- `is_finite_horizon`, `video_recorders`.
- The old `num_actions`/`num_observations`/`num_states` are deprecated in favour of the
  `*_space` fields.

## Assets and actuators

`ArticulationCfg` (`assets/articulation/articulation_cfg.py`):

- `prim_path`, `spawn`, `init_state`, `actuators` (dict of group name to actuator cfg).
- `soft_joint_pos_limit_factor` (1.0), `articulation_root_prim_path`,
  `joint_ordering`/`body_ordering`, `collision_group`, `debug_vis`.

`ArticulationCfg.InitialStateCfg`:

| Field | Default |
|-------|---------|
| `pos` | `(0, 0, 0)` |
| `rot` | `(0, 0, 0, 1)` XYZW |
| `lin_vel` / `ang_vel` | zeros |
| `joint_pos` / `joint_vel` | `{".*": 0.0}` (regex keys) |

Positions are **env-local**: add `scene.env_origins` when writing root poses yourself.

Spawners (`isaaclab.sim`):

- `UsdFileCfg(usd_path, variants, scale, ...)`, `UrdfFileCfg`, `MjcfFileCfg`, `MeshFileCfg`,
  `GroundPlaneCfg`.
- Shape primitives, lights (`DistantLightCfg`, `DomeLightCfg`), cameras (`PinholeCameraCfg`,
  `FisheyeCameraCfg`), and `MultiAssetSpawnerCfg`.
- Common `FileCfg` slots: `rigid_props`, `collision_props`, `mass_props`,
  `articulation_props`, `joint_drive_props`, `fixed_tendons_props`, `physics_material`,
  `visual_material`, `activate_contact_sensors`, `fix_root_link`, `ensure_drives_exist`.

Actuator base fields (`actuators/actuator_base_cfg.py`). Every scalar also accepts
`{regex: value}`.

| Field | Meaning |
|-------|---------|
| `joint_names_expr` | Regex list of joints in this group |
| `stiffness`, `damping` | PD gains (implicit: solver drive). `None` keeps the USD value |
| `armature` | Reflected inertia added to the joint |
| `friction`, `dynamic_friction`, `viscous_friction` | Joint friction (`dynamic_friction` is PhysX/OvPhysX only) |
| `actuator_effort_limit`, `actuator_velocity_limit` | Rated limits, clip the explicit model output |
| `joint_effort_limit`, `joint_velocity_limit` | Solver-side joint property overrides |
| `effort_limit`, `velocity_limit`, `effort_limit_sim`, `velocity_limit_sim` | Deprecated aliases (removed in 3.1; already gone on `main`) |

Actuator subclasses:

| Cfg | Extra fields |
|-----|--------------|
| `ImplicitActuatorCfg` | none |
| `IdealPDActuatorCfg` | none |
| `DCMotorCfg` | `saturation_effort` |
| `DelayedPDActuatorCfg` | `min_delay`, `max_delay` (physics steps) |
| `RemotizedPDActuatorCfg` | `joint_parameter_lookup` |
| `ActuatorNetMLPCfg` | `network_file`, `pos_scale`, `vel_scale`, `torque_scale`, `input_order`, `input_idx` |
| `ActuatorNetLSTMCfg` | `network_file` |

Runtime:

- Commands: `robot.actuators.target_command.set_position_index(value=, joint_ids=, env_ids=)`,
  plus `set_velocity_index`, `set_effort_index` and the `_mask` variants.
- Telemetry: `robot.actuators.applied_effort`, `.computed_effort`.
- Group topology is frozen after construction.

## Sensors

Base fields (`SensorBaseCfg`): `prim_path`, `update_period` (0 means every physics step),
`debug_vis`.

| Cfg | Key fields | Data |
|-----|-----------|------|
| `CameraCfg` | `spawn` (`PinholeCameraCfg`/`FisheyeCameraCfg` or `None` for an existing prim), `offset=CameraCfg.OffsetCfg(pos, rot, convention="world"\|"ros"\|"opengl")`, `width`, `height`, `data_types` (`rgb`, `depth`, `distance_to_image_plane`, `normals`, `semantic_segmentation`, `instance_id_segmentation`, `albedo`, `simple_shading_*`, ...), `renderer_cfg`, `depth_clipping_behavior`, `semantic_filter` | `data.output[name]` → `ProxyArray (N, H, W, C)` |
| `ContactSensorCfg` | `history_length`, `track_air_time`, `force_threshold`, `filter_prim_paths_expr`, `track_pose`, `track_contact_points`, `track_friction_forces`, `max_contact_data_count_per_prim` | `net_forces_w`, `net_normal_forces_w`, `net_friction_forces_w`, `force_matrix_w`, `*_history`, `current_air_time`, `last_air_time`, `contact_pos_w` |
| `RayCasterCfg` | `mesh_prim_paths`, `pattern_cfg` (`patterns.GridPatternCfg(resolution, size)`, `LidarPatternCfg`, `BpearlPatternCfg`, `PinholeCameraPatternCfg`), `offset`, `ray_alignment` (`base`/`yaw`/`world`), `max_distance`, `drift_range` | Hit points and distances. `MultiMeshRayCaster(Camera)` for dynamic meshes |
| `ImuCfg` | `offset` | `ang_vel_b`, `lin_acc_b` (gravity included) |
| `PvaCfg` | `offset` | Pose, velocities, accelerations, `projected_gravity_b` (no gravity in acceleration) |
| `FrameTransformerCfg` | `prim_path` (source), `source_frame_offset`, `target_frames=[FrameTransformerCfg.FrameCfg(prim_path, name, offset=OffsetCfg(pos, rot))]` | Target poses in the source frame and in world |
| `JointWrenchSensorCfg` | | `force`, torque per joint (child-side frame) |

Renderer cfgs:

- `isaaclab_newton.renderers.NewtonWarpRendererCfg`
- `isaaclab_ov.renderers.OVRTXRendererCfg`
- `isaaclab_physx.renderers.IsaacRtxRendererCfg` (needs Isaac Sim)
- `isaaclab_tasks.utils.presets.MultiBackendRendererCfg`, a preset with `newton_renderer`
  (default), `ovrtx`, `isaacsim_rtx` and `rtx`

## Manager terms

`ObservationTermCfg` (`ObsTerm`):

- `func`, `params`, `modifiers`, `noise` (`NoiseCfg` or `NoiseModelCfg`), `clip`, `scale`.
- `delay_min_lag`, `delay_max_lag`, `delay_hold_prob`: latency in observation samples.
- `history_length`, `flatten_history_dim`.

`ObservationGroupCfg` (`ObsGroup`): `concatenate_terms` (True), `concatenate_dim` (-1),
`enable_corruption` (False; noise is off until True), `history_length`, `history_order`.

Noise cfgs (`isaaclab.utils.noise`): `ConstantNoiseCfg`, `UniformNoiseCfg(n_min, n_max)`,
`GaussianNoiseCfg(mean, std)`, `NoiseModelCfg`, `NoiseModelWithAdditiveBiasCfg`.

`EventTermCfg` (`EventTerm`):

- `func`, `params`, `mode` (`prestartup`, `startup`, `reset`, `interval`, or custom).
- For `interval`: `interval_range_s`, `is_global_time`, `resample_interval_on_reset`.
- For `reset`: `min_step_count_between_reset`.

`RewardTermCfg`: `func`, `params`, `weight`. The manager multiplies by the step dt.
`TerminationTermCfg`: `func`, `params`, `time_out` (True = truncation). `SceneEntityCfg(name,
joint_names=, body_names=)` resolves `joint_ids`/`body_ids` once at startup.

Action cfgs in `isaaclab.envs.mdp` include `JointPositionActionCfg(asset_name, joint_names,
scale, use_default_offset)`, `JointEffortActionCfg` and `JointVelocityActionCfg`, plus
differential IK and OSC actions for task-space control (needed by Mimic). Read
`isaaclab/envs/mdp/actions/` for the current list.

## Built-in event functions (`isaaclab/envs/mdp/events.py`)

| Function | Typical mode | Key params |
|----------|--------------|------------|
| `randomize_rigid_body_material` | startup | `asset_cfg`, `static_friction_range`, `dynamic_friction_range`, `restitution_range`, `num_buckets` (PhysX buckets; Newton ignores dynamic friction and buckets) |
| `randomize_rigid_body_mass` | startup | `mass_distribution_params`, `operation` (`add`/`scale`/`abs`), `distribution` (`uniform`/`log_uniform`/`gaussian`) |
| `randomize_rigid_body_com` | startup | `com_range={"x":(),"y":(),"z":()}` (Newton: position only) |
| `randomize_rigid_body_inertia`, `randomize_rigid_body_scale` | startup / prestartup | Scale must be prestartup |
| `randomize_rigid_body_collider_offsets` | startup | PhysX rest/contact offsets; Newton margin/gap |
| `randomize_actuator_gains` | startup/reset | `stiffness_distribution_params`, `damping_distribution_params`, `operation` (default `abs`), `distribution` |
| `randomize_joint_parameters` | startup/reset | Friction, armature, limits |
| `randomize_fixed_tendon_parameters` | startup | |
| `randomize_physics_scene_gravity` | reset/interval | |
| `apply_external_force_torque` | reset | `force_range`, `torque_range` |
| `push_by_setting_velocity` | interval | `velocity_range={"x":(),"y":()}` |
| `reset_root_state_uniform` | reset | `pose_range`, `velocity_range` (x, y, z, roll, pitch, yaw) |
| `reset_root_state_with_random_orientation`, `reset_root_state_from_terrain` | reset | |
| `reset_joints_by_offset`, `reset_joints_by_scale`, `reset_joints_within_limits_range` | reset | `position_range`, `velocity_range` |
| `reset_scene_to_default` | reset | `reset_joint_targets` |
| `randomize_visual_color`, `randomize_visual_texture_material` | reset | Rendering domain randomisation |

Curriculum and ADR: `CurriculumTermCfg` with `modify_term_cfg`. See `isaaclab_tasks/core/lift/
adr_curriculum.py` and `docs/source/how-to/curriculums.rst`.
