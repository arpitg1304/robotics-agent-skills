# Performance and Debugging

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.
The debug order and the most common rows also stay in SKILL.md.

## Physics settings that matter

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

## Full symptom table

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
