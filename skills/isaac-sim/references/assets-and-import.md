# Assets and Import: USD, Robot Configs, URDF/MJCF

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.

## USD essentials

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

## Ready-made robot configs

`stiffness=None`/`damping=None` keep the USD-authored gains. Ready-made configs live in
`isaaclab_assets.robots` (`FRANKA_PANDA_CFG`, `UR10_CFG`, `ANYMAL_D_CFG`, `UNITREE_GO2_CFG`,
`G1_CFG`, `H1_CFG`, `SPOT_CFG`, `SO101_CFG` and others). Read the matching file before you copy
one.

## URDF and MJCF importer gotchas

The convert commands are in SKILL.md (Assets and Import).

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
