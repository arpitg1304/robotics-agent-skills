# MJCF Reference (MuJoCo 4.0.1)

Defaults below were read from a compiled `MjModel` in MuJoCo 4.0.1 or from
`doc/XMLreference.rst` at the 4.0.1+ main branch (2026-11-18). "Recommended" values are the
starting points from the official *Modeling > Model tuning* guide: heuristics, not laws.
For anything not listed, grep the installed docs:

```bash
python - <<'EOF'
import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).parent / "doc" / "XMLreference.rst")
EOF
grep -n "^.. _body-geom-solmix:" -A12 <that path>
```

## `<option>`

| Attribute | Default | Recommended / notes |
|---|---|---|
| `timestep` | `0.002` | 0.5-2 ms contact-rich manipulation; 5-15 ms locomotion |
| `integrator` | `Euler` | `implicitfast`. Values: `Euler`, `RK4`, `implicit`, `implicitfast`, `discrete` |
| `cone` | `pyramidal` | `elliptic` for grasping and feet |
| `impratio` | `1` | `10` manipulation, `10-100` legs (elliptic only) |
| `solver` | `Newton` | `CG` only when Hessian factorisation dominates; `PGS` rarely |
| `iterations` | `100` | Warm-started Newton usually stops in 1-4; lower on MJX-JAX |
| `ls_iterations` | `50` | Lower on MJX-JAX |
| `tolerance` | `1e-8` | `1e-10` for accuracy, looser for speed |
| `ls_tolerance` | `0.01` | |
| `noslip_iterations` | `0` | `1-3` for residual slow slip (CPU, MuJoCo Warp; ignored by MJX-JAX) |
| `noslip_tolerance` | `1e-6` | |
| `ccd_iterations` / `ccd_tolerance` | `35` / `1e-6` | Convex collision solver |
| `jacobian` | `auto` | dense if `nv <= 60`, sparse above |
| `gravity` | `0 0 -9.81` | +Z up |
| `density` / `viscosity` / `wind` | `0` | Fluid forces off unless set |
| `sleep_tolerance` | `0.001` | Used when the `sleep` flag is enabled |

`<option><flag .../></option>` (enable/disable). Defaults in 4.0.1: `warmstart`, `island`,
`nativeccd`, `multiccd`, `eulerdamp`, `autoreset`, `filterparent`, `midphase`, `refsafe`
enabled; `sleep`, `speculative`, `energy`, `fwdinv`, `diagexact`, `override`, `ipc`
disabled. Runtime equivalents: `mujoco.mjtDisableBit.mjDSBL_*` in `model.opt.disableflags`
and `mujoco.mjtEnableBit.mjENBL_*` in `model.opt.enableflags`. `multiccd` became a
default-on (disable) flag; old code that sets an `mjENBL_MULTICCD` bit no longer exists.

`override` + `o_solref`, `o_solimp`, `o_margin`, `o_friction` replace all contact parameters
globally: handy for sweeping softness in the viewer, not for shipping models.

## `<compiler>`

| Attribute | Default (MJCF) | Default (URDF) | Notes |
|---|---|---|---|
| `angle` | `degree` | `radian` (forced) | Set `radian` explicitly in MJCF |
| `autolimits` | `true` | | `range` implies `limited` |
| `meshdir`, `texturedir`, `assetdir` | unset | | Relative to the main model file |
| `strippath` | `false` | `false` in 4.0.1 (older releases: true) | Strips directories; leaves registered provider URIs alone |
| `discardvisual` | `false` | `true` | Drops contype=conaffinity=0 geoms, materials, textures |
| `fusestatic` | `false` | `true` | Merges jointless bodies into parents |
| `inertiafromgeom` | `auto` | | `true` overrides `<inertial>`; `false` requires it |
| `inertiagrouprange` | `0 5` | | Geom groups used for inertia inference |
| `boundmass` / `boundinertia` | `0` | | Lower bounds for messy imports |
| `balanceinertia` | `false` | | Fix triangle-inequality violations |
| `eulerseq` | `xyz` | | URDF `rpy` corresponds to `XYZ` |
| `alignfree` | `false` | | Align free bodies with their inertial frame |
| `settotalmass` | `-1` | | Deprecated; warns |

## `<geom>`

| Attribute | Default | Notes |
|---|---|---|
| `type` | `sphere` | plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh, sdf |
| `contype` / `conaffinity` | `1` / `1` | Collide iff `(ct1 & ca2) || (ct2 & ca1)` |
| `condim` | `3` | 1 frictionless, 3 sliding, 4 + torsional, 6 + rolling. Pair uses the max |
| `friction` | `1 0.005 0.0001` | sliding, torsional (length), rolling (length). Pair uses element-wise max |
| `priority` | `0` | Higher priority geom's parameters win |
| `solmix` | `1` | Weighting of solref/solimp between equal-priority geoms |
| `solref` | `0.02 1` | timeconst, dampratio; or `-stiffness -damping`. Rigid: 0.004-0.01, >= 2 x timestep |
| `solimp` | `0.9 0.95 0.001 0.5 2` | dmin dmax width midpoint power |
| `margin` / `gap` | `0` / `0` | Contact active within margin; hovering if margin without gap |
| `density` | `1000` | Used when `mass` is absent |
| `mass` | unset | Overrides density |
| `group` | `0` | Visibility groups 0-5; convention: 2 visual, 3 collision (Menagerie) |
| `castshadow` | `true` | New in 4.0 |
| `fromto` | unset | Capsule/cylinder/box/ellipsoid between two points |

Meshes: `<mesh file="..." scale="..." maxhullvert="-1"/>`. A mesh collides as its convex
hull; `maxhullvert` (default -1, unlimited, must be > 3) caps hull size. `inertia="shell"`
for thin or non-watertight meshes. Meshes in mm need `scale="0.001 0.001 0.001"`.

## `<joint>`

| Attribute | Default | Notes |
|---|---|---|
| `type` | `hinge` | hinge, slide, ball, free (`<freejoint/>` shortcut) |
| `axis` | `0 0 1` | Local frame |
| `range` | `0 0` | Units follow `compiler/angle` |
| `armature` | `0` | Reflected inertia I_rotor x gear^2; set on every geared joint |
| `damping` | `0 0 0` | Linear, quadratic, cubic. XML scalar OK; MjSpec wants a 3-vector |
| `stiffness` | `0 0 0` | Polynomial spring around `springref` |
| `frictionloss` | `0` | Dry friction, one constraint row per DoF |
| `actuatorfrcrange` | unset | Clamps the sum of actuator forces on the joint (URDF `effort` lands here) |
| `solreflimit` / `solimplimit` | `0.02 1` / default solimp | Joint limits are soft too |
| `margin` | `0` | Limit activation distance |

`<freejoint align="auto"/>` (default `auto`). `body/gravcomp` (0-1) cancels gravity on a body,
useful to emulate gravity-compensated arms.

## Actuators

All shortcuts expand to `<general>` with `gaintype`, `biastype`, `dyntype`, `gainprm`,
`biasprm`, `dynprm`. Common attributes: `joint` / `tendon` / `site` (+ `refsite`) /
`body` transmission, `gear` (`1 0 0 0 0 0`), `ctrlrange`, `forcerange`, `ctrllimited`,
`forcelimited` (inferred with `autolimits`), `lengthrange`, `armature`, `damping`
(both reflected by gear^2), `delay` + `nsample` + `interp` (control latency), `group`.

| Shortcut | Key attributes (default) | Compiles to |
|---|---|---|
| `motor` | | gain 1, no bias: force = gear x ctrl |
| `position` | `kp` (1), `kv` (0) or `dampratio` (0), `timeconst` (0), `inheritrange` (0) | `gainprm[0]=kp`, `biasprm=[0, -kp, -kv]` |
| `velocity` | `kv` (1) | `gainprm[0]=kv`, `biasprm=[0, 0, -kv]` |
| `intvelocity` | `kp`, `actrange` | integrator activation + position servo |
| `damper` | `kv` | force = -kv x velocity x ctrl |
| `pid` | `kp`, `kv`, `ki`, `imax`, `slewmax`, `input` subset of `[pos, vel, ff]` | pid gain/dyn types |
| `dcmotor` | `resistance`, `motorconst`, `nominal`, `inductance`, `input`, ... | dcmotor types |
| `cylinder`, `muscle`, `adhesion`, `orientation` | see XML reference | |

`dampratio` is converted to `kv` at compile time from the reflected inertia at `qpos0`
(including armature, excluding passive damping/frictionloss). Changing mass or `kp` at
runtime does not update it. Prefer one `pid` actuator over stacking position + velocity +
motor actuators on the same joint.

MjSpec: `act = spec.add_actuator(name=..., target="joint_name", trntype=mujoco.mjtTrn.mjTRN_JOINT)`
then `act.set_to_position(kp=..., kv=...)`. Also `set_to_motor`, `set_to_velocity`,
`set_to_intvelocity`, `set_to_damper`, `set_to_pid`, `set_to_dcmotor`, `set_to_cylinder`,
`set_to_muscle`, `set_to_adhesion`, `set_to_orientation`.

## Sensors

Element names in 4.0.1: `touch accelerometer velocimeter gyro force torque magnetometer
rangefinder camprojection jointpos jointvel tendonpos tendonvel actuatorpos actuatorvel
actuatorfrc jointactuatorfrc tendonactuatorfrc ballquat ballangvel jointlimitpos
jointlimitvel jointlimitfrc tendonlimitpos tendonlimitvel tendonlimitfrc framepos framequat
framexaxis frameyaxis framezaxis framelinvel frameangvel framelinacc frameangacc subtreecom
subtreelinvel subtreeangmom insidesite distance normal fromto contact tactile clock user
plugin`.

Common sensor attributes: `name`, `noise` (metadata only, never applied), `cutoff` (clamps
absolute value), `nsample`, `delay`, `interval` (periodic sampling), `interp`, `user`.

`frame*` sensors take `objtype` (body, xbody, geom, site, camera) + `objname`, and optional
`reftype`/`refname` to express the result in another frame. `force`/`torque` sensors measure
the interaction between a site's body and its parent, in the site frame.

Contact sensor:

```xml
<contact name="pad_obj" geom1="left_pad" geom2="cube" num="2"
         data="found force" reduce="maxforce"/>
```

- Match by `geom1/geom2`, `body1/body2`, `subtree1/subtree2`, or a `site` volume.
- `data` is an ordered subset of `found force torque dist pos normal tangent linvel angvel`
  (must keep that order). `force`/`torque`/`linvel`/`angvel` are in the contact frame (normal
  first); `pos`/`normal`/`tangent` in world frame.
- `reduce`: `mindist`, `maxforce`, `netforce` (single combined wrench).
- `found` holds the number of matching contacts in every filled slot, 0 for empty slots.
- MJX-JAX supports only geom-geom or unfiltered matching with `mindist`/`maxforce`.

## Equality constraints

`connect` (body1, body2 or site1/site2, anchor), `weld` (body1, body2 or sites, optional
`relpose`, `torquescale`), `joint` (joint1, joint2, `polycoef`), `tendon`, `flex`,
`flexstrain`, and (in the enum) `distance`. All accept `active`, `solref`, `solimp`.
Toggle at runtime with `data.eq_active[i]`. A weld from a `mocap="true"` body to a free body
is the stable way to teleoperate or kinematically drive an object; tune its `solref` so it
yields to contacts.

## Contact element

```xml
<contact>
  <exclude body1="base" body2="link1"/>
  <pair geom1="finger" geom2="cube" condim="4" friction="1.2 1.2 0.01 0.0001 0.0001"
        solref="0.005 1"/>
</contact>
```

`pair` friction has 5 values: two sliding (anisotropic), torsional, two rolling. Pairs
bypass `contype`/`conaffinity` and are the main lever for MJX-JAX contact budgets.

## Keyframes

`<key name time qpos qvel act ctrl mpos mquat plastic/>`. Mocap fields are `mpos` and
`mquat`, not `mocap_pos`. Missing values fall back to `qpos0` / zeros / body pose.
`mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)` then `mj_forward`.

## Units and conventions

MuJoCo has no fixed unit system but its defaults assume MKS (density 1000, gravity 9.81).
Quaternions `[w, x, y, z]`. Cameras look along local -Z, +Y up, `fovy` in degrees (default 45).
Only the last `<visual>` block counts; `offwidth`/`offheight` default 640 x 480.

## URDF conversion recipe

1. Add the extension block as the first child of `<robot>`:
   ```xml
   <mujoco>
     <compiler meshdir="meshes" strippath="true" discardvisual="false"
               fusestatic="false" balanceinertia="true"/>
   </mujoco>
   ```
   It is not schema-checked: a typo is silently ignored. `strippath="true"` + `meshdir`
   is the simplest way to resolve `package://pkg/meshes/x.stl` (verified in 4.0.1).
2. Load and save: `spec = mujoco.MjSpec.from_file("robot.urdf"); open("robot.xml","w").write(spec.to_xml())`.
3. In the MJCF: set `<compiler angle="radian"/>` (URDF values are radians), `<option>`
   integrator/cone, move shared values into `<default>` classes, put visual meshes in a
   non-colliding class, replace collision meshes with primitives or convex parts, add
   `exclude` for adjacent links that touch at `qpos0`.
4. Add actuators. URDF `effort` became joint `actuatorfrcrange`; either keep it or move it to
   actuator `forcerange`. Add `armature`, `damping`, `frictionloss` from datasheets or sysid.
5. Check: `nu` as expected, no contacts at `qpos0` (`mj_forward`, inspect `data.ncon`),
   inertia boxes plausible in the viewer, mass total matches the real robot.
6. Commit the MJCF; treat the URDF as an input, not the source of truth.

Tools: Menagerie READMEs document each model's conversion steps and are the best examples;
`mujoco.MjSpec` replaces dm_control PyMJCF for programmatic editing; `spec.attach(...)`
composes robots, grippers and scenes with name prefixes.

## Rules that prevent most modelling bugs (full text)

- **Angles**: `compiler/angle` defaults to `"degree"` for MJCF (always radian for URDF).
  `range="-1.57 1.57"` without `angle="radian"` is +/-1.57 degrees. Set it explicitly.
- **Asset paths**: `meshdir`/`texturedir`/`assetdir` are relative to the main model file.
  Only the last `<visual>` block in a file counts; merge them.
- **Mass and inertia**: `inertiafromgeom="auto"` (default) infers mass and inertia from geoms
  only when a body has no `<inertial>`, using `density="1000"` (water) unless a geom sets
  `mass`. Every geom in `inertiagrouprange` (default `"0 5"`) counts, including visual
  geoms with `contype=conaffinity=0`: a body with a collision sphere (group 3) and an
  identical visual sphere (group 2) compiled to **twice** the mass (verified). Fix with
  `inertiagrouprange`, `density="0"` or `mass="0"` on one set, or explicit `<inertial>` from
  CAD (preferred for real robots). `inertiafromgeom="true"` overrides imported nonsense
  inertias; `boundmass`/`boundinertia` clamp tiny URDF dummy links; `balanceinertia` fixes
  triangle-inequality violations.
- **Defaults classes**: `<default class="x">` nests; children inherit from parents;
  elements select with `class="x"`, and `childclass` on a body applies to its subtree. The
  top-level default is the place to change MuJoCo's dated physics defaults once.
- **Joints**: `type` is `hinge` (default), `slide`, `ball`, `free` (or `<freejoint/>`).
  `armature` (reflected rotor inertia, I_rotor x gear^2) and `damping` default to 0 and should
  almost never be 0 on a geared robot joint. Since 3.7.0 `damping` and `stiffness` accept up
  to three polynomial coefficients; XML `damping="0.1"` still works, but in `MjSpec` assign a
  3-vector (`joint.damping = [0.1, 0, 0]`; a scalar raises `TypeError`). `frictionloss` (dry friction) adds a constraint
  row per joint; use it where stiction matters.
- **Actuators** (shortcuts of `<general>`): `motor` (torque = gear x ctrl), `position`
  (`kp` default 1, `kv` default 0, or `dampratio`; `timeconst` adds a filter;
  `inheritrange="1"` copies the joint range into `ctrlrange`), `velocity`, `intvelocity`,
  `damper`, `cylinder`, `muscle`, `adhesion`, plus newer `pid` (pos/vel setpoints, `ki`,
  `imax`, `slewmax`), `dcmotor` and `orientation`. A position actuator compiles to
  `gainprm[0]=kp`, `biasprm[1:3]=[-kp, -kv]`. Always set `forcerange` (unlimited by default)
  and keep `ctrlrange` inside the joint `range` except on grippers. Actuators also accept
  `armature`, `damping` (reflected through `gear`), and `delay`/`nsample` for control latency.
- **Collision filtering**: two geoms can collide iff
  `(contype1 & conaffinity2) || (contype2 & conaffinity1)`; both default to 1. Geoms in the
  same body never collide; parent-child bodies are excluded by default (`filterparent`).
  Use `<contact><exclude body1 body2/></contact>` for specific pairs and `<pair>` to
  force a pair with custom `condim`/`friction` (5 values)/`solref`/`solimp`.
- **Equality constraints**: `connect` (ball joint between bodies), `weld` (6-DoF lock; the
  standard way to drive a free body from a `mocap="true"` body), `joint` (polynomial joint
  coupling, e.g. mirrored gripper fingers), `tendon`, `flex`. Soften with `solref`/`solimp`.
- **Keyframes**: `<key name qpos qvel act ctrl mpos mquat time/>`; note mocap fields are
  `mpos`/`mquat`. Missing entries fall back to `qpos0`/zeros. Re-record keyframes whenever
  you add or remove joints: `qpos` is positional.
- **Sensors** write fixed slots into `data.sensordata`. Use them (not `data.contact`) for
  observations: `<contact>` sensors (since 3.3.5) give a fixed-size, filtered, sorted view of
  contacts (`geom1/geom2/body1/body2/subtree1/site`, `num`, `data="found force ..."`,
  `reduce="mindist|maxforce|netforce"`); `touch` sums normal force inside a site volume.
  The sensor `noise` attribute **does not affect simulation**; it only stores a standard
  deviation. Add noise yourself.
