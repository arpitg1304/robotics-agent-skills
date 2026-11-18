---
name: mujoco
description: >
  MuJoCo physics simulation for robotics: writing and debugging MJCF models, importing URDF,
  simulating, controlling and rendering from Python, tuning contacts, and running batched GPU
  simulation. Use this skill when the user mentions MuJoCo, MJCF, mujoco.MjModel / MjData,
  mj_step, MjSpec, mujoco.Renderer, MUJOCO_GL, EGL/OSMesa headless rendering, the passive
  viewer or MuJoCo Studio, MuJoCo Menagerie (Franka, UR5e, SO-100/SO-101, ALOHA, G1),
  MJX, MuJoCo Warp (mujoco_warp), MuJoCo Playground, dm_control, or Gymnasium MuJoCo envs.
  Also trigger for contact tuning (solref, solimp, condim, friction, impratio, noslip),
  exploding or jittering simulations, objects sinking or grasps slipping, position actuator
  kp/kv, joint armature and damping, contype/conaffinity collision filtering, keyframes,
  integrator and timestep choice, and system identification or domain randomisation of
  MuJoCo model fields for sim-to-real.
---

# MuJoCo Skill

Verified against MuJoCo 4.0.1, 2026-11-18. Examples were executed with `mujoco==4.0.1`,
`mujoco-mjx==4.0.1` (JAX 0.12.1, CPU), `mujoco-warp==4.0.1` (Warp 1.19.0, CPU device) and
`gymnasium==1.4.0` on Python 3.12, macOS arm64.

MuJoCo moves fast (4.0.0 shipped 2026-11-06 with breaking changes; a new `discrete`
integrator and speculative contacts landed in autumn 2026). Before relying on a detail that
is not in this skill, check the installed docs: `pip install mujoco` bundles the Sphinx
manual and Google's own agent skills under `pathlib.Path(mujoco.__file__).parent / "doc"`
(`doc/skills/python`, `rendering`, `accelerated`, `spec_editing`, ...). Grep there for the
literal `.. _option-integrator:`-style anchor of an attribute.

## When to Use This Skill
- Writing or fixing an MJCF scene: bodies, joints, actuators, sensors, defaults, keyframes
- Converting a URDF to MJCF and making it simulate well
- Writing a Python control loop, reading sensors, resetting episodes, saving state
- Rendering RGB / depth / segmentation offscreen, including on headless Linux
- Diagnosing exploding, jittering, sinking, tunneling or slipping simulations
- Choosing between CPU MuJoCo, `mujoco.batch`, MJX and MuJoCo Warp for parallel rollouts
- Identifying actuator gains, armature and friction from hardware logs; randomising model fields

Out of scope here: RL algorithms, reward design and sim-to-real theory
(`robot-reinforcement-learning`); Isaac Sim / Isaac Lab (`isaac-sim`); LeRobot datasets and
policies (`lerobot`, `imitation-learning`).

## Ecosystem Map

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

Menagerie arms relevant to low-cost manipulation (directory / main file):
`robotstudio_so101/so101.xml` (SO-101, 6 position actuators, `scene.xml`, `scene_box.xml`),
`trs_so_arm100/so_arm100.xml` (SO-ARM100 5-DoF), `low_cost_robot_arm/`, `i2rt_yam/`,
`trossen_wxai/`, `trossen_vx300s/`, `trossen_wx250s/`, `aloha/`, `agilex_piper/`,
`arx_l5/`. Industrial: `franka_emika_panda/`, `franka_fr3/`, `universal_robots_ur5e/`,
`kuka_iiwa_14/`, `kinova_gen3/`, `ufactory_xarm7/`. Some models ship `*_mjx.xml` variants
tuned for MJX. Each README states the minimum MuJoCo version and the derivation steps.

Choosing a backend (heuristic): one env or real-time loop -> CPU `mujoco`; tens to a few
thousand envs on a many-core box, or you need `noslip`/PGS/plugins/float64 ->
`mujoco.batch`; thousands of envs on an NVIDIA GPU -> MuJoCo Warp (or MJX with
`impl="warp"` inside JAX); gradients through dynamics, TPU, AMD or Apple GPU -> MJX-JAX.

## MJCF Essentials

### A complete, runnable model

This model (executed, see the loop below) shows the patterns that matter: explicit angle
units, `implicitfast`, a default class tree that separates visual and collision geoms,
armature and damping on joints, a force-limited critically damped position servo, sensors
and a keyframe.

```xml
<mujoco model="one_link_arm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>

  <default>
    <joint damping="0.1" armature="0.01"/>
    <position kp="20" dampratio="1" forcerange="-5 5"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom group="3"/>
    </default>
  </default>

  <worldbody>
    <light pos="0 0 2"/>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <camera name="side" pos="0 -1.2 0.4" xyaxes="1 0 0 0 0.3 1"/>
    <body name="link" pos="0 0 0.3">
      <joint name="shoulder" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
      <geom class="collision" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="0.5"/>
      <geom class="visual" type="capsule" fromto="0 0 0 0.3 0 0" size="0.025"/>
      <site name="tip" pos="0.3 0 0"/>
    </body>
  </worldbody>

  <actuator>
    <position name="shoulder" joint="shoulder" ctrlrange="-1.5 1.5"/>
  </actuator>

  <sensor>
    <jointpos name="shoulder_pos" joint="shoulder"/>
    <jointvel name="shoulder_vel" joint="shoulder"/>
    <framepos name="tip_pos" objtype="site" objname="tip"/>
  </sensor>

  <keyframe>
    <key name="home" qpos="0" ctrl="0"/>
  </keyframe>
</mujoco>
```

### Rules that prevent most modelling bugs

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

### URDF import

```python
spec = mujoco.MjSpec.from_file("robot.urdf")   # or MjModel.from_xml_path for a quick look
print(spec.compile().nu)                         # 0: URDF has no actuator model
```

What goes wrong and the fix:

| Symptom | Cause | Fix |
|---|---|---|
| `nu == 0`, robot collapses | URDF has no actuators; `<limit effort>` becomes joint `actuatorfrcrange` only | Add actuators in MJCF/MjSpec (example below) |
| `Error opening file 'package:/...'` | `package://` URIs are not resolved by release builds | `<mujoco><compiler meshdir="meshes" strippath="true"/></mujoco>` inside `<robot>`, or rewrite paths (verified) |
| Visual meshes missing | URDF defaults `discardvisual="true"` | `<compiler discardvisual="false"/>` |
| Links merged, sites/cameras lost | URDF defaults `fusestatic="true"` | `fusestatic="false"` or attach to a moving body |
| Contacts slow and jittery | Detailed STL collision meshes, collided as convex hulls | Replace with primitives or convex decomposition; cap `maxhullvert` |
| Unstable, tiny links | Dummy links with ~1e-6 inertia, mass ratios > 1000:1 | `boundmass`/`boundinertia`, fix at source |
| Typos silently ignored | The `<mujoco>` block inside URDF is **not** schema-checked | Convert once, then edit and version the MJCF |

In 4.0.1 a URDF loads with `strippath=false`, `fusestatic=true`, `discardvisual=true`
(checked empirically); older Menagerie conversions set `strippath="false"` explicitly, so
set it explicitly rather than relying on the version. The URDF `<mujoco>` extension accepts `compiler`,
`option` and `size`. Recommended workflow (MuJoCo docs): add extensions, load, save as MJCF,
then add actuators, defaults and sensors in the MJCF and keep that as the source of truth.

```python
# Executed: add what URDF cannot express, then save MJCF.
spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
j1 = spec.joint("j1")
j1.armature = 0.01
j1.damping = [0.05, 0, 0]
act = spec.add_actuator(name="j1", target="j1", trntype=mujoco.mjtTrn.mjTRN_JOINT)
act.set_to_position(kp=30, kv=1.0)      # also set_to_motor, set_to_pid, set_to_velocity, ...
act.ctrlrange = [-1.5, 1.5]
act.forcerange = [-10, 10]
model = spec.compile()
open("robot.xml", "w").write(spec.to_xml())
```

Composition: `parent.attach(child_spec, prefix="hand/", site=parent.site("attachment_site"))`
attaches one spec to another (used by Menagerie to put hands on arms).

Load `references/mjcf-reference.md` for attribute tables with verified defaults (option,
compiler, geom, joint, actuator, sensor, equality) and the full URDF conversion recipe.

## Python Simulation Loop

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_string(XML)     # or from_xml_path / MjSpec(...).compile()
data = mujoco.MjData(model)

CONTROL_HZ = 50
n_substeps = round(1.0 / (CONTROL_HZ * model.opt.timestep))   # 10 physics steps per action
assert abs(n_substeps * model.opt.timestep - 1.0 / CONTROL_HZ) < 1e-9

mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)          # kinematics + sensors at t=0, time not advanced

act = data.actuator("shoulder")         # named views, no index bookkeeping
for k in range(100):
    act.ctrl[0] = 0.8 * np.sin(2 * np.pi * 0.5 * data.time)
    mujoco.mj_step(model, data, nstep=n_substeps)   # ctrl held (zero-order hold)

q = data.sensor("shoulder_pos").data[0]
tip = data.sensor("tip_pos").data.copy()            # copy: views are overwritten in place
assert np.isfinite(data.qpos).all()
```

Key semantics:

- `mj_step` = forward dynamics at `data.time`, then integrate to `time + timestep`.
  **After `mj_step`, `sensordata`, `xpos`, contacts etc. describe the state at the start of
  that step**, one step behind `qpos` (verified: qpos 2.6765 vs jointpos sensor 2.6635 for a
  swinging link). Call `mj_forward` before reading if you need them consistent with `qpos`,
  or use `mj_step1` / set `ctrl` / `mj_step2` so the controller sees fresh quantities
  (single-step integrators only; RK4 needs `mjcb_control`).
- `mj_forward` computes everything without integrating. Call it after editing `qpos`,
  `qvel`, `mocap_pos` or after a reset, before reading positions or sensors.
- `mj_step(m, d, nstep=N)` repeats with the same `ctrl` and releases the GIL once.
- Named access: `model.body("x")`, `data.joint("j").qpos`, `data.actuator("a").ctrl`,
  `data.sensor("s").data`, `model.key("home").id`. Accessors exist for body, geom, joint,
  site, camera, light, actuator, sensor, tendon, equality, keyframe, mesh, and more. With
  `MjSpec` use `data.bind(spec_element)`. Writes go through views, so `.copy()` when logging.
- Never write `xpos`, `xquat`, `sensordata`: they are outputs. Write `qpos`, `qvel`, `ctrl`,
  `qfrc_applied`, `xfrc_applied`, `mocap_pos`, `mocap_quat`.
- Quaternions are scalar-first `[w, x, y, z]`; SciPy and ROS are scalar-last.
- Free-joint `qpos` is 7 numbers (pos + quat) but `qvel` is 6, so `nq != nv`. Use
  `model.jnt_qposadr` / `jnt_dofadr` or named access instead of hard-coded slices.

Reset and state save/restore (executed):

```python
mujoco.mj_resetData(model, data)                      # to qpos0, zero velocity, time 0
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

spec = mujoco.mjtState.mjSTATE_INTEGRATION     # everything needed for exact continuation
state = np.empty(mujoco.mj_stateSize(model, spec))
mujoco.mj_getState(model, data, state, spec)
mujoco.mj_step(model, data, nstep=50)
mujoco.mj_setState(model, data, state, spec)
mujoco.mj_forward(model, data)
```

`mjSTATE_FULLPHYSICS` = time + qpos, qvel, act, history, plastic, plugin state. It excludes
`ctrl`, applied forces, mocap poses and solver warm-start; `mjSTATE_INTEGRATION` adds them.
Verified: restoring FULLPHYSICS (plus ctrl) continued with a 1e-16 difference, INTEGRATION
bit-identically. Use FULLPHYSICS for compact logging, INTEGRATION for checkpoint/replay.

Determinism: the same binary, model, initial state and control sequence reproduce
bit-identical trajectories on CPU (asserted in the executed example; `mujoco.batch` is
documented as bit-identical to serial stepping). Things that break it: resetting by writing
`qpos`/`qvel`/`time` instead of calling `mj_resetData` (solver warm-start carries over;
measured 3e-6 divergence after 300 contact steps, versus bit-identical with `mj_resetData`),
different library versions or builds, GPU float32 backends, and unseeded Python randomness.

Interactive viewing:

```python
import time, mujoco, mujoco.viewer
with mujoco.viewer.launch_passive(model, data) as viewer:   # macOS: run with `mjpython`
    while viewer.is_running():
        t0 = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()                         # push state to the viewer, pull perturbations
        time.sleep(max(0, model.opt.timestep - (time.time() - t0)))
```

Modify viewer options under `with viewer.lock():`. `python -m mujoco.viewer --mjcf=scene.xml`
opens a file directly; MuJoCo Studio (`python -m mujoco.studio.viewer --model=scene.xml
--gfx=web`) streams to a browser from a headless host.

## Rendering

```python
import os
# os.environ["MUJOCO_GL"] = "egl"   # headless Linux + NVIDIA; "osmesa" = CPU software. BEFORE import.
import mujoco

with mujoco.Renderer(model, height=480, width=640) as renderer:   # create ONCE, reuse
    renderer.update_scene(data, camera="overhead")    # name, id, or mujoco.MjvCamera
    rgb = renderer.render()                           # (480, 640, 3) uint8

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="overhead")
    depth = renderer.render()                         # (480, 640) float32, metres
    renderer.disable_depth_rendering()

    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="overhead")
    seg = renderer.render()                           # (480, 640, 2) int32: [obj id, mjtObj type]
    renderer.disable_segmentation_rendering()

box = model.geom("box_geom").id
mask = (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (seg[..., 0] == box)
```

Executed: depth at the image centre of a camera 1.5 m above a 0.2 m box returned 1.3, and
the segmentation id matched the box geom id. Notes:

- `MUJOCO_GL` values (from the bindings source): Linux `egl`, `osmesa`, `glx`, `glfw`;
  macOS `cgl` (default) or `glfw`; Windows `wgl`/`glfw`; `disable` turns GL off. It is read
  at import time, so set it before `import mujoco` (and before any library that imports it).
  On EGL multi-GPU nodes `MUJOCO_EGL_DEVICE_ID` selects the device (a visible-device index or
  CUDA id, ordered like `CUDA_VISIBLE_DEVICES`). `PYOPENGL_PLATFORM`, if set, must be `egl`
  for the EGL backend and `osmesa` for OSMesa, or the import fails.
- Width/height cannot exceed `<visual><global offwidth offheight/>` (default 640x480).
- Cameras look along their local -Z with +Y up. Use `xyaxes`, `zaxis` or `mode="targetbody"`.
  `fovy` is vertical field of view in degrees.
- `render()` returns a fresh array by default; pass `out=` to reuse a buffer. Rendering
  is much slower than physics: render at the policy rate, not every substep.
- Collision geoms are often in an invisible group; set `MjvOption.geomgroup` and pass
  `scene_option=` to `update_scene` to see them.
- The Filament PBR renderer (`mujoco.rendering.filament`, `mujoco.mjrf`) is the new path for
  photoreal images; `mujoco.Renderer` (classic OpenGL) is in maintenance mode but simplest.
  For thousands of cameras on GPU use the MuJoCo Warp batch renderer (see reference).

## Contacts and Stability

MuJoCo contacts are soft constraints: a small, controlled penetration is how they work.
Tune in this order (from the official "Model tuning" guide; numbers are its starting points,
i.e. heuristics):

1. **Fix modelling bugs first**: initial interpenetration, visual/collision mismatch, concave
   meshes collided as hulls, implausible inertias, mm vs m mesh scale, degrees vs radians,
   position targets driving into limits or surfaces.
2. **Integrator**: default is `Euler`. Use `implicitfast` for almost everything (same cost,
   implicit in `kv`/damping). `implicit` for fast-spinning coupled chains, `RK4` for
   near-energy-conserving systems. `discrete` (new, Sept 2026, still changing) is implicit in
   position too and stays stable with stiff servos or springs against contacts; it is not
   supported by MJX-JAX.
3. **Timestep**: default 2 ms. Contact-rich manipulation often wants 0.5-2 ms; locomotion
   tolerates 5-15 ms. Throughput scales linearly with timestep up to divergence.
4. **Friction cone**: `cone="elliptic"` with `impratio="10"` for manipulation (10-100 for legs);
   avoid large `impratio` with the default `pyramidal` cone.
5. **solref** (`"timeconst dampratio"`, default `"0.02 1"`): 20 ms is soft. Rigid contacts:
   0.004-0.01, keep `timeconst >= 2 * timestep`. Negative form `"-stiffness -damping"` sets
   them directly (`solref="-1000 0"` gives elastic bounce).
6. **solimp** (`"dmin dmax width midpoint power"`, default `"0.9 0.95 0.001 0.5 2"`): rest
   penetration and creep scale with `1 - dmin`; `0.95` halves them at some solver cost.
   `solimp="0.01 ..."` almost disables the constraint; do not confuse it with `solref`.
7. **condim / friction** (default `3`, `"1 0.005 0.0001"`): `condim="4"` adds torsional and
   `6` adds rolling friction; use on grasp pads and round objects. Torsional/rolling
   coefficients are lengths. Contacts take the **max** condim and element-wise **max**
   friction of the two geoms unless one has higher `priority`. Frictionless: `condim="1"`.
8. **margin / gap**: leave `margin=0` unless paired with `gap`, else objects hover.
9. **Slow slip**: soft contacts allow slow creep even inside the cone. Elliptic +
   `impratio` first, then `noslip_iterations="1"`-`"3"` (roughly 2x step cost, makes inverse
   dynamics ill-defined). Backend support differs: see the GPU section.
10. **Tunneling** of thin/fast geoms: enable `<option><flag speculative="enable"/></option>`
    (new in 4.0, not for hfield/SDF/flex) before shrinking the timestep.

Collision geometry: most primitive pairs (sphere, capsule, box, plane, ...) have dedicated
pair functions that are much cheaper and give better normals than general convex CCD, and
far cheaper than SDF collisions. A mesh is always collided as its **convex
hull**, so a mug or bowl mesh behaves like a solid blob: decompose it (CoACD, or the
Menagerie approach of several convex parts) and cap `maxhullvert` (Menagerie SO-101 uses
128; MJX-JAX wants <= 64). `multiccd` and `nativeccd` are enabled by default in 4.0.1, which
gives multiple contacts on flat mesh faces.

Grasp recipe (executed: a parallel pinch lifting a 0.1 kg cube 0.17 m): `implicitfast`,
elliptic cone, `impratio=10`, `solref="0.01 1"`, pad geoms with `condim="4"`, `priority="1"`
and explicit friction, a force-limited position servo commanded slightly past contact so it
squeezes, joint armature and damping on the finger slides, and `exclude` between the palm
and finger bodies. Menagerie SO-101's gripper class (`condim="6"`,
`friction="1 5e-3 5e-4"`, `solref="0.01 1"`, `priority="1"`) is a good template.

### Debugging table

| Symptom | Likely cause | Fix |
|---|---|---|
| Sim explodes / `mjWARN_BADQACC` / state silently resets | Timestep too large for stiffness; explicit `kv` or damping under Euler; light link with no armature; initial penetration | `implicitfast`, smaller timestep, add `armature`, remove initial overlaps, bound `forcerange` |
| Explodes only when the actuator saturates | Saturated actuators contribute no implicit damping (4.0 change) | Joint `armature` + passive `damping`, or smaller timestep |
| Objects sink into the floor or each other | `solref` too soft, mass ratio > 1000:1, `solimp` dmin low | `solref` 0.004-0.01, `solimp` 0.95, realistic masses |
| Objects hover above surfaces | `margin` without matching `gap`; mesh hull larger than visual | Remove margin; check collision group rendering |
| Grasped object slips or rotates out | Pyramidal cone, condim 3, low normal force (servo stops at contact), friction mixing picking the low value | Elliptic + `impratio=10`, `condim=4/6`, command past contact, `priority`, `noslip_iterations` |
| High-frequency jitter at rest | High gains vs armature, stick-slip, explicit damping | Add armature, `implicitfast`, lower `kp` or add `dampratio=1`, smaller timestep |
| Thin parts pass through each other | Tunneling within one step | `speculative` flag, thicker collision geoms |
| Mug/bowl behaves like a solid | Convex hull of a concave mesh | Convex decomposition |
| Slow simulation | Mesh-mesh collisions, too many candidate pairs, high `iterations` | Primitives, `contype`/`exclude`, `maxhullvert`, profile with `testspeed`; enable `sleep` for resting clutter |
| Joint drifts past its limit | Limits are soft too | Joint `solreflimit`/`solimplimit`, keep `ctrlrange` inside `range` |
| Actuator does nothing | `ctrlrange` in degrees vs radians, `kp` left at default 1, `gear` sign | Inspect `model.actuator_gainprm`, `actuator_ctrlrange` |

Detecting divergence (executed; a 1 g link driven by `kp=50 kv=5` at `dt=10 ms`):

```python
def make(integrator, armature):
    return f"""<mujoco><option timestep="0.01" integrator="{integrator}"/>
      <worldbody><body><joint name="j" axis="0 1 0" armature="{armature}"/>
        <geom type="capsule" fromto="0 0 0 0.05 0 0" size="0.005" mass="0.001"/></body></worldbody>
      <actuator><position joint="j" kp="50" kv="5"/></actuator></mujoco>"""

for integ, arm in [("Euler", 0), ("implicitfast", 0), ("Euler", 0.1)]:
    m = mujoco.MjModel.from_xml_string(make(integ, arm)); d = mujoco.MjData(m)
    d.ctrl[0] = 1.0
    mujoco.mj_step(m, d, nstep=500)
    bad = d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number
    print(integ, arm, bad, round(d.qpos[0], 3))
# Euler 0 1 0.027   <- diverged, auto-reset, ended far from the 1.0 target
# implicitfast 0 0 1.0
# Euler 0.1 0 1.0
```

The `autoreset` flag (on by default) resets the state when it detects numerical problems,
so a diverged run can look merely wrong rather than crash. In RL pipelines treat any
`BADQACC`/`BADQPOS` warning count increase or non-finite `qpos` as a terminal, invalid
transition rather than letting the reset hand the policy a teleported state.
`model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY` fills `data.energy`
(potential, kinetic): growing energy in an unforced, undamped model means integration error.

## GPU and Batched Simulation

Device data structures need static sizes, which is the root of most surprises.

MJX (JAX). Executed on CPU; the same code runs on GPU/TPU:

```python
import jax, jax.numpy as jnp, mujoco
from mujoco import mjx

mx = mjx.put_model(mujoco.MjModel.from_xml_string(XML))   # impl="jax" by default
dx0 = mjx.make_data(mx)

def rollout(d, n_steps=250):
    def body(d, _):
        d = mjx.step(mx, d)
        return d, d.qpos[2]
    return jax.lax.scan(body, d, None, length=n_steps)

vx = jax.random.uniform(jax.random.PRNGKey(0), (256,), minval=-1, maxval=1)
batch = jax.vmap(lambda v: dx0.replace(qvel=dx0.qvel.at[0].set(v)))(vx)
final, z_traj = jax.jit(jax.vmap(rollout))(batch)          # (256, 7), (256, 250)

# Domain randomisation: vmap only the randomised field, then batch it via in_axes.
gf = jax.vmap(lambda mu: mx.geom_friction.at[:, 0].set(mu))(jnp.linspace(0.2, 1.5, 256))
mx_b = mx.tree_replace({"geom_friction": gf})
in_axes = jax.tree_util.tree_map(lambda x: None, mx).tree_replace({"geom_friction": 0})
stepped = jax.jit(jax.vmap(mjx.step, in_axes=(in_axes, 0)))(mx_b, batch)
```

Do not return a whole `mjx.Model` from a `vmap`ped function: every field gets a batch axis
and stepping fails with a length-mismatch `IndexError` (hit while testing).

MuJoCo Warp (executed on the Warp CPU device):

```python
import mujoco, mujoco_warp as mjw, numpy as np
mjm = mujoco.MjModel.from_xml_string(XML); mjd = mujoco.MjData(mjm)
nworld = 16
m = mjw.put_model(mjm, batch_sizes={"geom_friction": nworld})   # per-world model field
fr = np.tile(mjm.geom_friction[None], (nworld, 1, 1)); fr[:, :, 0] = np.linspace(0.3, 1.2, nworld)[:, None]
m.geom_friction.assign(fr)
d = mjw.put_data(mjm, mjd, nworld=nworld, nconmax=8, njmax=64)   # or mjw.make_data(...)
for _ in range(300):
    mjw.step(m, d)                     # in place; capture in a CUDA graph on GPU
assert not np.any(d.overflow.numpy())  # buffers overflowed -> undefined behaviour
mjw.get_data_into(mjd, mjm, d, world_id=3)
```

What differs from CPU MuJoCo (details and full tables in `references/gpu-batched-sim.md`):

- float32 on device vs float64 on CPU: stiff models that are fine on CPU can drift or NaN.
- No `PGS` on any GPU path. `noslip`: MJX-JAX has no implementation and does **not** reject
  `noslip_iterations > 0`, so it is silently ignored (verified in 4.0.1). MuJoCo Warp 4.0.1
  implements it except with `discrete` (the MJX docs table still lists it as unsupported).
- MJX-JAX additionally lacks constraint islands, plugins, flex, SDF, cylinder-box/mesh and
  ellipsoid-box/mesh collisions, and `implicitfast` with fluid drag. `integrator="discrete"`
  passes `put_model` but `mjx.step` raises `NotImplementedError` (verified).
- MJX-JAX allocates contact slots for every geom pair that *could* collide; prune with
  `contype`/`conaffinity`, `exclude`, explicit `<pair>`s, and the custom numerics
  `max_contact_points` / `max_geom_pairs`. Keep hulls under ~32 vertices for mesh-mesh.
  Lower `iterations`/`ls_iterations` (Newton often converges in 1-4).
- MuJoCo Warp / MJX-Warp sizes contacts with `nconmax`/`naconmax` and constraints with
  `njmax` (per world, hard cap); `<size nconmax njmax>` in XML is ignored. Excess contacts
  are dropped and flagged in `d.overflow`. Read contacts through contact sensors.
- MJX-Warp resets subsets of worlds with `data.where(done, reset_data)`, not `tree_map`.
- A GPU step is slower than a CPU step; GPUs win on throughput. One env on MJX-JAX can be
  ~10x slower than CPU MuJoCo (official figure).

Playground 1.x environments are layered PyTorch modules (`reaf.simulation.torch`): a
device layer binds joints/actuators/cameras/sensors to the model, a task layer supplies
features, rewards and terminations, and an `EnvironmentReset` hook gets privileged model and
data access for domain randomisation. Training: `python -m mujoco_playground.rl.rsl_rl.train
--env=SO101PickCube`. For the older JAX `MjxEnv` / Brax workflow use `rss2025/`. Algorithm
and reward choices belong to `robot-reinforcement-learning`.

## Sim-to-Real for MuJoCo Models

Identify before you randomise: randomising around a wrong nominal model only teaches the
policy to be robust to the wrong robot.

What to identify on an arm (in typical order of impact, heuristic): joint `armature`
(reflected rotor inertia dominates small links and geared servos), actuator `kp`/`kv` and
`forcerange` (servo firmware gains do not map 1:1 to MuJoCo gains; Menagerie SO-101 notes
its STS3215 `kp="998.22" kv="2.731" forcerange="-2.94 2.94"` were derived for a servo P gain
of 16), joint `frictionloss` and `damping`, control latency, and link masses for payloads.

`mujoco.sysid` (`pip install "mujoco[sysid]"`) fits `MjSpec` parameters to logged
trajectories with box-constrained Gauss-Newton over batched `mujoco.rollout` evaluations,
with helpers for physically consistent inertia (`body_inertia_param`), actuator gains
(`apply_pdgain`), sensor delays, and an HTML report. Playground's
`mujoco_playground/tasks/sysid/so101` fits armature, frictionloss, stiffness and damping for
all six SO-101 joints from teleop logs and is the best worked example. Excite every joint
with reversals and varying speeds; a slow sinusoid cannot separate armature from damping.

Validate the pipeline on synthetic data first. In a test here (data generated by the model
itself, known armature 0.03 and damping 0.2), `mujoco.sysid` recovered both exactly, but a
one-sample misalignment between the sensor log and the control log biased them by -5% and
-8% with zero noise. The reference file shows the alignment that recovered exactly.

Latency is native: actuator `delay` + `nsample` (history buffer; `delay` > 0 requires
`nsample` > 0, minimum effective delay one timestep) and the same on sensors, plus sensor
`interval` for lower-rate sensors. Prefer this over ad-hoc Python queues so the delay is
part of the saved model and of `mjSTATE_FULLPHYSICS`.

Domain randomisation on a compiled `MjModel` (executed). Real-valued fields are safe to
write; some need `mj_setConst` to recompute derived quantities:

```python
b = model.body("link1").id
s = rng.uniform(0.8, 1.2)
model.body_mass[b] = nominal_mass[b] * s
model.body_inertia[b] = nominal_inertia[b] * s           # scale with mass
model.dof_armature[:] = nominal_armature * rng.uniform(0.8, 1.2)
model.geom_friction[:, 0] = rng.uniform(0.6, 1.2)
kp = 30 * rng.uniform(0.9, 1.1)
model.actuator_gainprm[0, 0] = kp
model.actuator_biasprm[0, 1] = -kp                       # position servo: keep bias = -kp
mujoco.mj_setConst(model, data)                          # mass, inertia, armature, geom pose/size
mujoco.mj_resetData(model, data)                         # mj_setConst uses data as scratch
```

- `mj_setConst` does **not** recompute a `dampratio`-derived `kv`; set `biasprm[2]` yourself
  or recompile from the spec.
- Integer and structural fields (geom type, `nq`, `contype` on geoms that did not collide at
  compile time, mesh vertices of colliding meshes) are unsafe at runtime: change the `MjSpec`
  and recompile.
- Per-sim fields on CPU: `mujoco.batch.Batch(model, nsim, nthread)`, then
  `b.body("link1").mass = values[:, None]` (shape `(nsim, 1)`) and `b.set_const()`.
- On GPU: batched `mjx.Model` fields through `in_axes`, or `mjw.put_model(...,
  batch_sizes={...})` (indexed `field[world % batch]`).

Load `references/sim-to-real-model-fields.md` for the full runtime-modifiable field table,
a sysid skeleton, and a DR field checklist. Randomisation ranges and curricula are
algorithm design: see `robot-reinforcement-learning`.

## Anti-Patterns

- **Leaving the physics defaults**: Euler + pyramidal cone + `solref 0.02` + zero armature is
  the most common source of jitter and slip. Set option and top-level defaults once.
- **Degrees by accident**: forgetting `<compiler angle="radian"/>` with radian ranges.
- **Visual meshes that collide and add mass**: `contype="0" conaffinity="0"` stops collision
  but not inertia inference; also use explicit `<inertial>`, `inertiagrouprange`, or
  `mass="0"`/`density="0"` on one of the two geom sets.
- **Raw indices**: `data.qpos[7]` breaks when a joint is added. Use named access,
  `jnt_qposadr`, or `bind`.
- **Reading `sensordata` right after `mj_step` and assuming it matches `qpos`**: it is one step
  behind. Use `mj_forward` or `mj_step1`/`mj_step2`.
- **Using `data.contact` as an observation**: variable length and order; use contact sensors.
- **Trusting `noise=` on sensors**: it is metadata only.
- **Creating a `Renderer` per frame**: leaks GL contexts and is slow; create once, close it.
- **Setting `MUJOCO_GL` after `import mujoco`** (or after importing dm_control/gymnasium).
- **Randomising `kp` without updating `biasprm[1]`**: the servo then has a spring to the
  wrong setpoint.
- **Ignoring `BADQACC` / auto-reset** in RL rollouts.
- **Hand-tuning gains by eye** when `mujoco.sysid` and a 2-minute excitation log exist.
- **Porting a CPU model to MJX unchanged**: dense mesh scenes crawl, unsupported features
  raise at `put_model` or `step`, and `noslip` is silently dropped. Make an `_mjx.xml`
  variant with primitives and pruned pairs, then compare CPU and GPU rollouts.
- **Using `mujoco_py`, `mujoco.load_model_from_path`, or Gymnasium `-v2/-v3` envs**: legacy
  APIs; use `mujoco.MjModel.from_xml_path` and `-v5`.
- **Old tutorials' `composite`/`cable` plugin**: removed in 4.0; use 1D `flexcomp`.

## Checklist

- [ ] `<compiler angle="radian">` set (or all angles verified as degrees); `meshdir` correct
- [ ] `<option>`: `integrator="implicitfast"`, `timestep` chosen deliberately, elliptic cone
      + `impratio` for manipulation
- [ ] Visual and collision geoms in separate default classes; collision uses primitives or
      convex parts; `maxhullvert` capped
- [ ] Masses and inertias plausible (render inertia boxes); no ratios beyond ~1000:1
- [ ] Every geared joint has `armature` and `damping`; actuators have `forcerange`
- [ ] `ctrlrange` inside joint `range` (except grippers); servo gains documented with source
- [ ] No interpenetration at `qpos0` and every keyframe (`data.ncon` and contact distances
      after `mj_forward`)
- [ ] Control loop: substeps computed from `timestep`, `mj_forward` after resets, named access
- [ ] Observations from sensors, copied; contact sensors for touch/force
- [ ] `BADQACC` / non-finite state checked and treated as failure
- [ ] Determinism test: two rollouts from the same keyframe and controls match exactly
- [ ] Headless runs set `MUJOCO_GL` before import; `offwidth/offheight` large enough
- [ ] GPU port: `put_model` succeeds, `nconmax/njmax` sized from measured peaks, overflow
      checked at episode boundaries, CPU vs GPU rollouts compared
- [ ] Sim-to-real: armature, gains, friction and latency identified from logs before DR
- [ ] MuJoCo version pinned (Menagerie READMEs state minimum versions; 4.0 changed behaviour)

## Related Skills

- `robot-reinforcement-learning`: RL algorithms, reward design, DR ranges, sim-to-real theory
- `isaac-sim`: the alternative GPU simulator stack (Isaac Sim / Isaac Lab)
- `lerobot`: SO-100/SO-101 hardware, datasets, and policies that pair with Menagerie models
- `imitation-learning`: policies trained from demonstrations collected in MuJoCo
- `robot-data-quality`: validating logged trajectories used for sysid and training
- `robotics-testing`: simulation tests, determinism and golden-trajectory regression tests
- `ros2`: bridging a MuJoCo sim to ROS 2 topics and controllers

## References

Load on demand:

- `references/mjcf-reference.md`: when writing or reviewing MJCF in detail; attribute tables
  with verified defaults, actuator shortcut mappings, sensor list, URDF conversion recipe.
- `references/gpu-batched-sim.md`: when porting to MJX or MuJoCo Warp, sizing contact
  buffers, writing batched resets/DR, or reading Playground/rss2025 env code.
- `references/sim-to-real-model-fields.md`: when randomising model fields at runtime, running
  `mujoco.sysid`, or adding latency and noise models.

Primary sources: mujoco.readthedocs.io (XML reference, Modeling > Model tuning, Computation,
Python, MJX, MuJoCo Warp), github.com/google-deepmind/mujoco (`doc/changelog.rst`,
`doc/skills/`), mujoco_menagerie, mujoco_playground, mujoco_warp.
