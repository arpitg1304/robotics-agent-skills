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

Full component table and Menagerie model list: `references/ecosystem.md`.

- **Core**: `pip install mujoco` (4.0.1), Python >= 3.11. Since 4.0 `mujoco` is a
  metapackage: `mujoco-core` (numpy only; headless servers, RL workers) + `mujoco-studio` +
  `glfw`/`pyopengl`.
- **CPU batch**: `from mujoco import batch, rollout`; multi-threaded, float64, all
  solvers/plugins.
- **MJX** (`pip install mujoco-mjx`, `from mujoco import mjx`): `impl="jax"` (default; any XLA
  device; differentiable) or `impl="warp"` (NVIDIA; not differentiable).
- **MuJoCo Warp** (`pip install mujoco-warp`, `import mujoco_warp as mjw`): NVIDIA GPU,
  highest-throughput batches, PyTorch training; not differentiable.
- **MuJoCo Playground** (`pip install playground`, 1.0.1): rewritten as **PyTorch** tasks on
  `reaf-sim` + MuJoCo Warp with RSL-RL training and real-robot deploy. The original JAX/MJX
  envs live in the repo's `rss2025/` directory.
- **Menagerie**: curated MJCF robots (`pip install mujoco-menagerie`); never hand-write a
  common arm. SO-101: `robotstudio_so101/so101.xml`. Some models ship `*_mjx.xml` variants.
- **dm_control** 1.0.52 requires `mujoco>=4.0.1`; for new procedural models use
  `mujoco.MjSpec`, not PyMJCF. **Gymnasium** 1.4.0: `*-v5` envs only; `-v2`/`-v3` raise a
  mujoco-py error.
- **Viewers**: `mujoco.viewer` (managed/passive) and MuJoCo Studio
  (`python -m mujoco.studio.viewer --model=x.xml`, `--gfx=web`), the successor of `simulate`.
- **4.0 changes noted in this skill**: metapackage split; saturated actuators contribute no
  implicit damping; the `composite`/`cable` plugin was removed (use 1D `flexcomp`);
  `speculative` contacts are new.

Choosing a backend (heuristic): one env or real-time loop -> CPU `mujoco`; tens to a few
thousand envs on a many-core box, or you need `noslip`/PGS/plugins/float64 ->
`mujoco.batch`; thousands of envs on an NVIDIA GPU -> MuJoCo Warp (or MJX with
`impl="warp"` inside JAX); gradients through dynamics, TPU, AMD or Apple GPU -> MJX-JAX.

## MJCF Essentials

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

Rules that prevent most modelling bugs (full text: `references/mjcf-reference.md`, last
section; attribute tables with verified defaults are in the same file):

- **Angles**: `compiler/angle` defaults to `"degree"` for MJCF (always radian for URDF). Set it.
- **Mass**: every geom in `inertiagrouprange` (default `"0 5"`) adds inferred mass, including
  visual geoms with `contype=conaffinity=0`; a duplicated visual geom doubled the mass
  (verified). Use explicit `<inertial>`, `inertiagrouprange`, or `mass="0"`/`density="0"`.
- **Joints**: `armature` and `damping` default to 0 and should almost never be 0 on a geared
  joint. In `MjSpec`, `damping`/`stiffness` take a 3-vector (a scalar raises `TypeError`).
- **Actuators**: `position` has `kp` default 1, `kv` 0; it compiles to `gainprm[0]=kp`,
  `biasprm[1:3]=[-kp, -kv]`. Always set `forcerange`; keep `ctrlrange` inside the joint `range`
  except on grippers.
- **Collision filtering**: collide iff `(contype1 & conaffinity2) || (contype2 & conaffinity1)`;
  same-body and parent-child pairs are excluded; use `<exclude>` and `<pair>`.
- **Keyframes**: `qpos` is positional; re-record after adding or removing joints. Mocap
  fields are `mpos`/`mquat`.
- **Sensors**: observe through `data.sensordata` and `<contact>` sensors, not `data.contact`.
  Sensor `noise` **does not affect simulation**; add noise yourself.

URDF import (`MjSpec.from_file("robot.urdf")`: `nu == 0`, `package://` paths, discarded
visuals, fused links) and `MjSpec` editing: `references/python-examples.md`, with the
conversion recipe in `references/mjcf-reference.md`.

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

Key semantics (full list, state save/restore and the passive viewer:
`references/python-examples.md`):

- **After `mj_step`, `sensordata`, `xpos`, contacts etc. describe the state at the start of
  that step**, one step behind `qpos`. Call `mj_forward` before reading if you need them
  consistent with `qpos`, or use `mj_step1` / set `ctrl` / `mj_step2`.
- `mj_forward` computes everything without integrating. Call it after editing `qpos`,
  `qvel`, `mocap_pos` or after a reset, before reading positions or sensors.
- Never write `xpos`, `xquat`, `sensordata`: they are outputs.
- Quaternions are scalar-first `[w, x, y, z]`; SciPy and ROS are scalar-last.
- Free-joint `qpos` is 7 numbers but `qvel` is 6, so `nq != nv`: use named access or
  `jnt_qposadr` / `jnt_dofadr`.

Reset with `mujoco.mj_resetData` or `mj_resetDataKeyframe`; checkpoint with `mj_getState` /
`mj_setState` and `mjSTATE_INTEGRATION` (bit-identical continuation).

Determinism: the same binary, model, initial state and control sequence reproduce
bit-identical trajectories on CPU (asserted in the executed example; `mujoco.batch` is
documented as bit-identical to serial stepping). Things that break it: resetting by writing
`qpos`/`qvel`/`time` instead of calling `mj_resetData` (solver warm-start carries over;
measured 3e-6 divergence after 300 contact steps, versus bit-identical with `mj_resetData`),
different library versions or builds, GPU float32 backends, and unseeded Python randomness.

## Rendering

Create `mujoco.Renderer(model, height, width)` once (as a context manager) and reuse it:
`update_scene(data, camera="name")` then `render()` gives `(H, W, 3)` uint8;
`enable_depth_rendering()` gives float32 metres and `enable_segmentation_rendering()` gives
`[obj id, mjtObj type]`. Set `MUJOCO_GL` (`egl`, `osmesa`, ...) **before** `import mujoco`.
Width/height cannot exceed `<visual><global offwidth offheight/>` (default 640x480). Render at
the policy rate, not every substep. Executed example, all `MUJOCO_GL` values, cameras, and the
Filament and Warp renderers: `references/python-examples.md`.

## Contacts and Stability

MuJoCo contacts are soft constraints. Tune in this order: fix modelling bugs (initial
interpenetration, concave meshes collided as hulls, bad inertias, units) -> `implicitfast`
-> timestep -> `cone="elliptic"` + `impratio` -> `solref` -> `solimp` -> `condim`/friction ->
`noslip`. The full guide with defaults and starting values, collision geometry, the grasp
recipe and the executed divergence example: `references/contacts-debugging.md`.

Most common symptoms (full table in the reference):

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
| Actuator does nothing | `ctrlrange` in degrees vs radians, `kp` left at default 1, `gear` sign | Inspect `model.actuator_gainprm`, `actuator_ctrlrange` |

The `autoreset` flag (on by default) resets the state when it detects numerical problems,
so a diverged run can look merely wrong rather than crash. In RL pipelines treat any
`BADQACC`/`BADQPOS` warning count increase or non-finite `qpos` as a terminal, invalid
transition rather than letting the reset hand the policy a teleported state.

## GPU and Batched Simulation

Device data structures need static sizes, which is the root of most surprises. Executed MJX
and MuJoCo Warp examples (rollouts, domain randomisation via `in_axes` / `batch_sizes`) and
the full list of differences from CPU: `references/gpu-batched-sim.md`.

- float32 on device vs float64 on CPU: stiff models that are fine on CPU can drift or NaN.
- No `PGS` on any GPU path. MJX-JAX silently ignores `noslip_iterations > 0` (verified in
  4.0.1) and rejects `integrator="discrete"` at `mjx.step`.
- MJX-JAX allocates contact slots for every geom pair that *could* collide: prune with
  `contype`/`conaffinity`, `exclude` and `<pair>`s.
- MuJoCo Warp sizes buffers with `nconmax`/`naconmax`/`njmax` per world (`<size>` in XML is
  ignored); excess contacts are dropped and flagged in `d.overflow`. Check it.
- A GPU step is slower than a CPU step; GPUs win on throughput.

## Sim-to-Real for MuJoCo Models

Identify before you randomise: randomising around a wrong nominal model only teaches the
policy to be robust to the wrong robot. Identify joint `armature`, actuator `kp`/`kv` and
`forcerange`, `frictionloss`/`damping`, latency and payload masses with `mujoco.sysid`
(`pip install "mujoco[sysid]"`); model latency natively with actuator/sensor `delay` +
`nsample`. After changing mass, inertia or armature on a compiled model call `mj_setConst`
then `mj_resetData`; keep a position servo's `biasprm` consistent with a randomised `kp`.
Details, the executed DR example and the runtime-modifiable field table:
`references/sim-to-real-model-fields.md`. Randomisation ranges and curricula are algorithm
design: see `robot-reinforcement-learning`.

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

## Reference Files

Load on demand:

- `references/ecosystem.md`: when picking a package or Menagerie model; full component table
  with install names and status, and the Menagerie arm list.
- `references/mjcf-reference.md`: when writing or reviewing MJCF in detail; attribute tables
  with verified defaults, actuator shortcut mappings, sensor list, URDF conversion recipe,
  and the full modelling-rules text.
- `references/python-examples.md`: when writing Python beyond the minimal loop; full loop
  semantics, reset and `mj_getState`/`mj_setState`, the passive viewer, the executed
  RGB/depth/segmentation example and `MUJOCO_GL` details, URDF import pitfalls and `MjSpec`
  editing.
- `references/contacts-debugging.md`: when a sim explodes, jitters, sinks, tunnels or slips,
  or when tuning a grasp; the ten-step tuning guide, collision geometry, the full symptom
  table, divergence detection and energy checks.
- `references/gpu-batched-sim.md`: when porting to MJX or MuJoCo Warp, sizing contact
  buffers, writing batched resets/DR, or reading Playground/rss2025 env code; includes the
  executed MJX and Warp examples.
- `references/sim-to-real-model-fields.md`: when randomising model fields at runtime, running
  `mujoco.sysid`, or adding latency and noise models; includes the executed DR example.

Primary sources: mujoco.readthedocs.io (XML reference, Modeling > Model tuning, Computation,
Python, MJX, MuJoCo Warp), github.com/google-deepmind/mujoco (`doc/changelog.rst`,
`doc/skills/`), mujoco_menagerie, mujoco_playground, mujoco_warp.
