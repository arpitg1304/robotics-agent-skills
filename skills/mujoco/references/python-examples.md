# MuJoCo Python Examples (MuJoCo 4.0.1)

Executed examples and API semantics moved from SKILL.md: simulation-loop semantics,
reset and state save/restore, the interactive viewer, offscreen rendering, and URDF
import with `MjSpec`. The minimal loop itself stays in SKILL.md.

## Simulation loop: key semantics

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

## Reset and state save/restore

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

## Interactive viewing

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
  For thousands of cameras on GPU use the MuJoCo Warp batch renderer (see `gpu-batched-sim.md`).

## URDF import and MjSpec editing

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
