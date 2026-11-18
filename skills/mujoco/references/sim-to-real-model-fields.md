# Sim-to-Real: Identifying and Randomising MuJoCo Model Fields (MuJoCo 4.0.1)

Scope: the MuJoCo mechanics of system identification, latency modelling and domain
randomisation. Which parameters to randomise, by how much, and how to train a robust policy
belongs to `robot-reinforcement-learning`. Sources: `doc/programming/simulation.rst`
(*mjModel changes*), `doc/modeling.rst` (*Model tuning*, *Delays*), `python/mujoco/sysid/`
README, Playground `tasks/sysid/so101`. Code below was executed on MuJoCo 4.0.1.

## Where each physical quantity lives

| Physical quantity | MJCF | `MjModel` field | `MjSpec` handle |
|---|---|---|---|
| Rotor / gearbox reflected inertia | `joint/armature`, `actuator/armature` | `dof_armature` | `spec.joint(n).armature` |
| Viscous friction | `joint/damping` (up to 3 coeffs) | `dof_damping` (+ `dof_dampingpoly`) | `spec.joint(n).damping = [a, b, c]` |
| Coulomb friction / stiction | `joint/frictionloss` | `dof_frictionloss` | `spec.joint(n).frictionloss` |
| Servo P / D gains | `position kp kv` | `actuator_gainprm[:,0]`, `actuator_biasprm[:,1:3]` = `-kp, -kv` | `act.set_to_position(kp=..., kv=...)`; `sysid.apply_pdgain` |
| Torque limit | `actuator/forcerange`, `joint/actuatorfrcrange` | `actuator_forcerange`, `jnt_actfrcrange` | |
| Gear ratio / sign | `actuator/gear` | `actuator_gear` | |
| Link mass, COM, inertia | `inertial` | `body_mass`, `body_ipos`, `body_inertia`, `body_iquat` | `spec.body(n).mass` ...; `sysid.body_inertia_param` |
| Contact friction | `geom/friction`, `pair/friction` | `geom_friction`, `pair_friction` | |
| Contact softness | `solref`, `solimp` | `geom_solref`, `geom_solimp` | |
| Control latency | `actuator/delay` + `nsample` | `actuator_delay`, `actuator_history` | |
| Sensor latency / rate | `sensor/delay`, `nsample`, `interval` | `sensor_delay`, `sensor_history`, `sensor_interval` (period, phase) | |
| Gravity compensation | `body/gravcomp` | `body_gravcomp` | |
| Joint zero offset (calibration) | `qpos0` via keyframe / `ref` | `qpos0` | |

Sensor `noise` is stored in `sensor_noise` but never applied. Add measurement noise and
bias in your observation code.

Check field names in your installed version before writing DR code:
`[f for f in dir(model) if f.startswith("actuator_")]`.

## Runtime modification rules

General rule (official docs): real-valued fields are safe to change, structural integer
fields are not. Exceptions that matter for DR:

| Field(s) | Safe? |
|---|---|
| `geom_friction`, `geom_solref`, `geom_solimp`, `dof_damping`, `dof_frictionloss`, `actuator_gainprm`, `actuator_biasprm`, `actuator_forcerange`, `opt.*` | Safe directly |
| `body_mass`, `body_inertia`, `body_ipos`, `body_iquat` | Safe with `mj_setConst` (scale mass and inertia together; keep inertia triangle inequality) |
| `dof_armature`, `qpos0`, `qpos_spring` | Safe with `mj_setConst` |
| `body_pos`, `body_quat` | Safe with `mj_setConst`, **unsafe for static bodies** (BVH invalidated) |
| `geom_pos`, `geom_quat`, `geom_size`, `geom_margin`, `geom_gap` | Safe with `mj_setConst`; inferred masses/inertias do not follow; mesh bounds follow the mesh, not `size` |
| `geom_contype`, `geom_conaffinity` | Safe with `mj_setConst`, but a geom that did not collide at compile time in a body whose other geoms did cannot be made to collide |
| `geom_condim`, `geom_priority`, `*_limited`, `*_group`, `*_matid` | Safe (integer exceptions) |
| `site_pos`, `site_quat`, `eq_data`, `hfield_size` | Safe with `mj_setConst` |
| `cam_pos`, `cam_quat`, `light_pos`, `light_dir` | Mostly safe; tracking cameras/lights need `mj_setConst` |
| `body_gravcomp` | Safe; update `flg_gravcomp` (or `mj_setConst`) when switching between all-zero and non-zero |
| `mesh_vert`, `mesh_face` of colliding meshes, `bvh_aabb` | Unsafe: edit the `MjSpec` and recompile |
| `hfield_data` | Safe (values in [0, 1]); call `mjr_uploadHField` for rendering |

`mj_setConst` does not recompute values the compiler derived once: `dampratio`-derived `kv`,
`springdamper`-derived joint stiffness/damping, tendon spring lengths, model statistics.
It uses the `MjData` as scratch and wakes sleeping trees, so reset the data afterwards.

Executed CPU DR loop:

```python
rng = np.random.default_rng(0)
nominal = dict(mass=model.body_mass.copy(), inertia=model.body_inertia.copy(),
               armature=model.dof_armature.copy(), kp=model.actuator_gainprm[:, 0].copy())

def randomise(model, data):
    s = rng.uniform(0.8, 1.2, size=model.nbody)
    model.body_mass[:] = nominal["mass"] * s
    model.body_inertia[:] = nominal["inertia"] * s[:, None]
    model.dof_armature[:] = nominal["armature"] * rng.uniform(0.8, 1.2, size=model.nv)
    model.geom_friction[:, 0] = rng.uniform(0.6, 1.2)
    kp = nominal["kp"] * rng.uniform(0.9, 1.1, size=model.nu)
    model.actuator_gainprm[:, 0] = kp
    model.actuator_biasprm[:, 1] = -kp           # position actuators only
    mujoco.mj_setConst(model, data)
    mujoco.mj_resetData(model, data)
```

If several actuator types share the model, randomise `biasprm` only for those whose
`actuator_biastype` is affine with `biasprm[1] == -gainprm[0]` (position servos).

When in doubt, the safe, slower path is: mutate the `MjSpec`, `spec.compile()`, new
`MjData`. On MuJoCo Warp this costs one host compile per variant; per-world field batching
(`batch_sizes`) avoids it for real-valued fields.

## Latency and rates

```xml
<actuator>
  <!-- 20 ms command latency at a 2 ms timestep: delay = nsample * timestep -->
  <position name="j1" joint="j1" kp="50" dampratio="1" delay="0.02" nsample="10"/>
</actuator>
<sensor>
  <jointpos name="j1_enc" joint="j1" delay="0.004" nsample="2"/>
  <gyro name="imu_gyro" site="imu" interval="0.005" nsample="2"/>  <!-- 200 Hz sensor -->
</sensor>
```

- `delay > 0` requires `nsample > 0`; the minimum effective delay is one timestep.
- History buffers live in `mjData.history` and are part of `mjSTATE_PHYSICS`, so delayed
  systems checkpoint and replay correctly. `mj_resetData` zero-fills them; use
  `mj_initCtrlHistory` / `mj_initSensorHistory` for a non-zero start.
- `nsample > 0` with `delay = 0` keeps a readable history without changing the simulation
  (`mj_readCtrl`, `mj_readSensor`).
- Randomise latency by recompiling from the spec with a different `delay` (it changes buffer
  sizes when `nsample` changes; keep `nsample` at the maximum and vary `delay` below it).

## System identification with `mujoco.sysid`

Install `pip install "mujoco[sysid]"` (adds matplotlib, plotly, jinja2, ...). The toolbox
optimises `MjSpec` parameters by box-constrained nonlinear least squares, with every
finite-difference perturbation run in one batched `mujoco.rollout` call.

Executed example (synthetic "hardware" data from a model with known parameters; the fit
recovers armature 0.0300 and damping 0.2000 exactly from nominal 0.01 / 0.05 in 6
iterations, about 0.1 s):

```python
import mujoco, numpy as np
from mujoco import sysid

spec = mujoco.MjSpec.from_string(XML)          # nominal model
model = spec.compile()

def set_armature(s, p): s.joint("j").armature = p.value[0]
def set_damping(s, p):  s.joint("j").damping = [p.value[0], 0, 0]

params = sysid.ParameterDict()
params.add(sysid.Parameter("armature", nominal=0.01, min_value=1e-4, max_value=0.1,
                           modifier=set_armature))
params.add(sysid.Parameter("damping", nominal=0.05, min_value=0.0, max_value=1.0,
                           modifier=set_damping))

control = sysid.TimeSeries.from_control_names(times, ctrl_log, model)     # (T, nu)
measured = sysid.TimeSeries.from_names(times, sensor_log, model)          # (T, nsensordata)
x0 = sysid.create_initial_state(model, qpos_0, qvel_0)
ms = sysid.ModelSequences("arm", spec, "chirp", x0, control, measured)

residual_fn = sysid.build_residual_fn(models_sequences=[ms])
opt_params, result = sysid.optimize(initial_params=params, residual_fn=residual_fn)
print(opt_params["armature"].value, opt_params["damping"].value)
# sysid.save_results(out_dir, [ms], params, opt_params, result, residual_fn) + default_report
```

**Time alignment matters.** In that synthetic test the fit was exact only when sample `k`
of the sensor log held the value `data.sensordata` has after `k` calls to `mj_step` (that is,
the sensor evaluated at time `t_{k-1}`; zeros at `k=0`). Logging the sensor at `t_k` instead
(via `mj_forward` before each step) biased armature by -5% and damping by -8% with zero
noise. Real logs have their own transport delays on top. Before trusting a fit on hardware
data: (1) generate synthetic data from your own nominal model with the exact logging code
path, perturb the parameters, and check the fit recovers them; (2) estimate command and
measurement delays explicitly (`sysid.apply_delay` and the Playground SO-101 script, which
estimates serial transport delay first).

Other built-ins: `sysid.body_inertia_param(..., InertiaType.Mass | MassIpos | Pseudo)`
(Pseudo = physically consistent 10-D inertia), `sysid.apply_pdgain(spec, "act", value)`,
`apply_pgain`, `apply_dgain`, `apply_gain`, `apply_bias`. The measurement-correction API
(delays, gains, biases) is "functionally complete but not yet final" per its README.

## Excitation and data

- Excite every joint with direction reversals and a range of speeds and accelerations
  (chirps, multi-sines, teleop sweeps). A single slow sinusoid cannot separate armature
  (acceleration-proportional) from damping (velocity-proportional) from frictionloss
  (sign of velocity).
- Log commanded `ctrl` exactly as sent to the hardware, at the timestamps it was applied.
- Fit gains against the servo's real behaviour: hobby servos (Feetech STS3215 on SO-101) run
  firmware PID in their own units; MuJoCo `kp/kv` are effective torque gains. Menagerie
  SO-101's `kp="998.22" kv="2.731" forcerange="-2.94 2.94"` came from such a derivation and
  Playground's SO-101 sysid fits armature, frictionloss, stiffness and damping per joint.
- Hold out a trajectory to validate; compare in the viewer side by side with replayed real
  data. Watch for parameters stuck at their bounds (often a sign of a missing effect such
  as backlash, cable forces, or a wrong gear ratio).
- Backlash can be modelled with two coaxial hinges (one actuated with armature, one with a
  tiny range) as in the MuJoCo modelling guide; Menagerie SO-101 defines but does not use
  such a `backlash` class.

## DR field checklist

- [ ] Nominal model identified from logs (not from CAD alone)
- [ ] Each randomised field is in the safe list above, or changed through `MjSpec` + recompile
- [ ] `mj_setConst` called after mass/inertia/armature/geometry changes, then `mj_resetData`
- [ ] Position-servo `biasprm` kept consistent with randomised `kp`/`kv`
- [ ] Latency modelled with `delay`/`nsample`, not a Python queue
- [ ] Observation noise added in code (sensor `noise=` is ignored)
- [ ] On GPU: only randomised fields batched (`in_axes` / `batch_sizes`), rest shared
- [ ] Ranges and curriculum justified in the training design (`robot-reinforcement-learning`)

## Overview and executed DR example

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
-8% with zero noise. The *Time alignment matters* note in the `mujoco.sysid` section shows the alignment
that recovered exactly.

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
