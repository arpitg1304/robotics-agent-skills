# Contacts, Stability and Debugging (MuJoCo 4.0.1)

Full contact-tuning guide, collision geometry and grasp notes, the complete symptom
table, and the executed divergence-detection example, moved from SKILL.md.

## Tuning order

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
   dynamics ill-defined). Backend support differs: see `gpu-batched-sim.md`.
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

## Debugging table

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

## Detecting divergence

Executed; a 1 g link driven by `kp=50 kv=5` at `dt=10 ms`):

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
