# Batched and GPU Simulation (MuJoCo 4.0.1)

Sources: `doc/mjx.rst`, `doc/mjwarp/index.rst`, mujoco_warp README, mujoco_playground
README/AGENTS.md (all at main, 2026-11-18), plus tests run against `mujoco-mjx==4.0.1`
and `mujoco-warp==4.0.1` on CPU. Throughput numbers are not reproduced here; measure with
`mjwarp-testspeed` or your own loop.

## Choosing

| Need | Use |
|---|---|
| 1 env, real time, MPC, teleop, full feature set, float64 | `mujoco` (CPU) |
| 10s-1000s of envs on CPU, per-sim parameters, noslip/PGS/plugins | `mujoco.batch.Batch` (stateful) or `mujoco.rollout` (open-loop) |
| 1000s of envs on NVIDIA, PyTorch learner | MuJoCo Warp (`mujoco_warp`) |
| Same, but your stack is JAX | MJX with `impl="warp"` |
| Gradients through physics, TPU, AMD, Apple GPU | MJX with `impl="jax"` (default) |

## CPU batch

```python
from mujoco import batch, rollout
b = batch.Batch(model, nsim=1024, nthread=8)   # persistent=True if sleep/IPC/stateful plugins
qpos, ctrl = b.bind("qpos"), b.bind("ctrl")    # (nsim, nq), (nsim, nu) views
b.body("link1").mass = masses[:, None]         # per-sim model field, shape (nsim, 1)
b.set_const()                                  # recompute derived constants
ctrl[:] = policy(qpos)
b.step()                                       # or b.step(ids=active_ids)
```

Verified: per-sim masses produced different steady-state angles across 8 sims. Bit-identical
to serial `MjData` stepping (documented). `rollout.rollout(model, data, initial_state,
control, nstep=...)` runs fixed control sequences `(nbatch, nstep, nu)` from full-physics
initial states `(nbatch, mj_stateSize(model, mjSTATE_FULLPHYSICS))` without Python in the
loop and returns `(state, sensordata)`; pass a list of `MjData` (one per thread) for
parallelism and a list of models for per-rollout parameters.

## MJX-JAX

- `mx = mjx.put_model(m)`, `dx = mjx.make_data(mx)` or `mjx.put_data(m, d)`; `mjx.step`,
  `mjx.forward`, `mjx.get_data_into(d, m, dx)` to copy back.
- `mjx.Data`/`mjx.Model` are immutable pytrees: `dx.replace(qpos=...)`,
  `mx.tree_replace({"geom_friction": ...})`.
- Named access: `dx.bind(mx, spec.body("torso")).xpos`,
  `dx = dx.bind(mx, spec.actuators).set("ctrl", values)`.
- Batched resets: `jax.tree.map(lambda a, b: jnp.where(done_bcast, a, b), reset, data)` or
  the unified `data.where(done, reset_data)` which also works for MJX-Warp.
- Persistent compile cache: `jax.config.update("jax_compilation_cache_dir", "/tmp/jax_cache")`.
- GPU: `XLA_FLAGS=--xla_gpu_triton_gemm_any=true` (documented ~30% on NVIDIA).

Feature support (from the MJX feature-parity table, MJX-JAX column):

| Category | Supported in MJX-JAX |
|---|---|
| Joints | free, ball, slide, hinge |
| Transmission | joint, jointinparent, site, tendon |
| Actuator dynamics | none, integrator, filter, filterexact, muscle |
| Gain / bias | fixed, affine, muscle / none, affine, muscle |
| Geoms | plane, hfield, sphere, capsule, box (implemented as a mesh), mesh. Ellipsoid and cylinder collide only with other primitives. Unsupported: SDF; cylinder vs sphere/box/mesh/hfield; ellipsoid vs box/mesh/hfield |
| Constraints | equality, joint limits, frictionless/pyramidal/elliptic contact, dof and tendon friction |
| Equality | connect, weld, joint, tendon |
| Integrators | Euler, RK4, implicitfast (not with fluid drag). `discrete` raises `NotImplementedError` at `mjx.step` (verified) |
| condim | 1, 3, 4, 6 (1 not with elliptic) |
| Solvers | CG, Newton. No PGS. `noslip_iterations` accepted but has no implementation (verified: no code path) |
| Jacobian | dense only |
| Sensors | most kinematic/force sensors; contact sensor only geom-geom or unfiltered, `mindist`/`maxforce` |
| Flex, plugins, islands | not supported |

Performance rules (MJX-JAX):

- It allocates contact slots for every geom pair that can collide after
  `contype`/`conaffinity`/`exclude`/`pair` filtering. Large scenes produce huge constraint
  arrays and slow compiles. `put_data` raises if the CPU data already has more contacts.
- Cap with custom numerics:
  ```xml
  <custom>
    <numeric name="max_geom_pairs" data="16"/>
    <numeric name="max_contact_points" data="32"/>
  </custom>
  ```
- Meshes use a branchless SAT: keep hulls <= ~200 vertices against primitives and
  < ~32 for mesh-mesh; `maxhullvert="64"` or less is the documented starting point.
- Reduce `iterations`/`ls_iterations` until just stable; Newton for GPU, CG for TPU.
- `jacobian`: auto picks sparse on TPU or `nv >= 60`; dense on GPU can be 10-20% faster.
- Disabling `eulerdamp` can help and is often not needed.
- Single-scene MJX-JAX can be ~10x slower than CPU MuJoCo; it pays off at thousands of envs.

Menagerie `*_mjx.xml` variants and rss2025 envs show these simplifications applied
(primitive collision geoms, explicit pairs, low iteration counts).

## MJX-Warp

```python
mx = mjx.put_model(m, impl="warp")
dx = mjx.make_data(m, impl="warp", naconmax=naconmax, njmax=njmax)  # pip install "mujoco-mjx[warp]"
```

- `naconmax`: contacts over all worlds (scale it by the vmap batch); `njmax`: constraints
  per world. Tune in the MJWarp viewer by increasing until no overflow.
- Contacts live in the private `mjx.Data._impl`; read them through contact sensors.
- Not differentiable. CUDA graph modes and batch rendering: `mjx.rst` anchors
  `MjxWarpGraphModes`, `MjxWarpBatchRendering`.

## MuJoCo Warp (`mujoco_warp`)

Install `pip install mujoco-warp` (depends only on `mujoco-core`); viewer and recorder:
`pip install "mujoco-warp[viewer]"` then `mjwarp-viewer model.xml`; benchmark:
`mjwarp-testspeed model.xml --measure_alloc`.

- Structs: `m = mjw.put_model(mjm)`, `d = mjw.put_data(mjm, mjd, nworld=N, nconmax=..., njmax=...)`
  or `mjw.make_data(mjm, nworld=N, ...)`. Fields are Warp arrays with a leading world axis;
  `d.qpos.numpy()` copies to host, `wp.copy`/`.assign` writes.
- Functions use PEP 8 names of the C API: `mjw.step`, `mjw.forward`, `mjw.reset_data`,
  `mjw.get_data_into(mjd, mjm, d, world_id=i)`.
- Sizes: `nconmax` (expected contacts per world; total cap `nconmax * nworld`, a world may
  exceed it if the total fits), `naconmax` (total; overrides `nconmax`), `njmax` (hard cap per
  world). XML `<size nconmax njmax>` is ignored (deprecated).
- Overflow: `d.overflow` is a per-world bitmask of `mjw.OverflowType` (e.g. `NARROWPHASE`).
  Overflow is undefined behaviour. Check it at reset/rollout boundaries, not every step (each
  `.numpy()` synchronises the device). `m.opt.warn_overflow = False` silences device printf.
- CUDA graphs: `with wp.ScopedCapture() as cap: mjw.step(m, d)` then
  `wp.capture_launch(cap.graph)`; typically much faster than eager launches.
- Solver iterations matter less than in MJX: worlds exit early once converged. Tune
  `nconmax`/`njmax` first.
- Unsupported (README, 4.0.1): `implicitfast` midpoint feature, PGS, plugin actuators and
  sensors, `noslip` with `discrete`; flex is experimental; no autodiff. `noslip` itself is
  implemented in the 4.0.1 release.
- Scaling: better than MJX on nearly every workload; single connected mechanisms beyond ~60
  DoF remain slow; scenes with many independent bodies benefit from `sleep`.
- CPU device works for debugging (verified on macOS arm64, Warp 1.19.0, "CUDA not enabled").

Per-world model fields (domain randomisation):

```python
m = mjw.put_model(mjm, batch_sizes={"dof_damping": nworld, "geom_friction": nworld})
m.dof_damping.assign(per_world_damping)          # shape (nworld, nv)
```

Batched fields are indexed `field[worldid % field.shape[0]]`, so a batch of 4 cycles over
the worlds. Structural changes (different meshes per world) follow the per-world assets
workflow in `mjwarp/index.rst` (`mjwBatch`): compile all variants from one `MjSpec` and copy
the dependent fields (`geom_dataid`, sizes, AABBs, body mass/inertia, ...) per world.

Batch renderer: ray-traced, many cameras per world, meshes/textures/heightfields/flex/Gaussian
splats; optional OptiX via `mujoco-warp[optix]` (not capturable in CUDA graphs).

## MuJoCo Playground

Playground 1.x (`pip install playground`, Python 3.12-3.13) is a PyTorch suite built on
`reaf` / `reaf-sim`, MuJoCo 4.x and MuJoCo Warp:

- `mujoco_playground/robots/`: `g1`, `so101`, `trossen_wxai`, `yam` with `sim/` and `real/`
  sides.
- `mujoco_playground/tasks/`: `manipulation/pick_cube/{so101,trossen_wxai}`, `whole_body`
  tracking, `teleop/so101`, `sysid/{so101,yam}`, `agentic/tabletop_v0`, `dm_control_suite`.
- Environment layers (`reaf.simulation.torch`): device layer (joints, actuators, cameras,
  sites, contact sensors bound to the model), task layer (features, rewards, terminations,
  discounts), torch wrapper; `EnvironmentReset` hooks (`on_startup`, `on_reset`, `on_step`,
  `check_termination`) get privileged model/data access for DR and NaN detection.
- Training: `python -m mujoco_playground.rl.rsl_rl.train --env=SO101PickCube --logdir=...`;
  playback in the Studio web viewer with `...rl.rsl_rl.play`. Deployment:
  `tasks/manipulation/pick_cube/so101/deploy` (`--mode=sim` / `--mode=real`).
- CUDA 12 vs 13 wheels matter (see README); macOS viewer needs `mjpython`.

The RSS 2025 JAX/MJX Playground (registry of `MjxEnv` tasks, Brax PPO training, DM Control
Suite, locomotion and manipulation) is preserved under `rss2025/` with its own
`pyproject.toml`. Existing code that imports `mujoco_playground.registry` targets that tree.

RL algorithm, reward and curriculum choices: `robot-reinforcement-learning`.

## CPU vs GPU parity checks

Before training at scale, step the same initial state on CPU and GPU for a few hundred
steps and compare `qpos`. Expect float32-level divergence that grows in contact-rich
scenes; a jump at the first contact usually means a missing feature or overflowed buffer.
The official `doc/skills/parity/SKILL.md` shipped in the `mujoco` wheel describes a
stage-by-stage method (`forward` isolation, contact matching).

## Executed MJX and MuJoCo Warp examples

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

What differs from CPU MuJoCo (details and full tables in the sections above):

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
