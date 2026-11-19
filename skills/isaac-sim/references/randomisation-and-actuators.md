# Randomisation and Actuators

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.

## Event modes

Event modes (`EventTermCfg.mode`): `prestartup` authors USD before sim start (scale, some
materials). `startup` runs once after sim start (mass, friction buckets). `reset` runs per
episode. `interval` runs during an episode, with `interval_range_s`.

## Example events config

```python
@configclass
class EventsCfg:
    friction = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": (0.6, 1.2),
                "dynamic_friction_range": (0.5, 1.0), "restitution_range": (0.0, 0.1), "num_buckets": 64})
    base_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "mass_distribution_params": (1 / 1.25, 1.25), "operation": "scale", "distribution": "log_uniform"})
    gains = EventTerm(func=mdp.randomize_actuator_gains, mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"), "stiffness_distribution_params": (0.8, 1.2),
                "damping_distribution_params": (0.8, 1.2), "operation": "scale"})
    push = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}})
```

The full list of built-in event functions and their parameters is in
`references/config-reference.md`. Confirm the parameter names in `isaaclab/envs/mdp/events.py`.

Backend differences that bite: PhysX material randomisation is bucketed and runs on CPU
tensors. Newton has one friction coefficient, so it ignores `dynamic_friction_range` and
`num_buckets`, and it writes position-only CoM changes. Give each backend its own event set
through a `PresetCfg` events config.

## Actuator models

Actuator models (`isaaclab.actuators`):

| Cfg | Physics | Use |
|-----|---------|-----|
| `ImplicitActuatorCfg` | PD inside the solver (stable at high gains) | Default for position-controlled arms. Gains are the drive stiffness/damping |
| `IdealPDActuatorCfg` | Explicit PD torque, computed by Isaac Lab | When you need the computed torque, or torque-level effects |
| `DCMotorCfg` | Explicit PD plus a torque-speed curve (`saturation_effort`, `actuator_velocity_limit`) | Legged robots with real motor limits |
| `DelayedPDActuatorCfg` | Explicit PD with `min_delay`/`max_delay` in **physics steps** | Actuation latency randomisation |
| `RemotizedPDActuatorCfg` | Delayed PD with a joint-angle-dependent torque lookup | Linkage-driven joints (knees) |
| `ActuatorNetMLPCfg` / `ActuatorNetLSTMCfg` | Learned actuator model from `network_file` (TorchScript) | ANYmal-style sim-to-real with identified drives |

- `actuator_effort_limit` clips the explicit model output. `joint_effort_limit` is the solver
  clamp. In 3.0 an explicit group **keeps** the solver limit, so effort is clipped twice. Set
  `joint_effort_limit >= actuator_effort_limit`.
- Explicit actuators need a small `sim.dt`. The stock legged tasks use 0.005 s. **Heuristic:**
  a stiff explicit PD at 1/60 s oscillates and explodes, while implicit PD tolerates larger steps.
- Add `armature` (reflected rotor inertia) for geared joints. Its absence is a common cause of
  jitter and of sim-to-real mismatch.

Sim-to-real hooks:

- Observation noise per term (`noise=`, active only with `enable_corruption=True`).
- Observation latency (`ObsTerm(delay_min_lag=, delay_max_lag=)`, in observation samples) and
  action latency (`DelayedPDActuatorCfg`).
- Direct envs: `observation_noise_model` and `action_noise_model`
  (`NoiseModelWithAdditiveBiasCfg`).
- Actuator nets for identified drives.
- PhysX↔Newton policy transfer as a cheap sim-to-sim check before hardware
  (`docs/source/how-to/transfer_policies_between_physx_and_newton.rst`).

Which parameters to randomise, by how much, and why: `robot-reinforcement-learning`.
