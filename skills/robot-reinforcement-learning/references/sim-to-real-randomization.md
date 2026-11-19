# Sim-to-Real: Randomisation Parameters and Reality-Gap Audit

Checked 2026-11-18. Two sets of shipped values are quoted from source so you have a real anchor:
legged_gym (`legged_gym/envs/base/legged_robot_config.py`, Isaac Gym era) and the Isaac Lab
velocity-tracking task (`isaaclab_tasks/core/velocity/velocity_env_cfg.py`). Ranges for other
parameters are heuristics: start from measured values on your robot and widen until training
just starts to slow down, not further.

## What to Randomise

| Parameter | legged_gym default | Isaac Lab velocity task | Heuristic range / method | Failure it prevents |
|---|---|---|---|---|
| Ground friction | `friction_range=[0.5, 1.25]` | static `(0.8, 0.8)`, dynamic `(0.6, 0.6)` (fixed in base cfg; robot cfgs override) | 0.3-1.5 for indoor/outdoor legged; measure for manipulation | Slipping, policies tuned to one floor |
| Base / link mass | `added_mass_range=[-1, 1]` kg (off by default) | base mass scaled log-uniform `(1/1.25, 1.25)` | +-10-25% plus payload range you will carry | Payload sensitivity |
| Centre of mass | - | base CoM x,y `+-0.05` m, z `+-0.01` m | +-2-5 cm | Tipping, biased gaits |
| External pushes | every 15 s, `max_push_vel_xy=1.0` m/s | every 10-15 s, base velocity `+-0.5` m/s | Scale up with a curriculum | Fragile balance |
| Initial joint state | - | position scaled `(0.5, 1.5)` of default | Cover the states your real reset produces | Failing from non-nominal starts |
| Observation noise | joint pos 0.01, joint vel 1.5, lin vel 0.1, ang vel 0.2, gravity 0.05, heights 0.1 | uniform: lin vel `+-0.1`, ang vel `+-0.2`, gravity `+-0.05`, joint pos `+-0.01`, joint vel `+-1.5`, height scan `+-0.1` | Measure sensor noise on the robot; add 1-2x | Over-trusting clean sim state |
| Motor strength / torque scale | - | - | 0.8-1.2 multiplier | Weak or saturated real actuators |
| PD gains | - | - | +-10-20% around nominal Kp/Kd | Different tracking stiffness on hardware |
| Action latency | - | - | Uniform 0 to (measured latency + one control period); implement with a FIFO buffer | Oscillation, phase lag; the most common silent gap |
| Observation delay | - | - | Same buffer idea on sensors, especially cameras (often 30-100 ms) | Policies reacting to stale state |
| Joint friction / damping / armature | - | - | Identify, then randomise +-20-50% | Sluggish or overshooting joints |
| Restitution | `(0, 0)` | `(0, 0)` | 0-0.3 for impacts | Bouncy contacts |
| Terrain | heightfield curriculum | rough-terrain generator + curriculum | Include anything the robot will meet | Tripping on real ground |
| Object properties (manipulation) | - | - | Mass, size, friction, initial pose | Overfitting to one object |
| Visual (pixels only) | - | - | Lighting, textures, camera pose +-1-3 cm / few degrees, distractors | Vision policies breaking on the real scene |

Isaac Lab's `mode="startup"` events are sampled once per env at launch, `mode="reset"` per
episode, and `mode="interval"` periodically (pushes). Per-episode resampling is the usual choice
for dynamics. See the `isaac-sim` and `mujoco` skills for the simulator-side APIs.

## Latency Buffer Pattern (simulator-agnostic)

```python
from collections import deque
import random

class DelayBuffer:
    """Delay actions (or observations) by k policy steps, k resampled per episode."""

    def __init__(self, max_delay_steps):
        self.max_delay = max_delay_steps
        self.reset(None)

    def reset(self, initial):
        self.k = random.randint(0, self.max_delay)
        self.buf = deque([initial] * (self.max_delay + 1), maxlen=self.max_delay + 1)

    def push_pop(self, x):
        self.buf.append(x)
        return self.buf[-1 - self.k]   # k=0 -> no delay
```

Sub-step delays (latency smaller than one control period) need the buffer at simulation rate
inside the decimation loop, not at policy rate.

## System Identification Before Randomisation

1. Log real trajectories: joint position/velocity/current under chirp or step targets, with
   the same PD gains and control rate as deployment.
2. Replay the same commands open-loop in sim; overlay curves.
3. Fit joint damping, friction, armature, motor constants, and latency (MuJoCo ships
   `mujoco.sysid`, used in MuJoCo Playground's sysid task; see the `mujoco` skill).
4. Centre the randomisation ranges on the identified values. Randomising around wrong nominal
   values wastes the policy's capacity on robustness it does not need.
5. If the actuator is hard to model (series-elastic, hydraulic, high-gear-ratio with backlash),
   train an actuator network on logged (position error history, velocity history) -> torque
   data and use it in sim (Hwangbo et al. 2019, arXiv 1901.08652).

## Reality-Gap Audit (run before every hardware trial)

| Check | How |
|---|---|
| Same control rate | Policy step period on robot equals `sim_dt * decimation`; measure jitter |
| Same action semantics | Joint order, sign conventions, default pose offset, `action_scale`, clipping, gripper mapping identical in sim and deploy code. Unit-test with a recorded sim (obs, action) pair |
| Same observation pipeline | Same units, frames (base vs world), normaliser statistics, history length and ordering, previous-action term |
| Same PD gains | Sim Kp/Kd equal the values flashed to motor drivers (watch unit conversions, per-joint gains) |
| Torque / velocity limits | Sim limits no looser than hardware |
| Latency measured | Timestamp sensor read -> action applied on the robot; included in randomisation |
| Filters | Any action filter used on the robot was present in training |
| State estimator | Policy trained on noise/bias like the real estimator (or on estimator outputs in sim) |
| Replay test | Feed logged real observations to the policy offline; compare action statistics with sim |
| Safety | E-stop, joint limits, torque limits, and a watchdog that holds position if the policy stalls |

## Moved From SKILL.md

The subsections below were moved verbatim from SKILL.md section 5.

### Domain Randomisation: What to Randomise

Randomise what you cannot measure precisely, centred on what you can:

- **Dynamics:** friction, link masses and payload, centre of mass, joint damping/friction/armature,
  motor strength (torque scale), PD gains.
- **Timing:** action latency and observation delay (often the largest unmodelled gap), control
  jitter.
- **Sensing:** observation noise and bias per channel, IMU drift, dropped frames; camera pose,
  lighting and textures for pixel policies.
- **Disturbances:** pushes, terrain, initial states, object properties.

### System Identification and Actuator Models

1. Identify first, randomise second: fit damping, friction, armature, latency from logged real
   trajectories under the same PD gains and rate (MuJoCo's `mujoco.sysid`; see `mujoco`).
2. When the actuator cannot be modelled analytically (series-elastic, backlash, hydraulics),
   learn an actuator network from (position-error history, velocity history) -> torque and use
   it in sim (Hwangbo et al., arXiv 1901.08652).
3. Dynamics randomisation for manipulation: Peng et al., arXiv 1710.06537. Visual randomisation:
   Tobin et al., arXiv 1703.06907.

### Smoothness and Action Filtering

- Penalise action rate and joint acceleration in the reward; consider explicit smoothness
  regularisation of the policy (CAPS, arXiv 2012.06644).
- A low-pass or exponential filter on actions must be **in the training loop** if it is on the
  robot; adding it only at deployment changes the dynamics the policy learned.
- Check the action spectrum in sim: energy near the Nyquist rate of the control loop will
  excite unmodelled resonances on hardware.
