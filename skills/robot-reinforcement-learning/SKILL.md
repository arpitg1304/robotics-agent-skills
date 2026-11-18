---
name: robot-reinforcement-learning
description: >
  Reinforcement learning for robots: deciding when RL beats imitation or classical control,
  MDP design (observations, action spaces, control rate, terminated vs truncated), reward
  design and reward hacking, PPO and SAC/TD3 selection, massively parallel sim training,
  domain randomisation and sim-to-real, safe real-robot RL, and debugging training curves.
  Use this skill when the user mentions reinforcement learning, RL, PPO, SAC, TD3, GAE,
  reward function, reward shaping, curriculum, domain randomization, sim-to-real, actuator
  network, asymmetric actor-critic, teacher-student distillation, rsl_rl, rl_games, skrl,
  Stable-Baselines3, CleanRL, Brax training, FastTD3, HIL-SERL, SERL, RLPD, DrQ, residual RL,
  DSRL, RL fine-tuning of a BC, diffusion or VLA policy, time-out bootstrapping, entropy
  collapse, value loss explosion, NaN during training, or evaluating RL with seeds and IQM.
  Also for locomotion and manipulation policies trained in Isaac Lab, mjlab or Playground.
---

# Robot Reinforcement Learning

Library landscape checked 2026-11-18. Versions, defaults and paper claims below were read from
the official repositories and arXiv on that date; anything marked **heuristic** is practitioner
experience, not a guarantee. Re-check versions before pinning.

RL on robots fails far more often from a wrong MDP, a hackable reward or a sim that does not
match the robot than from the choice of algorithm. Spend effort in that order.

## When to Use This Skill

- Deciding whether a robot task should use RL, imitation learning, classical control, or a hybrid
- Writing or reviewing an RL environment: observations, actions, rewards, terminations, resets
- Choosing PPO vs SAC/TD3, and rsl_rl vs rl_games vs skrl vs SB3 vs CleanRL vs Brax
- Training locomotion or manipulation in massively parallel GPU simulation
- Planning sim-to-real: domain randomisation, system identification, latency, actuator models
- Running RL on a physical robot (HIL-SERL style) with safety layers and learned rewards
- Fine-tuning a behaviour-cloned, diffusion or VLA policy with RL
- Diagnosing training that plateaus, collapses, produces NaNs, or exploits the reward
- Reporting results with multiple seeds and confidence intervals

## Reference Files

Load these only when the task needs the detail:

| File | Load when |
|---|---|
| [references/hyperparameters.md](references/hyperparameters.md) | Setting or tuning PPO / SAC / distillation hyperparameters; comparing to shipped Isaac Lab and SB3 values |
| [references/library-comparison.md](references/library-comparison.md) | Choosing or switching RL libraries; interop bugs (obs groups, time-out flags, autoreset, normaliser export) |
| [references/sim-to-real-randomization.md](references/sim-to-real-randomization.md) | Writing domain randomisation, latency buffers, system identification, or a pre-hardware reality-gap audit |

## 1. Mental Model

An RL project has four things you design, in decreasing order of how often they are the bug:

```
  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐   ┌───────────────┐
  │ Data          │   │ MDP           │   │ Reward        │   │ Algorithm     │
  │ distribution  │ > │ obs, actions, │ > │ terms, weights│ > │ PPO/SAC, nets,│
  │ sim + DR +    │   │ rate, resets, │   │ curriculum,   │   │ hyperparams   │
  │ resets        │   │ terminations  │   │ termination   │   │               │
  └───────────────┘   └───────────────┘   └───────────────┘   └───────────────┘
```

The optimiser will find the cheapest route to reward inside the simulator you give it,
including routes you did not intend. Treat every learned behaviour as "correct for the MDP I
wrote" until proven otherwise.

### Is RL the Right Tool?

| Situation | Prefer | Why |
|---|---|---|
| Accurate model, cost is easy to write, need guarantees or interpretability | Classical control (MPC, WBC, IK + trajectory optimisation) | No training, predictable, certifiable |
| Easy to teleoperate, reward hard to write, perception-heavy manipulation | Imitation learning (see `imitation-learning`, `lerobot`) | Demos encode intent directly |
| Dynamics hard to model but cheap to simulate (legged locomotion, in-hand manipulation, agile flight) | RL in parallel sim + sim-to-real | Policy learns robustness the model cannot express |
| Demonstrations impossible or below the required performance (speed, recovery, super-human agility) | RL | RL can exceed the demonstrator |
| BC policy works 60-80% and you need 95%+ on one setup | Hybrid: RL fine-tuning on real or sim data | Cheaper than many more demos when success is detectable |
| Reward needs a human to judge every episode | Imitation, or learned reward with heavy validation | Real-world RL needs automatic reward |

### Hybrid Approaches (fast-moving area; verify before committing)

| Approach | Idea | Representative work |
|---|---|---|
| RL with demonstrations in the buffer | Off-policy RL sampling demos and online data together | RLPD (arXiv 2302.02948): 50/50 sampling, LayerNorm critics, critic ensembles |
| Human-in-the-loop real-world RL | Demos + learned success classifier + human interventions + SAC | SERL (2401.16013), HIL-SERL (2410.21845); implemented in LeRobot |
| Residual RL | RL learns a correction added to a fixed base controller or BC policy | Johannink et al. (1812.03201), Silver et al. (1812.06298), ResiP (2407.16677), Policy Decorator (2412.13630) |
| Latent-noise steering | RL chooses the input noise of a frozen diffusion/flow policy | DSRL (2506.15799); black-box access to the base policy |
| Direct fine-tuning of diffusion policies | Treat denoising steps as an MDP and run PPO | DPPO (2409.00588) |
| RL for VLAs | Advantage-conditioned training on autonomous + intervention data; outcome-reward RL in sim | pi*0.6 / RECAP (2511.14759); SimpleVLA-RL (2509.09674, mostly simulation results); ConRFT (2502.05450) |
| Teacher-student | RL teacher with privileged state, distilled into a deployable student | Lee et al. 2020 (2010.11251), RMA (2107.04034) |
| Motion imitation reward | RL tracks reference motions (mocap, retargeted human data) | DeepMimic (1804.02717); humanoid whole-body tracking |

Most headline numbers for VLA RL fine-tuning come from the authors' own papers. Treat them as
evidence a method can work, not that it will work on your robot.

## 2. MDP Design

### Observations

| Group | Typical contents | Rules |
|---|---|---|
| Proprioception | Joint positions relative to default pose, joint velocities, base angular velocity, projected gravity, previous action | Express in body frame; include the previous action so the policy can reason about its own command |
| Task / command | Goal pose relative to the robot, velocity command, object pose relative to gripper | Relative, not world-frame: world coordinates make the policy memorise positions |
| Exteroception | Height scan, depth, camera images | Must exist on the real robot at the same rate and latency |
| History | Last N proprio frames or a recurrent state | Needed when latency, contact state or dynamics parameters are unobserved |
| Privileged (critic or teacher only) | Friction, masses, true base velocity, contact forces, terrain heights | Never in the deployed actor unless the robot estimates it |

- Scale every input to roughly unit range (hand-chosen scales or a running normaliser, below).
  Raw joint velocities of 20 rad/s next to quaternions of magnitude 1 slow learning.
- Base linear velocity is easy in sim and hard on hardware. Isaac Lab's velocity task observes
  it (with +-0.1 noise); a deployed policy needs a state estimator that matches, or drop it.
- If the time limit is part of the task (must finish by T), add remaining time to the
  observation (Pardo et al., arXiv 1712.00378). If it is only a training convenience, leave it
  out and bootstrap at truncation.

### Action Spaces

| Action space | Use for | Pros | Cons |
|---|---|---|---|
| Joint position target `q_default + scale * a`, PD at higher rate | Default for legged and humanoid | Stable exploration, transfers well; Isaac Lab velocity task uses `scale=0.5` | Policy cannot command torque directly; PD gains become part of the MDP |
| Joint position delta from current `q` | Arms, slow tasks | Small steps are safe | Drift; integral behaviour harder to learn |
| Joint torque | High-rate, well-identified actuators | Full authority | Needs high control rate and accurate actuator model; transfers worst |
| End-effector delta (pose twist) via IK / impedance / OSC | Real-robot manipulation RL | Safe workspace bounds, small action dim; HIL-SERL uses EE twists with an impedance controller | Controller quality limits the policy |
| Discrete gripper open/close | Grasping | Avoids chattering half-closed grippers | HIL-SERL uses a separate DQN-style critic for it |

- **Action scale is a hyperparameter.** Too large: the initial std (often 1.0) slams joints to
  limits on step one. Too small: the policy cannot reach the needed poses and saturates.
- **Clip in the environment**, then apply. Log the fraction of clipped actions.
- PPO's Gaussian is unbounded; SAC's is tanh-squashed to [-1, 1]. Rescale to physical units in
  one place, shared by training and deployment code.

### Control Rate

- Legged default (heuristic, matches Isaac Lab and legged_gym): physics at 200 Hz
  (`dt=0.005`), policy at 50 Hz (`decimation=4`), PD inside the physics loop.
- Real-robot manipulation RL: commonly 10-30 Hz with EE deltas (LeRobot's HIL-SERL env config
  defaults to `fps=30`).
- Higher policy rate means more steps per second of behaviour: raise gamma so the effective
  horizon `1/(1-gamma)` steps still covers the behaviour (0.99 at 50 Hz is ~2 s).

### Terminations, Truncations and the Time-Out Bug

| Event | Gymnasium flag | Bootstrap? | Example |
|---|---|---|---|
| MDP genuinely ends (absorbing state) | `terminated=True` | No: future value is 0 | Robot fell, object dropped off table, joint limit violated |
| Episode cut for convenience | `truncated=True` | **Yes**: use V(final observation) | Time limit, curriculum reset, env reset for logging |

Treating a time-out as terminal teaches the critic that states near the time limit are worth
almost nothing, so the policy learns time-dependent, myopic behaviour and the value loss never
settles. Every major stack has a channel for this: Isaac Lab marks the time-out termination
with `time_out=True`, rsl_rl reads `extras["time_outs"]` and adds `gamma * V` to the reward on
those steps, SB3 replay buffers read `info["TimeLimit.truncated"]`
(`handle_timeout_termination=True` by default). Custom wrappers break it most often.

Gymnasium 1.x vector envs default to `AutoresetMode.NEXT_STEP`: the step after an episode ends
returns the reset observation, and that transition must be masked out. In `SAME_STEP` mode,
bootstrap from `info["final_obs"]`, never from the returned (already reset) observation.

### Example: Environment Skeleton

Dependency-free so it runs anywhere (gymnasium is not required; the API matches
`gymnasium.Env`: `reset() -> (obs, info)`, `step() -> (obs, reward, terminated, truncated, info)`).
Subclass `gymnasium.Env` and add `observation_space` / `action_space` in real code.

```python
import math
import random


class JointReachEnv:
    """1-DoF joint reaching task with a gymnasium-style API, standard library only."""

    sim_dt = 0.005          # physics step [s]
    decimation = 4          # physics steps per policy step -> 50 Hz policy
    max_episode_steps = 250 # 5 s episode -> time limit is a TRUNCATION
    action_scale = 1.6      # rad of target offset from default pose per unit action
    q_default = 0.0
    kp, kd, torque_limit = 20.0, 1.0, 5.0
    q_limit = 2.5           # rad; leaving this is a failure TERMINATION
    weights = {"track": 1.0, "action_rate": -0.01, "torque": -1e-4, "success": 0.5}

    def __init__(self):
        self.rng = random.Random()

    def reset(self, seed=None):
        if seed is not None:
            self.rng.seed(seed)
        self.q, self.qd = self.rng.uniform(-0.5, 0.5), 0.0
        self.goal = self.rng.uniform(-1.5, 1.5)
        self.inertia = 0.1 * self.rng.uniform(0.8, 1.2)   # domain randomisation
        self.prev_action, self.t, self.ep_terms = 0.0, 0, {}
        return self._obs(), {}

    def _obs(self):
        # Goal error, joint pos rel. to default, velocity, previous action (Markov for this task).
        return [self.goal - self.q, self.q - self.q_default, self.qd, self.prev_action]

    def step(self, action):
        a = max(-1.0, min(1.0, float(action)))           # clip in the env, not only in the policy
        q_target = self.q_default + self.action_scale * a  # absolute joint target around default pose
        tau = 0.0
        for _ in range(self.decimation):                 # PD runs at sim rate, policy at 50 Hz
            tau = self.kp * (q_target - self.q) - self.kd * self.qd
            tau = max(-self.torque_limit, min(self.torque_limit, tau))
            self.qd += tau / self.inertia * self.sim_dt
            self.q += self.qd * self.sim_dt
        self.t += 1

        err = self.goal - self.q
        success = abs(err) < 0.02 and abs(self.qd) < 0.05
        failed = abs(self.q) > self.q_limit
        terms = {                                         # weighted, per-step, logged separately
            "track": self.weights["track"] * math.exp(-err * err / 0.25),
            "action_rate": self.weights["action_rate"] * (a - self.prev_action) ** 2,
            "torque": self.weights["torque"] * tau * tau,
            "success": self.weights["success"] * float(success),
        }
        reward = sum(terms.values())
        for k, v in terms.items():
            self.ep_terms[k] = self.ep_terms.get(k, 0.0) + v
        self.prev_action = a

        # Success is NOT terminal here: with positive per-step reward, ending on success
        # would teach the policy to hover just outside the tolerance and keep collecting.
        terminated = failed                              # true end of the MDP: do NOT bootstrap
        truncated = (not terminated) and self.t >= self.max_episode_steps  # time limit: DO bootstrap
        info = {"reward_terms": terms, "success": success, "failed": failed}
        if terminated or truncated:
            info["episode_reward_terms"] = dict(self.ep_terms)
        return self._obs(), reward, terminated, truncated, info


if __name__ == "__main__":
    env = JointReachEnv()
    obs, _ = env.reset(seed=0)
    done = False
    while not done:
        # Scripted check: command the goal as the joint target (goal = err + q).
        obs, r, terminated, truncated, info = env.step((obs[0] + obs[1]) / env.action_scale)
        done = terminated or truncated
    print(env.t, terminated, truncated, info["success"],
          {k: round(v, 3) for k, v in info["episode_reward_terms"].items()})
```

Running it prints `250 False True True {...}`: a scripted controller reaches the goal, the
episode ends by truncation, and each reward term is reported separately. **Always run a
scripted or random policy through a new env before training**; it catches sign errors,
unreachable goals and broken resets in seconds instead of hours.

### Example: GAE With Correct Terminated / Truncated Handling

```python
def compute_gae(rewards, values, next_values, terminated, truncated, gamma=0.99, lam=0.95):
    """GAE over one environment's rollout of length T.

    next_values[t] = V(s_{t+1}) for the TRUE next state: at an episode end, the final
    observation (SAME_STEP autoreset: info["final_obs"]), never the next episode's reset obs.
    terminated[t] -> no bootstrap.  truncated[t] -> bootstrap, but stop the recursion.
    """
    T = len(rewards)
    adv, last = [0.0] * T, 0.0
    for t in reversed(range(T)):
        not_terminal = 1.0 - float(terminated[t])
        episode_continues = 1.0 - float(terminated[t] or truncated[t])
        delta = rewards[t] + gamma * not_terminal * next_values[t] - values[t]
        last = delta + gamma * lam * episode_continues * last
        adv[t] = last
    returns = [a + v for a, v in zip(adv, values)]
    return adv, returns


if __name__ == "__main__":
    # Reward 1 forever -> true V = 1/(1-gamma) = 100. A 3-step time limit cuts the episode.
    V = 100.0
    _, ok = compute_gae([1.0] * 3, [V] * 3, [V] * 3, [0, 0, 0], [0, 0, 1], gamma=0.99, lam=1.0)
    _, bug = compute_gae([1.0] * 3, [V] * 3, [V] * 3, [0, 0, 1], [0, 0, 0], gamma=0.99, lam=1.0)
    print("truncation bootstrapped  :", [round(x, 2) for x in ok])    # [100.0, 100.0, 100.0]
    print("time-out treated terminal:", [round(x, 2) for x in bug])   # [2.97, 1.99, 1.0]
```

rsl_rl (v5.5.2 `ppo.py`) approximates the bootstrap by adding `gamma * V(s_t)` (the value of
the current state, not of the final observation) to the reward on time-out steps. That is a
reasonable approximation when consecutive states have similar value; the exact version needs
the final observation.

### Resets

- **Sim:** randomise initial states broadly (Isaac Lab scales default joint positions by
  0.5-1.5 at reset). Start some episodes from states along demonstrations or reference motions
  (reference state initialisation, DeepMimic) when the task has hard-to-reach phases.
- **Early termination** on falls or unrecoverable states speeds learning by not wasting samples,
  but interacts with reward sign (section 3).
- **Real robots:** resets are the bottleneck; see section 6.

## 3. Reward Design

### Structure

```
r_t = sum_i w_i * term_i(s_t, a_t, s_{t+1})
      task terms:  bounded tracking kernels, e.g. exp(-||err||^2 / sigma), success bonus
      regularisers: small penalties on action rate, torque, joint acceleration, power, limits
```

The Isaac Lab velocity task is a good template: `track_lin_vel_xy_exp` (weight 1.0,
std sqrt(0.25)), `track_ang_vel_z_exp` (0.5), and penalties such as `lin_vel_z_l2` (-2.0),
`action_rate_l2` (-0.01), `dof_torques_l2` (-1e-5), `dof_acc_l2` (-2.5e-7).

- **Dense vs sparse:** dense rewards learn fast but encode your solution (and your mistakes).
  Sparse success rewards are honest but need demos, curricula or very cheap samples. Real-world
  RL (SERL/HIL-SERL) uses sparse binary success plus demonstrations.
- **Potential-based shaping** `F = gamma * Phi(s') - Phi(s)` (Ng, Harada and Russell, ICML 1999)
  provably preserves the optimal policy. Use it for "distance to goal" style shaping; set Phi to
  0 at terminal states. Arbitrary shaping terms do not have this guarantee.
- **Normalise term magnitudes:** log each term's per-episode sum at initialisation. If one
  penalty dominates the task term before learning starts, the first thing learned is "do
  nothing".
- **Curricula:** ramp terrain difficulty, command ranges, penalty weights (start small, increase
  once the task is solved), and randomisation ranges (automatic domain randomisation,
  arXiv 1910.07113). Advance on measured success, not on iteration count.

### Reward and Termination Interact

| Reward sign per step | Termination | What the agent learns | Fix |
|---|---|---|---|
| Mostly negative | Failure ends episode | Fall early to stop paying penalties | Make per-step reward positive (legged_gym's `only_positive_rewards=True` clips the total at zero for this reason), or add a termination penalty larger than the remaining penalties |
| Mostly positive | Success ends episode | Avoid success; hover near the goal | Success bonus larger than the reward left in the episode, or make success non-terminal |
| Any | Time-out treated as terminal | Time-dependent, myopic behaviour | Bootstrap truncation (section 2) |

### Reward Hacking: Examples and Detection

| Hack | Reward that invited it | Detection |
|---|---|---|
| Feet vibrating or tapping to farm a feet-air-time term | Air-time reward without contact-force or slip penalty | Video; foot contact frequency far above gait frequency |
| High-frequency jitter that exploits contact solver penetration | Velocity tracking with weak action-rate penalty | Action spectrum; joint power; replay at lower sim dt |
| Knocking the object off the table | "Object moved toward goal" without bounds | Object termination counts; final-state images |
| Pushing the object with the arm instead of grasping | Object-pose reward only | Gripper-contact statistics |
| Occluding the camera with the gripper | Image success classifier with no hard negatives | Classifier score vs independent check; review positives |
| Standing still | Penalties dominate a weak task term | Task term near zero while penalties shrink |
| Walking on knees, dragging a link | No undesired-contact penalty | Contact sensor on non-foot bodies (Isaac Lab `undesired_contacts`) |

Detection habits:

- Log every reward term separately per episode (Isaac Lab logs `Episode_Reward/<term>`).
- Record a rollout video every N iterations and actually watch it.
- Track physical sanity metrics **not in the reward**: success from an independent check,
  max contact force, joint power, foot slip, action saturation fraction.
- If the reward curve rises while the success rate is flat, assume hacking until disproven.

## 4. Algorithm and Library Choice

### PPO for Massively Parallel Sim

On-policy PPO with thousands of GPU environments is the default for locomotion and much of
dexterous manipulation (Rudin et al., arXiv 2109.11978, trained ANYmal in minutes this way).
Key mechanics and where they bite:

- **GAE** (gamma, lambda) as above; time-out bootstrapping is essential because rollouts are
  short (Isaac Lab ANYmal: 24 steps per env, 4096 envs).
- **Advantage normalisation** per batch: keep on.
- **Observation normalisation:** a running mean/std, frozen and exported with the policy.
- **Entropy and action std:** rsl_rl's default Gaussian learns a state-independent std
  (clamped to a range) starting at `init_std=1.0`
  with `entropy_coef=0.005`. Log the mean std every iteration; it is the best single health
  signal.
- **KL control:** rsl_rl's adaptive schedule divides the learning rate by 1.5 when the KL
  exceeds `2 * desired_kl` and multiplies it by 1.5 when below `desired_kl / 2`
  (`desired_kl=0.01`, LR bounded to [1e-5, 1e-2]). SB3 uses `target_kl` for early stopping instead.
- **Clipping:** `clip_param=0.2`, gradient norm clip 1.0. Rarely the first thing to tune.

Full tables: [references/hyperparameters.md](references/hyperparameters.md).

### Off-Policy for Real Robots and Expensive Sims

| Method | Use when | Key ingredients |
|---|---|---|
| SAC / TD3 | Few environments, expensive samples | Replay buffer, target networks, auto-tuned entropy (SAC) |
| DrQ / DrQ-v2 (2004.13649, 2107.09645) | Pixel observations | Random-shift image augmentation on critic and actor inputs |
| RLPD (2302.02948) | You have demos or prior data | 50/50 demo/online batches, LayerNorm critics, 10-critic ensemble, UTD 20 in its example commands |
| SERL / HIL-SERL | Real-robot manipulation | RLPD-style SAC, pretrained ResNet-10 encoder, binary success classifier, EE impedance control, human interventions |
| FastTD3 (2505.22642) / FastSAC (2512.01996) | Massively parallel sim where wall-clock matters | Large batches, many envs, tuned off-policy; reported humanoid locomotion in ~15 min on one RTX 4090 (FastSAC paper) |

### Structural Tricks

- **Asymmetric actor-critic** (Pinto et al., arXiv 1710.06542): critic sees privileged state,
  actor sees only deployable observations. Nearly free in sim; in rsl_rl v5 route an extra
  observation group to the critic via `obs_groups`.
- **Teacher-student distillation:** train a privileged teacher with RL, then train a student on
  deployable inputs (proprio history, depth) with DAgger-style supervision (rsl_rl ships a
  `Distillation` algorithm). Use when the actor needs information it can only infer over time.
- **Adaptation modules (RMA):** student encodes history into an estimate of the teacher's
  latent environment parameters.
- **Symmetry augmentation:** mirror states/actions for bilateral robots (rsl_rl `symmetry_cfg`).
- **Recurrent policies or history stacks:** needed under latency, partial observability or
  randomised dynamics; start with a short stacked history before reaching for an LSTM.

### Library Choice (short version)

| You have | Start with |
|---|---|
| Isaac Lab, mjlab or MuJoCo Playground 1.x, locomotion/humanoid | rsl_rl (v5.5.2) PPO |
| Isaac Lab dexterous manipulation with existing configs | rl_games (v1.6.5) |
| Need JAX end to end with MJX | Brax training (v0.14.2); Playground's original JAX envs live under `rss2025/` |
| Need many algorithms, multi-agent, or JAX/Warp backends | skrl (2.1.2) |
| CPU Gymnasium env, quick reliable baseline | Stable-Baselines3 (v2.9.2) + sb3-contrib |
| Want to read or modify the algorithm | CleanRL single files (stable reference, last release 2022) |
| LeRobot arm, real-world RL | LeRobot (v0.7.0) HIL-SERL (`python -m lerobot.rl.learner` / `lerobot.rl.actor`) |

Simulator setup belongs in the `isaac-sim` and `mujoco` skills; details and interop gotchas
are in [references/library-comparison.md](references/library-comparison.md).

### Example: Running Observation Normaliser

```python
import math


class RunningMeanStd:
    """Per-dimension running mean/variance with a batched (Chan et al.) update."""

    def __init__(self, dim, eps=1e-4, clip=5.0):
        self.mean, self.var, self.count = [0.0] * dim, [1.0] * dim, eps
        self.clip = clip

    def update(self, batch):  # batch: list of observation vectors
        n = len(batch)
        if n == 0:
            return
        for i in range(len(self.mean)):
            col = [x[i] for x in batch]
            b_mean = sum(col) / n
            b_var = sum((c - b_mean) ** 2 for c in col) / n
            delta, tot = b_mean - self.mean[i], self.count + n
            m2 = self.var[i] * self.count + b_var * n + delta * delta * self.count * n / tot
            self.mean[i] += delta * n / tot
            self.var[i] = m2 / tot
        self.count += n

    def normalize(self, x):
        return [max(-self.clip, min(self.clip, (xi - m) / math.sqrt(v + 1e-8)))
                for xi, m, v in zip(x, self.mean, self.var)]

    def state_dict(self):  # ship this WITH the policy weights; deployment must use the same stats
        return {"mean": list(self.mean), "var": list(self.var), "count": self.count}
```

Use NumPy or torch in real code. The failure this prevents is not numerical, it is
operational: update statistics only during training, freeze them for evaluation, and export
them with the checkpoint (or bake them into the ONNX graph). A policy deployed without its
normaliser outputs confident garbage.

## 5. Sim-to-Real

### Domain Randomisation: What to Randomise

Randomise what you cannot measure precisely, centred on what you can:

- **Dynamics:** friction, link masses and payload, centre of mass, joint damping/friction/armature,
  motor strength (torque scale), PD gains.
- **Timing:** action latency and observation delay (often the largest unmodelled gap), control
  jitter.
- **Sensing:** observation noise and bias per channel, IMU drift, dropped frames; camera pose,
  lighting and textures for pixel policies.
- **Disturbances:** pushes, terrain, initial states, object properties.

Too-wide randomisation produces timid, slow policies; too-narrow produces policies that fail on
contact with hardware. Widen until training slows, then stop (heuristic). Shipped legged ranges
and a latency buffer: [references/sim-to-real-randomization.md](references/sim-to-real-randomization.md).

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

### Reality-Gap Checklist (short form; full audit in the reference file)

- [ ] Control period on robot equals `sim_dt * decimation`, jitter measured
- [ ] Joint order, signs, default pose, action scale and clipping identical in sim and deploy code
- [ ] Observation units, frames, history ordering and normaliser stats identical
- [ ] PD gains in sim equal the gains on the motor drivers
- [ ] Latency measured and randomised over
- [ ] Logged real observations replayed through the policy offline before closing the loop
- [ ] E-stop, joint/torque limits and a watchdog that holds position if the policy stalls

## 6. Real-World RL

Do RL directly on hardware when sim cannot capture the task (deformables, precise insertion,
cable routing) and success can be detected automatically. Reported budgets for context: SERL
learned PCB insertion, cable routing and object relocation in 25-50 minutes of training per
policy; HIL-SERL reached near-perfect success in 1-2.5 hours per task. Both used
demonstrations, pretrained vision encoders, careful workspace engineering and human
supervision; plan in hours of robot time, and move to sim if a task needs millions of steps.

### Safety Layers (outside the policy, fail closed)

| Layer | Implementation |
|---|---|
| Workspace bounds | Clip EE targets to a box (LeRobot `end_effector_bounds`; HIL-SERL bounds) |
| Step limits | Max EE translation/rotation per step (LeRobot `end_effector_step_sizes`) |
| Compliance | Impedance/admittance control with bounded stiffness; force/torque thresholds that abort |
| Joint and torque limits | Enforced in the driver, below the policy |
| Watchdog | Hold position if no action arrives within one control period plus margin |
| Human override | Intervention device always active; E-stop reachable |

### Resets

- **Scripted resets:** return to a fixed or randomised joint configuration (LeRobot
  `fixed_reset_joint_positions`, `reset_time_s`).
- **Human resets:** acceptable for hours-long runs; budget the human time.
- **Learned reset policy / forward-backward training:** a second policy undoes the task.
- **Reset-free:** design the task to be cyclic (pick, place, pick again).
- Randomise the reset pose slightly so the policy does not overfit one start.

### Reward From Classifiers

- Train a binary success classifier on images: HIL-SERL reports typically ~200 positive and
  ~1000 negative frames, with >95% accuracy on held-out data. LeRobot trains one with
  `lerobot-train` and applies it with `success_threshold`.
- The policy will search for false positives. Add frames from policy rollouts as hard
  negatives, and periodically audit classifier positives by eye.
- Zero-shot VLM reward models exist (LeRobot integrates TOPReward). Treat them as shaping or
  progress signals and validate against human labels before letting the policy optimise them.

### Human Interventions

- HIL-SERL stores intervention transitions in both the demonstration and RL buffers.
- A healthy run shows intervention rate falling over training (LeRobot's guide plots it);
  flat or rising intervention rate means the reward, action space or observation is wrong.
- Intervene to correct and to prevent damage, not to complete every episode; constant takeover
  turns RL into slow imitation.

## 7. Debugging: Symptom -> Cause -> Fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Reward flat from iteration 0 | Reward sign or scale wrong; observation bug; actions not applied | Scripted policy through the env; print each reward term; assert actions change state |
| Learns to stand still or fall immediately | Penalties dominate task term; negative per-step reward with failure termination | Rebalance weights; positive per-step reward or termination penalty |
| Value loss explodes or grows steadily | Unnormalised rewards/returns, time-outs treated as terminal, reward spikes | Bootstrap truncation; scale rewards to O(1) per step; clip outlier terms; lower LR |
| Policy std / entropy collapses early, then plateau | Entropy coefficient too low, LR too high, action scale too small | Raise `entropy_coef`; check KL; enlarge action scale |
| Policy std grows without bound | Entropy bonus too large or reward insensitive to action | Lower entropy coef; check that actions reach the actuators |
| KL spikes, adaptive LR pinned at its floor (1e-5) | Batch too small, too many epochs, non-stationary normaliser | Fewer epochs, more mini-batch data; freeze or slow normaliser updates |
| NaN in loss or actions | Division by zero in obs (normalised quaternion of zeros), sim blow-up, log-prob of saturated tanh, LR too high | `torch.autograd.set_detect_anomaly` briefly; assert finite obs/rewards in env; reset envs whose state is non-finite; clamp log-std |
| Actions pinned at bounds | Action scale too small, reward favours max effort, unbounded Gaussian clipped by env | Log clipped-action fraction; rescale; add effort penalty; consider tanh-squashed or Beta policy |
| Great in training, bad in eval | Eval uses stochastic actions or different DR; normaliser not loaded; overfit to seeds | Evaluate deterministic mean action on fixed seeds with frozen normaliser |
| Jittery gait, high-frequency actions | Weak action-rate penalty, policy rate too high, exploiting solver | Action-rate / acceleration penalty, CAPS, lower rate, filter in the loop |
| Works in sim, oscillates on robot | Latency, PD gain mismatch, filter missing in training | Measure latency and randomise it; match gains; include filter in sim |
| Works in sim, too weak on robot | Motor strength or friction optimistic in sim | SysID; randomise motor strength and joint friction |
| Huge seed-to-seed variance | Hard exploration, brittle reward | Curriculum, shaping, more envs; report IQM with CIs (section 8) |
| Off-policy Q-values diverge | High UTD without regularisation, bad terminal flags | LayerNorm critics, ensembles (RLPD), check `done` vs time-out handling |
| Real-world RL never improves | Classifier false positives, unsafe bounds too tight to solve task, too few demos | Audit classifier; widen bounds carefully; add demos; check intervention data is in the buffers |

**First 30 minutes on a new environment (heuristic):** random policy and scripted policy
rollouts with per-term reward logging; check observation ranges and that `terminated` and
`truncated` fire when expected; train 3 seeds for a short run; watch a video before tuning
anything.

## 8. Evaluation

- **Seeds:** at least 5 for sim claims (3 is the minimum for a quick check); vary only one
  thing between compared runs; equal compute or equal samples, stated explicitly.
- **Aggregate with IQM and bootstrap confidence intervals** (Agarwal et al., arXiv 2108.13264;
  library `rliable`, repository archived in 2024 but installable and correct). The mean is
  dragged by failed seeds; the median ignores too much.
- **Eval protocol:** deterministic action (mean), frozen normaliser, fixed set of initial
  conditions, both nominal and randomised dynamics; report success rate and task metrics
  (tracking error, time to success, energy), not only return.
- **Real robot:** fixed trial count per condition (20+ when possible), interleave conditions
  to avoid drift bias, report binomial confidence intervals.

```python
import math
import random


def iqm(scores):
    """Interquartile mean: mean of the middle 50% (rliable's recommended aggregate)."""
    s = sorted(scores)
    lo = int(0.25 * len(s))  # same cut as scipy.stats.trim_mean(scores, 0.25)
    mid = s[lo:len(s) - lo]
    return sum(mid) / len(mid)


def bootstrap_ci(scores, stat=iqm, reps=2000, alpha=0.05, seed=0):
    """Percentile bootstrap CI over seeds. Use rliable for stratified multi-task versions."""
    rng = random.Random(seed)
    boots = sorted(stat([rng.choice(scores) for _ in scores]) for _ in range(reps))
    return boots[int(alpha / 2 * reps)], boots[int((1 - alpha / 2) * reps) - 1]


def wilson_ci(successes, trials, z=1.96):
    """95% Wilson interval for a real-robot success rate (better than +-1.96*sqrt(p(1-p)/n))."""
    p = successes / trials
    denom = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return centre - half, centre + half


if __name__ == "__main__":
    per_seed = [0.92, 0.88, 0.95, 0.31, 0.90, 0.93, 0.89, 0.94, 0.12, 0.91]
    print("mean", round(sum(per_seed) / len(per_seed), 3), "IQM", round(iqm(per_seed), 3),
          "95% CI", tuple(round(c, 3) for c in bootstrap_ci(per_seed)))
    print("18/20 real trials:", tuple(round(c, 3) for c in wilson_ci(18, 20)))
```

Two failed seeds pull the mean to 0.775 while the IQM is 0.905; report both, and investigate
the failures rather than dropping them.

## 9. Anti-Patterns

- **Tuning hyperparameters before verifying the env.** Most "PPO does not learn" reports are
  env bugs. Scripted policy first.
- **One seed, one curve.** RL variance between seeds routinely exceeds the effect being claimed
  (Henderson et al., arXiv 1709.06560).
- **Summed reward only in the logs.** You cannot see hacking or dominance without per-term logs.
- **Privileged observations leaking into the actor.** Works in sim, impossible on hardware.
- **Randomising around guessed nominal values.** Identify first.
- **Deployment code rewritten from scratch.** Share observation building, action scaling and
  normalisation code between sim and robot, and test it with recorded pairs.
- **RL on hardware without an external safety layer.** The policy will explore the action that
  breaks the gripper.
- **Real-world RL for a task that needs millions of samples.** Move it to sim or use demos.

## 10. Checklist

**Problem framing**
- [ ] Wrote down why RL beats imitation or classical control for this task
- [ ] Success metric defined independently of the reward

**MDP**
- [ ] Observations are deployable (or explicitly privileged and critic/teacher-only)
- [ ] Observations scaled; normaliser frozen and exported with the policy
- [ ] Action space, scale, clipping and control rate chosen deliberately and documented
- [ ] `terminated` only for true terminal states; time limits are `truncated` and bootstrapped
- [ ] Scripted and random policies run through the env without errors or NaNs

**Reward**
- [ ] Every term logged separately; magnitudes checked at initialisation
- [ ] Reward sign vs termination checked (no incentive to die early or avoid success)
- [ ] Rollout videos reviewed at regular intervals; sanity metrics outside the reward tracked

**Training**
- [ ] Library and algorithm match the setting (parallel sim -> PPO; real robot -> off-policy with demos)
- [ ] Action std, KL, value loss, clipped-action fraction and per-term rewards logged
- [ ] At least 3 seeds before any conclusion; 5+ for reported results

**Sim-to-real**
- [ ] System identification done; randomisation centred on identified values
- [ ] Latency, observation noise, motor strength and friction randomised
- [ ] Reality-gap audit passed; logged real observations replayed offline
- [ ] Safety layer, E-stop and watchdog tested before the first closed-loop run

**Evaluation**
- [ ] Deterministic eval on fixed initial conditions, nominal and randomised
- [ ] IQM with bootstrap CIs across seeds; real-robot results with trial counts and intervals

## Related Skills

- `imitation-learning`: behaviour cloning, diffusion and VLA policies that RL can fine-tune
- `lerobot`: LeRobot datasets, robots and the HIL-SERL workflow end to end
- `robot-data-quality`: auditing demonstrations and logged real-robot data used in replay buffers
- `isaac-sim`: Isaac Sim / Isaac Lab environments, managers, wrappers and launch details
- `mujoco`: MuJoCo, MJX, MuJoCo Warp and Playground environments, `mujoco.sysid`
- `robotics-design-patterns`: safety layers, watchdogs and sim-to-real architecture
- `robotics-testing`: testing env code, deployment code and recorded-pair regression tests
- `robot-bringup`: hardware bringup, controllers and limits beneath a learned policy
