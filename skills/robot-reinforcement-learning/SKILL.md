---
name: robot-reinforcement-learning
description: >
  Reinforcement learning for robots: when RL beats imitation or classical control, MDP
  design (observations, action spaces, control rate, terminated vs truncated), reward design
  and reward hacking, algorithm choice, massively parallel sim training, safe real-robot RL,
  and debugging training curves. Use when the user mentions reinforcement learning, RL, PPO,
  SAC, TD3, GAE, reward function, reward shaping, curriculum, domain randomization,
  sim-to-real, actuator network, asymmetric actor-critic, teacher-student distillation,
  rsl_rl, rl_games, skrl, Stable-Baselines3, CleanRL, Brax training, FastTD3, HIL-SERL,
  SERL, RLPD, DrQ, residual RL, DSRL, RL fine-tuning of a BC, diffusion or VLA policy,
  time-out bootstrapping, entropy collapse, value loss explosion, NaN during training, or
  evaluating RL with seeds and IQM. Also for Isaac Lab, mjlab or Playground locomotion and
  manipulation policies.
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
| [references/mdp-design.md](references/mdp-design.md) | Full observation-group and action-space tables |
| [references/code-examples.md](references/code-examples.md) | Env skeleton, GAE, obs normaliser, IQM / CI code |
| [references/reward-design.md](references/reward-design.md) | Isaac Lab reward template; suspected reward hacking |
| [references/hybrid-methods.md](references/hybrid-methods.md) | RL with demos, a base controller, or a BC / diffusion / VLA policy |
| [references/algorithm-mechanics.md](references/algorithm-mechanics.md) | PPO internals, off-policy methods, asymmetric critics, distillation |
| [references/hyperparameters.md](references/hyperparameters.md) | Setting or tuning PPO / SAC / distillation hyperparameters; comparing to shipped Isaac Lab and SB3 values |
| [references/library-comparison.md](references/library-comparison.md) | Choosing or switching RL libraries; interop bugs (obs groups, time-out flags, autoreset, normaliser export) |
| [references/sim-to-real-randomization.md](references/sim-to-real-randomization.md) | Writing domain randomisation, latency buffers, system identification, or a pre-hardware reality-gap audit |
| [references/real-world-rl.md](references/real-world-rl.md) | RL on hardware: safety layers, resets, success classifiers, interventions |
| [references/debugging.md](references/debugging.md) | A training symptom is not in the short table in section 7 |

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

Hybrid approaches (RLPD, residual RL, DSRL, RL for VLAs): [references/hybrid-methods.md](references/hybrid-methods.md).

## 2. MDP Design

### Observations

Groups (proprioception, task / command, exteroception, history, privileged for critic /
teacher only), contents and rules: [references/mdp-design.md](references/mdp-design.md).

- Scale every input to roughly unit range (hand-chosen scales or a running normaliser).
  Raw joint velocities of 20 rad/s next to quaternions of magnitude 1 slow learning.
- Base linear velocity is easy in sim and hard on hardware. Isaac Lab's velocity task observes
  it (with +-0.1 noise); a deployed policy needs a state estimator that matches, or drop it.
- If the time limit is part of the task (must finish by T), add remaining time to the
  observation (Pardo et al., arXiv 1712.00378). If it is only a training convenience, leave it
  out and bootstrap at truncation.

### Action Spaces

Options (joint position target, the default for legged and humanoid; joint delta; torque;
EE delta via IK / impedance / OSC; discrete gripper), pros and cons:
[references/mdp-design.md](references/mdp-design.md).

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

Runnable stdlib env skeleton, GAE with correct terminated / truncated handling and a running
observation normaliser: [references/code-examples.md](references/code-examples.md). **Always run a
scripted or random policy through a new env before training**; it catches sign errors,
unreachable goals and broken resets in seconds instead of hours.

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

### Reward Hacking: Detection

Hack examples and the Isaac Lab reward template: [references/reward-design.md](references/reward-design.md).

Detection habits:

- Log every reward term separately per episode (Isaac Lab logs `Episode_Reward/<term>`).
- Record a rollout video every N iterations and actually watch it.
- Track physical sanity metrics **not in the reward**: success from an independent check,
  max contact force, joint power, foot slip, action saturation fraction.
- If the reward curve rises while the success rate is flat, assume hacking until disproven.

## 4. Algorithm and Library Choice

- **Massively parallel sim:** on-policy PPO with thousands of GPU environments is the default
  for locomotion and much of dexterous manipulation. Log the mean action std every iteration;
  it is the best single health signal.
- **Real robots and expensive sims:** off-policy (SAC / TD3; DrQ for pixels; RLPD when you have
  demos; SERL / HIL-SERL for real-robot manipulation).
- **Structural tricks:** asymmetric actor-critic, teacher-student distillation, RMA, symmetry
  augmentation, history stacks.

Mechanics and where they bite: [references/algorithm-mechanics.md](references/algorithm-mechanics.md).

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

## 5. Sim-to-Real

### Domain Randomisation: What to Randomise

Randomise what you cannot measure precisely, centred on what you can: dynamics, timing
(action latency and observation delay, often the largest unmodelled gap), sensing and
disturbances. Identify first, randomise second. A low-pass or exponential filter on actions
must be **in the training loop** if it is on the robot.

Too-wide randomisation produces timid, slow policies; too-narrow produces policies that fail on
contact with hardware. Widen until training slows, then stop (heuristic). Full list, legged
ranges, latency buffer, system ID, actuator networks, filtering:
[references/sim-to-real-randomization.md](references/sim-to-real-randomization.md).

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

Safety layers live **outside the policy and fail closed**: workspace bounds, step limits,
compliance, joint and torque limits, watchdog, human override. Table, resets, classifier
rewards: [references/real-world-rl.md](references/real-world-rl.md).

## 7. Debugging: Symptom -> Cause -> Fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Reward flat from iteration 0 | Reward sign or scale wrong; observation bug; actions not applied | Scripted policy through the env; print each reward term; assert actions change state |
| Learns to stand still or fall immediately | Penalties dominate task term; negative per-step reward with failure termination | Rebalance weights; positive per-step reward or termination penalty |
| Value loss explodes or grows steadily | Unnormalised rewards/returns, time-outs treated as terminal, reward spikes | Bootstrap truncation; scale rewards to O(1) per step; clip outlier terms; lower LR |
| Policy std / entropy collapses early, then plateau | Entropy coefficient too low, LR too high, action scale too small | Raise `entropy_coef`; check KL; enlarge action scale |
| NaN in loss or actions | Division by zero in obs (normalised quaternion of zeros), sim blow-up, log-prob of saturated tanh, LR too high | `torch.autograd.set_detect_anomaly` briefly; assert finite obs/rewards in env; reset envs whose state is non-finite; clamp log-std |
| Great in training, bad in eval | Eval uses stochastic actions or different DR; normaliser not loaded; overfit to seeds | Evaluate deterministic mean action on fixed seeds with frozen normaliser |
| Works in sim, oscillates on robot | Latency, PD gain mismatch, filter missing in training | Measure latency and randomise it; match gains; include filter in sim |
| Huge seed-to-seed variance | Hard exploration, brittle reward | Curriculum, shaping, more envs; report IQM with CIs (section 8) |

Full table: [references/debugging.md](references/debugging.md).

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

IQM, bootstrap CI and Wilson interval code:
[references/code-examples.md](references/code-examples.md). Investigate failed seeds rather
than dropping them.

## 9. Anti-Patterns

- **Tuning hyperparameters before verifying the env.**
- **One seed, one curve.**
- **Summed reward only in the logs.**
- **Privileged observations leaking into the actor.**
- **Randomising around guessed nominal values.**
- **Deployment code rewritten from scratch.**
- **RL on hardware without an external safety layer.**
- **Real-world RL for a task that needs millions of samples.**

Full text: [references/debugging.md](references/debugging.md).

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
