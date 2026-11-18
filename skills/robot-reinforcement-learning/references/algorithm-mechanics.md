# Algorithm Mechanics

Moved from SKILL.md section 4. Hyperparameter tables are in
[hyperparameters.md](hyperparameters.md); library details in
[library-comparison.md](library-comparison.md).

## PPO for Massively Parallel Sim

On-policy PPO with thousands of GPU environments is the default for locomotion and much of
dexterous manipulation (Rudin et al., arXiv 2109.11978, trained ANYmal in minutes this way).
Key mechanics and where they bite:

- **GAE** (gamma, lambda) as in [code-examples.md](code-examples.md); time-out bootstrapping is essential because rollouts are
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

Full tables: [hyperparameters.md](hyperparameters.md).

## Off-Policy for Real Robots and Expensive Sims

| Method | Use when | Key ingredients |
|---|---|---|
| SAC / TD3 | Few environments, expensive samples | Replay buffer, target networks, auto-tuned entropy (SAC) |
| DrQ / DrQ-v2 (2004.13649, 2107.09645) | Pixel observations | Random-shift image augmentation on critic and actor inputs |
| RLPD (2302.02948) | You have demos or prior data | 50/50 demo/online batches, LayerNorm critics, 10-critic ensemble, UTD 20 in its example commands |
| SERL / HIL-SERL | Real-robot manipulation | RLPD-style SAC, pretrained ResNet-10 encoder, binary success classifier, EE impedance control, human interventions |
| FastTD3 (2505.22642) / FastSAC (2512.01996) | Massively parallel sim where wall-clock matters | Large batches, many envs, tuned off-policy; reported humanoid locomotion in ~15 min on one RTX 4090 (FastSAC paper) |

## Structural Tricks

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
