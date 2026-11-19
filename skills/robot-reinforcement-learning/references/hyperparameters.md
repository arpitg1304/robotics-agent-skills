# Hyperparameter Reference

Checked 2026-11-18. "Shipped value" columns are read from source, not recalled: the Isaac Lab
ANYmal-C rough-terrain rsl_rl config
(`isaaclab_tasks/contrib/velocity/config/anymal_c/agents/rsl_rl_ppo_cfg.py`) and the
Stable-Baselines3 v2.9.2 constructor defaults. Everything in a "Heuristic" column is
practitioner experience, not a guarantee. Change one thing at a time and run at least 3 seeds
before concluding anything.

## PPO: Massively Parallel GPU Sim vs Classic CPU

| Parameter | Isaac Lab ANYmal-C (rsl_rl) | SB3 PPO default | What it controls | Heuristic |
|---|---|---|---|---|
| Parallel envs | 4096 (legged_gym default) | 1 (you choose) | Data per iteration | Thousands for GPU sim; throughput beats sample efficiency here |
| Steps per env per rollout | `num_steps_per_env=24` | `n_steps=2048` | Rollout length; horizon of GAE before bootstrap | 16-64 with thousands of envs. Very short rollouts lean heavily on the critic and on correct time-out bootstrapping |
| Samples per update | 4096 x 24 = 98,304 | 2048 | Batch size | - |
| Epochs | `num_learning_epochs=5` | `n_epochs=10` | Reuse of each batch | More epochs = more off-policy drift; watch KL and clip fraction |
| Mini-batches | `num_mini_batches=4` | `batch_size=64` (samples) | Gradient noise | rsl_rl splits the batch into N mini-batches; SB3 specifies size |
| Learning rate | `1e-3`, `schedule="adaptive"` | `3e-4` fixed | Step size | Adaptive: rsl_rl divides LR by 1.5 when KL > 2 x `desired_kl` and multiplies by 1.5 when KL < `desired_kl` / 2, bounded to [1e-5, 1e-2] |
| Target KL | `desired_kl=0.01` | `target_kl=None` | Trust region (rsl_rl: drives LR; SB3: early-stops epochs) | 0.005-0.02 |
| Clip range | `clip_param=0.2` | `clip_range=0.2` | Policy ratio clip | Rarely worth tuning first |
| Value clipping | `use_clipped_value_loss=True` | `clip_range_vf=None` | Limits critic step | Andrychowicz et al. 2020 (arXiv 2006.05990) found value-loss clipping did not help and could hurt; rsl_rl configs keep it on |
| Value loss coef | `1.0` | `vf_coef=0.5` | Critic vs actor gradient weight | Matters only with shared networks |
| Entropy coef | `0.005` | `ent_coef=0.0` | Exploration pressure | 0-0.01 for continuous control; raise if std collapses early, lower if std grows |
| Initial action std | `init_std=1.0` | `log_std_init=0.0` (std 1) | Initial exploration in normalised action units | Lower (0.3-0.5) when action_scale is large or the robot is fragile |
| gamma | `0.99` | `0.99` | Effective horizon ~1/(1-gamma) steps | 0.99 at 50 Hz = ~2 s horizon. Raise (0.995-0.998) for long tasks or high control rates |
| GAE lambda | `lam=0.95` | `gae_lambda=0.95` | Bias/variance of advantages | 0.9-0.97 |
| Grad norm clip | `max_grad_norm=1.0` | `0.5` | Stability | Keep on |
| Network | actor/critic MLP `[512, 256, 128]`, ELU (flat: `[128,128,128]`) | `[64, 64]` tanh | Capacity | Robotics PPO commonly uses 256-512 wide ELU MLPs; separate actor and critic |
| Obs normalisation | `obs_normalization=False` in this config | via `VecNormalize` | Input scaling | This config hand-scales observations instead; turn it on for new tasks with unscaled inputs |
| Advantage normalisation | per batch (option for per mini-batch) | `normalize_advantage=True` | Gradient scale | Keep on |

Episode: 20 s at 50 Hz (sim `dt=0.005`, `decimation=4`) = 1000 policy steps. Iterations:
1500 for rough terrain, 300 for flat.

### Tuning order (heuristic)

1. Make the env correct (rewards, resets, time-outs, observation scales) and confirm a
   hand-written controller earns sensible reward.
2. Set `num_envs` to saturate the GPU, then `num_steps_per_env` so a rollout spans a meaningful
   fraction of the behaviour you want (or rely on bootstrapping).
3. Adaptive LR with `desired_kl` around 0.01; adjust only if KL constantly hits the limits.
4. Entropy coefficient and initial std.
5. gamma for the task horizon.
6. Network size last.

## Off-Policy: SAC / TD3 Family

| Parameter | SB3 SAC default | Real-robot / RLPD-style practice | Notes |
|---|---|---|---|
| Learning rate | `3e-4` | `3e-4` | - |
| Batch size | `256` | 256 | RLPD samples 50% online, 50% offline (demo) data per batch ("symmetric sampling") |
| Buffer size | `1e6` | Fits all real data you will ever collect | Real-robot buffers rarely exceed ~1e5-1e6 transitions |
| `learning_starts` | `100` | Start once a few episodes (or demos) are in the buffer | - |
| Gradient steps per env step (UTD) | `gradient_steps=1` | RLPD example commands use `--utd_ratio=20`; its config sets `num_qs=10`, `num_min_qs=2`, `critic_layer_norm=True` | High UTD without critic regularisation overfits and overestimates; LayerNorm in the critic is the key fix RLPD reports |
| tau | `0.005` | 0.005 | Target network Polyak rate |
| gamma | `0.99` | 0.95-0.99 for short manipulation episodes | - |
| Entropy | `ent_coef="auto"`, `target_entropy="auto"` (= -dim(A)) | auto | If alpha decays to ~0 early, exploration has died |
| Timeouts | `handle_timeout_termination=True` (reads `info["TimeLimit.truncated"]`) | Must be correct | - |
| Image encoders | - | Pretrained ResNet-10 or similar + random-shift augmentation (DrQ) | SERL/HIL-SERL use pretrained visual backbones and image crops |

FastTD3 (arXiv 2505.22642) and FastSAC (arXiv 2512.01996) report that off-policy methods can
work with thousands of parallel envs when combined with large batches, a distributional critic
(FastTD3) and tuned hyperparameters, training humanoid locomotion on a single GPU in minutes to
hours. Start from their published configs rather than SB3 defaults.

## Distillation (Teacher-Student)

| Choice | Typical | Notes |
|---|---|---|
| Teacher | PPO policy with privileged obs (terrain heights, friction, contact states, true velocities) | Train first, freeze |
| Student input | Proprio history (e.g. 10-50 steps) or exteroception the robot really has | Use a GRU/LSTM/TCN or stacked history |
| Loss | MSE on actions (or on teacher latent, RMA-style) | rsl_rl ships a `Distillation` algorithm |
| Data | DAgger-style: student acts, teacher labels | Pure behaviour cloning of teacher rollouts suffers covariate shift |
| Fine-tune | Optional PPO fine-tune of the student | Watch for forgetting |
