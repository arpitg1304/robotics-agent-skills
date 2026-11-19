# Debugging: Full Symptom -> Cause -> Fix Table

Moved from SKILL.md section 7, which keeps the most common rows.

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

## Anti-Patterns (full text)

SKILL.md section 9 keeps the one-line headlines.

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
