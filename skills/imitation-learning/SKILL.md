---
name: imitation-learning
description: >
  Practical imitation learning (learning from demonstration) for robot manipulation:
  deciding whether IL fits, designing demonstration collection, choosing observation and
  action spaces, picking and configuring a policy (behaviour cloning, ACT, Diffusion
  Policy, flow matching, VLAs such as OpenVLA-OFT, Octo, pi0/pi0.5, SmolVLA, GR00T),
  training and checkpoint selection, honest rollout evaluation with confidence intervals,
  deployment timing, and diagnosing failures. Use this skill when the user mentions
  behaviour cloning, teleoperation demos, leader-follower arms, action chunking, temporal
  ensembling, fine-tuning a VLA, "how many demos", success rate, policy evaluation,
  DAgger or intervention data, covariate shift, a policy that freezes, jitters, misses
  grasps, or works in training but fails on the robot, or sim and real co-training.
---

# Imitation Learning for Robot Manipulation

Model landscape checked 2026-11-18; verify versions before relying on them.

Imitation learning (IL) is supervised learning on a dataset you collect yourself and an
evaluation you have to run on hardware. Most failed IL projects fail on data and
evaluation, not architecture. This skill covers the decisions in the order you make
them. Library commands live in `lerobot`, dataset QA in `robot-data-quality`.

**Reference files (load on demand):**
- `references/policy-landscape.md`: policy families and VLAs with releases, licences,
  GPU needs, published default hyperparameters, and sources. Load it before naming a
  model version, licence or hyperparameter, or when choosing between pretrained VLAs.
- `references/deployment-runtime.md`: tested code for temporal ensembling and
  asynchronous chunk execution, latency budgeting, and the safety clamp. Load it when
  writing the on-robot inference loop.

## When to Use This Skill

- Planning a demonstration collection campaign or deciding how many demos to record
- Choosing between ACT, Diffusion Policy, a flow-matching policy, or fine-tuning a VLA
- Designing action spaces (joint vs end-effector, absolute vs delta) and chunk sizes
- Training runs where loss looks fine but the robot fails
- Designing an evaluation protocol or comparing two policies
- Writing the deployment loop for a chunked policy (latency, async execution, smoothing)
- Diagnosing failures: freezing, drift, gripper timing, oscillation, camera sensitivity
- Planning correction data (DAgger, interventions) or sim and real co-training

## Is IL the Right Tool?

| Situation | Prefer | Why |
|---|---|---|
| Object poses known or cheaply perceived, rigid parts, repeatable fixture | Scripted control + perception | Deterministic, verifiable, no data campaign |
| Contact-rich or deformable task a human can demo in < 1 min (folding, insertion with wiggle, cable routing) | IL | Hard to script; demos capture the strategy |
| Reward easy to compute in sim, dynamics simulate well (locomotion, in-hand reorientation) | RL in sim (`robot-reinforcement-learning`) | Search beats human demos when the sim is trustworthy |
| Human cannot teleoperate it well (too fast, too precise for the interface) | RL, or scripted + learned residual | IL ceiling is the demonstrator |
| IL policy at 60-80% and the failures are specific | IL plus correction data, then possibly RL fine-tuning | Targeted data is cheapest; RL needs a reset and reward story |

Hybrid is normal: a scripted approach move plus a learned contact phase, or a behaviour
tree calling learned skills (`robotics-design-patterns`).

**What a demo budget buys (heuristic, single arm, one task, from-scratch policy):**

| Demos | Realistic outcome |
|---|---|
| 10-20 | Pipeline check. Expect success only near demonstrated start states. |
| 50 | ACT-style precise tasks in a fixed scene (the ACT paper used 50 per task, 10-20 min of data). Fragile to new positions, lighting, distractors. |
| 100-300 | Usable within a randomised start-state region; still a single scene. Octo's fine-tuning setups used ~100. |
| 1000+ or many hours across scenes | Needed for new environments and objects. pi0 post-training used 5 h for simple tasks, 100+ h for complex ones. |

Diversity of environments and objects matters more than raw count once each condition
has enough demos (Lin et al. 2024, *Data Scaling Laws in Imitation Learning*). Spend
the budget on coverage of the conditions you will be evaluated in.

## Mental Model

1. **The policy can only be as good as the state coverage of the data.** Behaviour
   cloning trains on states the demonstrator visited. Small errors take the robot to
   states nobody demonstrated, where errors grow (covariate shift; Ross et al. 2011,
   DAgger). Everything else (chunking, recovery data, interventions) manages this.
2. **The loss is a surrogate; rollouts are the metric.** Low validation loss does not
   mean high success (robomimic, Mandlekar et al. 2021). Budget robot time for
   evaluation from day one.
3. **Train and deploy must see identical inputs.** Same cameras, mounts, resolution,
   crop, colour order, normalisation statistics, control rate and action convention.
   Most "the model does not work" reports are pipeline mismatches.
4. **Consistent demonstrators make learnable data.** One strategy executed with small
   variation beats several strategies mixed, unless the policy class models multimodality
   and you have the data to cover every mode.
5. **Iterate on failures, not on architectures.** Loop: collect, train, evaluate on a
   fixed set, classify failures, collect data that targets the top failure, repeat.

## Decision 1: Data Collection

### Teleoperation modality

| Modality | Good for | Weaknesses |
|---|---|---|
| Leader-follower (ALOHA, SO-101 pair) | Joint-space actions, bimanual, fast dexterous motion, cheap | Needs a kinematically matched leader; no force feedback (operator relies on vision) |
| VR / hand tracking | Mobile and bimanual EEF control, humanoids, natural motions | IK singularities, tracking jitter, latency; operator fatigue |
| SpaceMouse / gamepad | EEF delta control on single arms, simple pick and place | Slow; operators move one axis at a time, producing unnatural, hesitant data |
| Kinesthetic (hand-guiding the arm) | Arms with good gravity compensation; contact tasks | The human is in the camera view; the robot pose is driven by a hand, so recordings mix human and robot |
| Handheld grippers (UMI-style) | In-the-wild collection without a robot | Needs latency matching and relative actions; embodiment gap at deployment |

Pick the interface that makes demonstrations **fast and smooth**. Hesitation, pauses
and corrections in demos become hesitation in the policy.

### What to keep constant vs randomise

Keep constant (or the policy will need data covering every variation):
- Camera mounts and intrinsics. Mark positions; check a reference overlay before each session.
- Robot base pose relative to the table, gripper fingertips, control rate.
- Task definition and success criterion, written down before collection starts.
- Strategy: one grasp approach, one ordering of subtasks (unless multimodality is intended).

Randomise deliberately, within the range you will evaluate in:
- Object initial positions and orientations: sample from a defined region (draw it on the
  table or use a printed grid) instead of "wherever I put it".
- Robot start configuration within a small region, not one fixed pose.
- Distractors, lighting, table cloth, object instances, if generalisation to them matters.

Record start conditions per episode (object pose region cell, instance ID, lighting tag).
You need them later to build eval sets and to find coverage gaps.

### Failures, recoveries and idle frames

- **Recoveries are the most valuable demos you are not collecting.** Include episodes
  where the operator recovers from a slip, a missed grasp or a nudged object. Without
  them the policy has never seen how to get back to the demo distribution.
- **Do not train on failed demos as if they were successes.** Keep them in the dataset
  marked as failures (useful for analysis, reward models, advantage-conditioned methods),
  and filter them out of BC training.
- **Trim idle frames.** Seconds of stillness at the start or end of an episode teach the
  policy that "stand still" is a valid action given an initial-looking frame, which shows
  up as a policy that never starts. openpi added an idle filter for DROID training for
  this reason. Detection and trimming tools are in `robot-data-quality`.

### Cameras

- **Wrist camera plus at least one external camera** is the default for manipulation. The
  wrist view resolves fine alignment and grasp state; the external view gives global
  context and object location. ALOHA used two wrist cameras plus front and top views.
- Fix exposure and white balance. Auto-exposure changes the input distribution with
  every arm motion.
- Avoid views where the operator's body or the leader arm is visible; it will be absent
  at deployment.
- Record at the control rate you will deploy at, or above it and downsample consistently.

### Collection session checklist

- [ ] Task spec and success criterion written down; same for every operator
- [ ] Camera positions verified against reference images; exposure locked
- [ ] Initial-condition region marked; per-episode conditions logged
- [ ] Episode boundaries: start recording at first motion, stop at task completion
- [ ] Operator practised until demos are smooth; first warm-up demos discarded
- [ ] Recovery episodes planned (for example 10-20% of the budget)
- [ ] Dataset QA run after every session (`robot-data-quality`), not after the campaign

## Decision 2: Observation and Action Design

### Action space

| Action | Pros | Cons | Use when |
|---|---|---|---|
| Absolute joint positions | Matches leader-follower teleop exactly; no IK at runtime; ACT reported delta joints did worse | Embodiment-specific; does not transfer across robots | Leader-follower arms, ACT, single robot |
| Delta joint positions | Small numeric range | Errors integrate; drift | Rarely the best choice |
| Absolute EEF pose | Robot-agnostic | Needs accurate calibration of base frame; IK at runtime | Calibrated cells, cross-robot datasets |
| Delta EEF (per step) | Natural for SpaceMouse; robust to base offset | Accumulates error over the chunk | Low-rate VLA control on SpaceMouse-style data (OpenVLA was trained on single-arm EEF control data) |
| Relative trajectory (each action relative to EEF pose at chunk start) | Robust to calibration and camera shift; no per-step accumulation | Needs good state estimation of the current pose | Handheld or cross-embodiment data; UMI found it beat delta and absolute; GR00T N1.7 uses relative EEF |

Rules:
- Prefer **position targets over velocities**. Diffusion Policy found position control
  consistently beat velocity control and suffered less compounding error.
- Use the action convention your pretrained VLA was trained with, or expect to need more
  fine-tuning data.
- Represent rotations continuously (6D rotation or quaternion with sign fixed), not Euler.
- Treat the gripper as its own dimension. Decide between continuous position and a
  binary open/close with a threshold; record what the teleop interface actually sent.

### Action chunking

Predict a sequence of future actions instead of one. It reduces the effective horizon
over which errors compound and handles pauses and non-Markovian demos. Published
defaults (starting points, not tuned values):
- ACT: chunk 100 at 50 Hz (2 s). Success rose sharply from chunk 1 to 100.
- Diffusion Policy: predict 16, execute 8, observe 2 frames, at 10 Hz.
- pi0, SmolVLA: chunks of 50.

Heuristics: the chunk should cover roughly 0.5-2 s of motion; execute only part of it
before replanning if the task needs reactivity (moving objects, contact surprises).
Longer chunks are smoother but less reactive. Execution modes (synchronous, temporal
ensembling, asynchronous) are under Decision 6.

### History

Start with no history or two frames. Long observation histories invite the copycat
problem: the policy learns to repeat its previous action from proprioceptive history
instead of reading the scene (Wen et al. 2020). Add history only to fix a failure that
needs it (occluded state, velocity estimation) and verify the fix on rollouts.

### Proprioception vs images

- Include proprioception (joint positions, gripper state); it is cheap and informative.
- If the policy ignores images and replays a trajectory from proprio, you get a policy
  that reaches for where the object usually is. Symptom: works only at demonstrated
  positions. Remedies: more position diversity, proprio dropout or noise during training.

### Images

- Resolution is modest in practice: Diffusion Policy's real tasks used 320x240 per
  camera; ACT streamed 480x640; VLAs fix the size in their processor (check the model
  card). Higher resolution mostly costs latency; crop before you upscale.
- Random crop at train time, centre crop at test time is the usual recipe. Diffusion
  Policy cropped 320x240 to 288x216; OpenVLA's LIBERO setup used 90%-area random crops
  and its README warns to centre crop at evaluation.
- Crop away regions that change but do not matter (people walking by, monitors).

### Normalisation

- Normalise each state and action dimension using dataset statistics (mean/std or
  min/max to [-1, 1]; diffusion and flow heads usually expect [-1, 1]).
- Save the statistics **with the checkpoint** and load them in the deployment code. A
  stale or recomputed stats file is a classic cause of a policy that moves wildly.
- Guard against near-constant dimensions (std near 0); clamp the denominator.

## Decision 3: Policy Choice

Start with the simplest policy that can represent the demos, and keep it as a baseline.

```
Single task, one robot, no language, < ~300 demos?
├── Joint-space leader-follower data, precise motions ......... ACT
├── Demos genuinely multimodal (several valid strategies) ..... Diffusion Policy (CNN first)
└── State-based sim task / pipeline sanity check .............. BC MLP baseline, then the above
Multiple tasks, language instructions, or new objects/scenes?
├── Compute-limited (one consumer GPU) ........................ SmolVLA fine-tune
├── 24-80 GB GPU, single or bimanual arm ...................... pi0 / pi0.5 (openpi) or OpenVLA-OFT
└── Humanoid or NVIDIA stack (Isaac, Jetson Thor) ............. GR00T N1.7 fine-tune
```

- **Behaviour cloning MLP/CNN**: single-step regression. Fine for state-based tasks and
  for proving the pipeline works. Collapses on multimodal demos (averages modes).
- **ACT**: CVAE plus transformer predicting chunks of absolute joint targets with L1 loss.
  ~80M parameters, ~5 h training on one 11 GB GPU in the paper. Strong default for
  leader-follower data.
- **Diffusion Policy**: denoises action sequences; models multimodal action
  distributions; slower inference (use DDIM with ~10 steps). Start with the CNN variant.
- **Flow-matching heads**: the same niche as diffusion with fewer integration steps;
  used inside pi0, SmolVLA and GR00T. Recent evidence suggests the benefit of
  generative heads comes mostly from iterative refinement rather than multimodality
  (arXiv 2512.01809), so a well-built regression chunk policy is a fair baseline.
- **VLAs**: pretrained vision-language backbones plus an action head. Worth it for
  language conditioning, multi-task, and generalisation to new objects and scenes. Not
  automatically better on a narrow single task, and inference is heavier.

**Fine-tune vs from scratch:** fine-tune a pretrained policy when your embodiment or
action convention is close to its pretraining data, when you need language or
generalisation, or when you have few demos per task across many tasks. Train from
scratch when latency is tight, the task is narrow, or the licence does not fit. When in
doubt, run both on the same evaluation set; the comparison costs less than an argument.

Licences differ (MIT, Apache-2.0, Gemma terms, NVIDIA licences, some non-commercial) and
can disagree between a README and the checkpoint's LICENSE file. Check
`references/policy-landscape.md` and then the actual checkpoint repo before a product
decision.

## Decision 4: Training

### Recipe (heuristic starting point)

1. Hold out **episodes**, not frames, for validation (frames from one episode are
   near-duplicates). Hold out a few initial-condition cells too if you care about
   generalisation.
2. Use the published or library defaults for the policy first (values in
   `references/policy-landscape.md`). Change one thing at a time.
3. Augment images: random crop/shift, colour jitter (brightness, contrast, saturation,
   small hue). Avoid flips (they swap left and right in action space) and large
   rotations unless you transform actions to match.
4. Train long enough that training loss has plateaued; IL policies often keep improving
   in rollouts well after validation loss has bottomed out.
5. Save checkpoints at regular intervals (for example every 10% of training) so you can
   select by rollouts.

Fine-tuning VLAs: start with the provider's recipe (openpi configs, GR00T's "few
thousand steps" guidance, LeRobot defaults). LoRA if memory-bound; full fine-tuning
when you have the GPU and enough data. Keep the action normalisation and camera keys
exactly as the fine-tuning config expects.

### Why validation loss is a weak proxy

- The loss measures one-step (or one-chunk) prediction on demonstrator states. Rollouts
  depend on behaviour in states the policy itself causes.
- Multimodal demos make the loss noisy and irreducible; a lower loss can mean more
  mode averaging.
- robomimic evaluated every checkpoint by rollout because the lowest-validation-loss
  checkpoint was often far from the best; SIMPLER (Li et al. 2024) found validation MSE
  correlates poorly with real-world success.

Use validation loss to catch bugs (divergence, a loss that never drops, a large
train/val gap suggesting leakage or memorisation), not to pick the deployed checkpoint.

### Checkpoint selection by rollouts

- Shortlist 3-5 checkpoints (for example late-training ones plus the val-loss minimum).
- Screen them on a small fixed set (10-20 trials each, same initial conditions) in sim
  or on the robot to discard clearly bad ones.
- Run the full evaluation protocol only on the 1-2 survivors. Report which checkpoint
  and why; selecting the best of many on the same trials inflates the reported rate, so
  confirm on fresh trials.

### Overfitting signs

- Success near demonstrated start states, failure a few centimetres away.
- The robot reaches to where the object was in most demos rather than where it is.
- Training loss near zero, validation loss rising early, rollouts unchanged or worse.
- Fixes: more start-state diversity, augmentation, smaller model or fewer epochs, proprio
  noise or dropout. More of the same demos rarely helps.

## Decision 5: Evaluation Protocol

A success rate is an estimate with error bars. Treat evaluation like an experiment.

### Protocol

1. **Define success before running** (for example "block fully inside bowl, gripper
   open, within 60 s"). Add partial-credit stages (reached, grasped, lifted, placed) to
   see where failures happen; report them separately from binary success.
2. **Fix an initial-condition set.** For example 25 marked object poses times 2 robot
   start poses. Reuse it for every policy you compare; photograph or overlay each setup.
3. **Split conditions:** in-distribution (within the demonstrated region), held-out
   positions (outside or between demonstrated cells), held-out objects or scenes if
   generalisation is claimed. Report each split; never pool them.
4. **Interleave and blind.** Alternate policies per condition in randomised order so
   drift (lighting, battery, wear, operator fatigue) is not confounded with the policy.
   If a human judges success or resets the scene, they should not know which policy ran
   (TRI's LBM study used blind, randomised A/B rollouts with matched initial conditions).
5. **Enough trials.** 10 trials do not distinguish most policy pairs (see below). Use
   50+ per policy per condition split for claims that one policy is better.
6. **Log everything:** checkpoint hash, dataset version, condition ID, outcome, stage
   reached, failure category, video. Failures are your next data collection plan.

### Statistics (tested, standard library only)

```python
"""Success-rate statistics for robot policy evaluation. Python 3.8+, standard library only."""
from math import ceil, comb, sqrt
from statistics import NormalDist


def wilson_interval(successes: int, trials: int, confidence: float = 0.95) -> tuple:
    """Wilson score interval for a success rate. Behaves at 0/n and n/n, unlike p +/- z*se."""
    if trials <= 0 or not 0 <= successes <= trials:
        raise ValueError("need trials > 0 and 0 <= successes <= trials")
    z = NormalDist().inv_cdf(0.5 + confidence / 2)
    p = successes / trials
    denom = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def fisher_exact_p(s_a: int, n_a: int, s_b: int, n_b: int) -> float:
    """Two-sided Fisher exact test for two independent sets of trials."""
    total_s, total_n = s_a + s_b, n_a + n_b

    def pmf(k: int) -> float:  # P(policy A gets k of the successes) if both are equal
        return comb(n_a, k) * comb(n_b, total_s - k) / comb(total_n, total_s)

    observed = pmf(s_a)
    ks = range(max(0, total_s - n_b), min(n_a, total_s) + 1)
    return min(1.0, sum(pmf(k) for k in ks if pmf(k) <= observed * (1 + 1e-9)))


def mcnemar_exact_p(a_only: int, b_only: int) -> float:
    """Paired comparison on the SAME initial conditions: a_only = conditions where only A
    succeeded, b_only = only B succeeded. Ties (both/neither) carry no information."""
    n = a_only + b_only
    if n == 0:
        return 1.0
    tail = sum(comb(n, k) for k in range(min(a_only, b_only) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def trials_per_policy(p_a: float, p_b: float, alpha: float = 0.05, power: float = 0.8) -> int:
    """Approximate trials per policy needed to tell p_a from p_b (two-sided, normal approx)."""
    z_a, z_b = NormalDist().inv_cdf(1 - alpha / 2), NormalDist().inv_cdf(power)
    p_bar = (p_a + p_b) / 2
    root = z_a * sqrt(2 * p_bar * (1 - p_bar)) + z_b * sqrt(p_a * (1 - p_a) + p_b * (1 - p_b))
    return ceil(root ** 2 / (p_a - p_b) ** 2)


if __name__ == "__main__":
    for s, n in [(8, 10), (10, 10), (40, 50)]:
        lo, hi = wilson_interval(s, n)
        print(f"{s}/{n} = {s / n:.0%}, 95% CI [{lo:.0%}, {hi:.0%}]")
    print(f"8/10 vs 5/10:   p = {fisher_exact_p(8, 10, 5, 10):.2f}")
    print(f"40/50 vs 25/50: p = {fisher_exact_p(40, 50, 25, 50):.4f}")
    print(f"paired, A-only 9 vs B-only 2: p = {mcnemar_exact_p(9, 2):.3f}")
    print("trials/policy, 50% vs 80%:", trials_per_policy(0.5, 0.8))
    print("trials/policy, 70% vs 80%:", trials_per_policy(0.7, 0.8))
```

Output (cross-checked against scipy and statsmodels):

```
8/10 = 80%, 95% CI [49%, 94%]
10/10 = 100%, 95% CI [72%, 100%]
40/50 = 80%, 95% CI [67%, 89%]
8/10 vs 5/10:   p = 0.35
40/50 vs 25/50: p = 0.0031
paired, A-only 9 vs B-only 2: p = 0.065
trials/policy, 50% vs 80%: 39
trials/policy, 70% vs 80%: 294
```

What this means in practice:
- **8/10 is consistent with anything from about 49% to 94%.** 10/10 still allows 72%.
- **8/10 vs 5/10 is not evidence that A is better** (p = 0.35). The same rates over 50
  trials each are.
- Telling 50% from 80% needs ~40 trials per policy; 70% from 80% needs ~300. Small
  improvements on a good policy are expensive to prove; say so instead of claiming them.
- When both policies ran on the same initial conditions, the paired McNemar test uses
  only the conditions where they disagree and is usually more sensitive than Fisher.
- Comparing many policies or checkpoints at once inflates false positives; correct for
  it (Bonferroni is simple and conservative) or confirm the winner on fresh trials.

### Reporting template

```
Policy: act_pick_cube @ step 80k (ckpt sha 3f2a...), dataset v4 (212 episodes)
Split            Trials  Success  Rate   95% Wilson CI   Mean stage reached
in-distribution  50      41       82%    [69%, 90%]      3.7 / 4
held-out poses   30      14       47%    [30%, 64%]      2.9 / 4
Failure categories (in-dist): missed grasp 5, dropped in transit 3, timeout 1
Conditions: 25 poses x 2 start configs, interleaved with baseline, evaluator blind
```

## Decision 6: Deployment

- **Run at the rate you trained on.** A chunk index means a time offset only at the
  demonstrated control rate. If the policy outputs at 10 Hz and the arm needs 100+ Hz,
  interpolate between targets in the low-level controller (Diffusion Policy interpolated
  10 Hz commands to 125 Hz).
- **Measure latency** (camera, inference p99, command-to-motion). The policy acts on
  observations that are that old. `references/deployment-runtime.md` has a procedure.
- **Pick an execution mode:**
  - *Synchronous:* infer, execute `n` actions of the chunk, repeat. Simplest; pauses
    during inference unless inference is fast relative to `n` steps.
  - *Temporal ensembling (ACT):* infer every step, average overlapping predictions.
    Smooth, but one inference per step and less reactive. Do not average across modes.
  - *Asynchronous:* infer the next chunk while executing the current one, drop the
    actions that went stale during inference. Needed for large VLAs at high rates.
    SmolVLA ships async inference; real-time chunking (arXiv 2506.07339) makes
    flow/diffusion chunks join smoothly.
- **Smoothing:** prefer chunking and ensembling over low-pass filtering the output; a
  filter adds lag that the policy never saw in training. If you must filter, filter
  lightly and evaluate with the filter on.
- **Safety limits outside the policy:** joint and workspace limits, per-step delta limit,
  gripper force limit, watchdog, NaN check, physical e-stop. Clamp, log, and count every
  clamp; frequent clamping means the policy is out of distribution.
- **Warm up before the episode:** run several inference calls before starting the
  control loop (first calls include CUDA initialisation and compilation). Begin from a
  start pose inside the demonstrated region.
- **Open-loop replay test before any closed-loop run:** feed recorded observations from
  a training episode through the deployment pipeline and compare predicted actions with
  the recorded actions. A large mismatch is a pipeline bug (normalisation, camera order,
  colour order, units, crop), not a model problem.

## Failure Modes

| Symptom | Likely cause | Fix |
|---|---|---|
| Good first seconds, then drifts and never recovers | Compounding error / covariate shift | Action chunking; recovery demos; DAgger-style correction data; wider start-state randomisation |
| Robot does not move at episode start | Idle frames at the start of demos; or obs normalisation off so actions are ~0 | Trim idle frames (`robot-data-quality`); open-loop replay test; verify stats file |
| Hesitates or freezes mid-task at a decision point | Observation does not disambiguate task phase; mode averaging between strategies | Wrist camera; short history; chunking; multimodal head; standardise strategy in demos |
| Moves between two valid paths, hits the obstacle | Mode averaging by a regression (L1/L2) head | Diffusion/flow/ACT CVAE; or collect one consistent strategy |
| Excellent offline loss, poor closed loop; tracks its own previous motion | Causal confusion / copycat on proprio history (de Haan et al. 2019; Wen et al. 2020) | Remove or shorten history; proprio noise/dropout; interventions targeted at the confusing states |
| Gripper closes early/late, or chatters | Few transition frames; latency; continuous output hovering near threshold | Wrist camera; binary gripper with hysteresis; consistent grasp timing in demos; latency compensation |
| Works only at demonstrated object positions | Memorisation; images underused vs proprio | More position diversity; crop/shift augmentation; held-out-position evaluation |
| Fails in different lighting, background, or with people nearby | Overfitting to appearance | Vary conditions during collection; colour jitter; crop distractors; sim co-training |
| Success drops after someone bumps a camera | Camera pose shift | Rigid mounts; pre-session overlay check; shift augmentation; rely more on wrist camera; relative actions |
| Oscillation around the target, worse with bigger models | Latency: acting on stale observations, often with per-step delta actions | Measure latency; async execution with stale-action dropping; absolute or relative-chunk targets; smaller or faster model |
| Jerk at every chunk boundary | Successive chunks disagree; synchronous pauses | Temporal ensembling, blending, real-time chunking; overlap execution with inference |
| Wild, fast motions on first deployment | Wrong or missing normalisation stats; units (deg vs rad); joint order | Load stats from checkpoint; open-loop replay test; safety clamp |
| Sim success high, real success zero | Visual or dynamics gap; mismatched cameras or control rate | Match camera placement and rate; co-train with real data; domain randomisation (`isaac-sim`, `mujoco`) |

## Correction Data and Other Remedies

- **DAgger-style correction data.** Classic DAgger (Ross et al. 2011) labels states the
  policy visits with expert actions. On real robots, **HG-DAgger-style interventions**
  (Kelly et al. 2019) are practical: run the policy, the operator takes over when it is
  about to fail, and the takeover segment is recorded and added to training. Log the
  takeover flag; train on intervention segments (and optionally the successful policy
  segments), not on the failure leading into them. Interventions concentrate data where
  the policy is weakest, which is usually worth several times more than fresh demos.
- **Targeted recollection.** Use the failure categories from evaluation. If 60% of
  failures are missed grasps on rotated objects, collect rotated-object grasps.
- **Augmentation.** Image crop/shift and colour jitter are nearly always on; add
  proprio noise to fight copycat behaviour.
- **Learning from experience / RL fine-tuning.** Advantage-conditioned training on
  autonomous rollouts plus corrections (pi*0.6 / Recap, arXiv 2511.14759) and RL
  fine-tuning of IL policies belong to `robot-reinforcement-learning`.

## Sim Data and Co-Training

- **Co-training:** mix sim and real demonstrations in one training set, usually with a
  fixed sampling ratio per batch. Maddukuri et al. 2025 report an average 38% real-world
  improvement from sim data even with notable sim-real differences. Match cameras,
  action conventions and control rates between sim and real as closely as possible;
  that alignment matters more than photorealism.
- **Sim-generated demonstrations:** systems like MimicGen (arXiv 2310.17596) and Isaac
  Lab Mimic generate many demos from a few human demos by adapting object-centric
  segments to new scene layouts. Useful for coverage of initial conditions; still
  evaluate on real hardware.
- **Sim evaluation** is good for regression testing and checkpoint screening, not a
  substitute for real trials unless you have shown sim and real success correlate for
  your task.
- Simulator setup, assets and domain randomisation: `isaac-sim`, `mujoco`.

## Anti-Patterns

- **Picking the architecture before looking at the data.** Watch 20 random episodes
  first; inconsistent demos defeat every architecture.
- **Claiming improvement from 10 trials.** Report Wilson intervals; run a test.
- **Evaluating on the training start positions only.** Always include held-out poses.
- **Changing camera mounts, rates or the gripper between collection and evaluation.**
- **Selecting the checkpoint by validation loss.**
- **Mixing strategies in demos with a regression policy.** It will average them.
- **Filtering policy outputs heavily to hide jitter.** Find the cause (latency, chunk
  boundaries, normalisation) instead.
- **No intervention or recovery data after the first evaluation.** The failures you
  observed are the data you need.

## Checklist

Before collecting:
- [ ] IL is justified over scripting or RL; success criterion written
- [ ] Teleop interface allows smooth, fast demos; operators practised
- [ ] Cameras fixed (wrist + external), exposure locked, reference images saved
- [ ] Initial-condition region defined and per-episode conditions logged

Before training:
- [ ] Dataset QA passed (`robot-data-quality`): sync, drops, idle frames, failed episodes
- [ ] Action convention, control rate and normalisation decided and documented
- [ ] Episode-level train/val split; augmentation on; checkpoints saved periodically

Before evaluating:
- [ ] Open-loop replay test passes on the deployment pipeline
- [ ] Fixed initial-condition set with in-distribution and held-out splits
- [ ] Interleaved, ideally blind trials; 50+ per policy per split for comparisons
- [ ] Wilson intervals and a significance test in the report

Before deploying:
- [ ] Latency measured (p99); execution mode chosen; warm-up implemented
- [ ] Safety clamps, watchdog and e-stop outside the policy; clamp counter logged
- [ ] Licence of every pretrained component checked for the intended use

## Related Skills

- `lerobot`: recording, training and deploying policies with LeRobot (commands, configs)
- `robot-data-quality`: dataset QA, idle-frame trimming, sync and format checks with forge
- `robot-reinforcement-learning`: RL fine-tuning of IL policies, reward design
- `mujoco`, `isaac-sim`: simulated demos, co-training data, sim evaluation, Isaac Lab Mimic
- `robot-perception`: camera calibration and synchronisation
- `robotics-design-patterns`: safety systems, watchdogs, behaviour trees around learned skills
- `robotics-testing`: regression tests for policies in simulation and CI
