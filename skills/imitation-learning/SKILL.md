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
- `references/policy-landscape.md`: policy families, VLAs, licences, GPU needs, published
  hyperparameters. Load before naming a model version, licence or hyperparameter, or
  when choosing a policy family or VLA.
- `references/data-collection.md`: demo budget, teleop modalities, camera setup,
  per-session checklist. Load when planning collection or answering "how many demos".
- `references/observation-action.md`: action-space table, proprio pitfalls, image size
  and crop. Load when choosing the action representation or image preprocessing.
- `references/evaluation-statistics.md`: tested success-rate statistics code, reporting
  template, why val loss misleads. Load when computing intervals, comparing policies,
  sizing an evaluation or writing results up.
- `references/deployment-runtime.md`: execution modes, tested ensembling and async code,
  latency budget, safety clamp. Load when writing the on-robot inference loop.
- `references/corrections-and-cotraining.md`: interventions, targeted recollection, sim
  co-training. Load when improving a policy after evaluation or using sim data.
- `references/failure-modes.md`: extra failure rows, overfitting signs. Load when a
  symptom is not in the Failure Modes table or the policy only works near demo poses.

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

How many demos (10-20 up to 1000+): `references/data-collection.md`.

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

Default: wrist camera plus at least one external camera, exposure and white balance
locked. Teleop modality comparison, camera details and per-session checklist:
`references/data-collection.md`.

## Decision 2: Observation and Action Design

Action-space table, proprio pitfall, image crop: `references/observation-action.md`.

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

Per-family notes (BC, ACT, Diffusion, flow, VLAs): `references/policy-landscape.md`.

**Fine-tune vs from scratch:** fine-tune a pretrained policy when your embodiment or
action convention is close to its pretraining data, when you need language or
generalisation, or when you have few demos per task across many tasks. Train from
scratch when latency is tight, the task is narrow, or the licence does not fit. When in
doubt, run both on the same evaluation set; the comparison costs less than an argument.

Licences differ and can disagree between a README and the checkpoint LICENSE; check
`references/policy-landscape.md`, then the checkpoint repo, before a product decision.

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

Fine-tuning VLAs: keep the action normalisation and camera keys exactly as the
fine-tuning config expects. Provider recipes, LoRA vs full: `references/policy-landscape.md`.

Use validation loss to catch bugs (divergence, a loss that never drops, a large
train/val gap suggesting leakage or memorisation), not to pick the deployed checkpoint.
Why it misleads (robomimic, SIMPLER): `references/evaluation-statistics.md`.

### Checkpoint selection by rollouts

- Shortlist 3-5 checkpoints (for example late-training ones plus the val-loss minimum).
- Screen them on a small fixed set (10-20 trials each, same initial conditions) in sim
  or on the robot to discard clearly bad ones.
- Run the full evaluation protocol only on the 1-2 survivors. Report which checkpoint
  and why; selecting the best of many on the same trials inflates the reported rate, so
  confirm on fresh trials.

Overfitting signs and fixes: `references/failure-modes.md`.

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

### Statistics

Tested code (Wilson, Fisher, McNemar, sample size), full output and the reporting
template: `references/evaluation-statistics.md`.

What this means in practice:
- **8/10 is consistent with anything from about 49% to 94%.** 10/10 still allows 72%.
- **8/10 vs 5/10 is not evidence that A is better** (p = 0.35). The same rates over 50
  trials each are.
- Telling 50% from 80% needs ~40 trials per policy; 70% from 80% needs ~300. Small
  improvements on a good policy are expensive to prove; say so instead of claiming them.

## Decision 6: Deployment

- **Run at the rate you trained on.** A chunk index means a time offset only at the
  demonstrated control rate. If the policy outputs at 10 Hz and the arm needs 100+ Hz,
  interpolate between targets in the low-level controller (Diffusion Policy interpolated
  10 Hz commands to 125 Hz).
- **Measure latency** (camera, inference p99, command-to-motion). The policy acts on
  observations that are that old. `references/deployment-runtime.md` has a procedure.
- **Pick an execution mode:** synchronous (simplest), temporal ensembling (ACT) or
  asynchronous (large VLAs at high rates); trade-offs in `references/deployment-runtime.md`.
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
| Oscillation around the target, worse with bigger models | Latency: acting on stale observations, often with per-step delta actions | Measure latency; async execution with stale-action dropping; absolute or relative-chunk targets; smaller or faster model |
| Wild, fast motions on first deployment | Wrong or missing normalisation stats; units (deg vs rad); joint order | Load stats from checkpoint; open-loop replay test; safety clamp |

More rows (appearance, camera bump, chunk jerk, sim-to-real): `references/failure-modes.md`.

Correction data (HG-DAgger) and sim co-training: `references/corrections-and-cotraining.md`.

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
