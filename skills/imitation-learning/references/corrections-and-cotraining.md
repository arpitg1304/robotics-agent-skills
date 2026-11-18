# Correction Data, Sim Data and Co-Training (reference)

Load this file after the first evaluation, when choosing how to improve a policy that
fails in specific ways, or when sim data is part of the plan.

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
