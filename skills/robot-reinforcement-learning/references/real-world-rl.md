# Real-World RL: Safety, Resets, Rewards, Interventions

Moved from SKILL.md section 6.

## Safety Layers (outside the policy, fail closed)

| Layer | Implementation |
|---|---|
| Workspace bounds | Clip EE targets to a box (LeRobot `end_effector_bounds`; HIL-SERL bounds) |
| Step limits | Max EE translation/rotation per step (LeRobot `end_effector_step_sizes`) |
| Compliance | Impedance/admittance control with bounded stiffness; force/torque thresholds that abort |
| Joint and torque limits | Enforced in the driver, below the policy |
| Watchdog | Hold position if no action arrives within one control period plus margin |
| Human override | Intervention device always active; E-stop reachable |

## Resets

- **Scripted resets:** return to a fixed or randomised joint configuration (LeRobot
  `fixed_reset_joint_positions`, `reset_time_s`).
- **Human resets:** acceptable for hours-long runs; budget the human time.
- **Learned reset policy / forward-backward training:** a second policy undoes the task.
- **Reset-free:** design the task to be cyclic (pick, place, pick again).
- Randomise the reset pose slightly so the policy does not overfit one start.

## Reward From Classifiers

- Train a binary success classifier on images: HIL-SERL reports typically ~200 positive and
  ~1000 negative frames, with >95% accuracy on held-out data. LeRobot trains one with
  `lerobot-train` and applies it with `success_threshold`.
- The policy will search for false positives. Add frames from policy rollouts as hard
  negatives, and periodically audit classifier positives by eye.
- Zero-shot VLM reward models exist (LeRobot integrates TOPReward). Treat them as shaping or
  progress signals and validate against human labels before letting the policy optimise them.

## Human Interventions

- HIL-SERL stores intervention transitions in both the demonstration and RL buffers.
- A healthy run shows intervention rate falling over training (LeRobot's guide plots it);
  flat or rising intervention rate means the reward, action space or observation is wrong.
- Intervene to correct and to prevent damage, not to complete every episode; constant takeover
  turns RL into slow imitation.
