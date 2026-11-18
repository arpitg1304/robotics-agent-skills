# Hybrid RL + Imitation Methods

Moved from SKILL.md section 1. Representative work for combining RL with demonstrations,
base controllers, or pretrained BC / diffusion / VLA policies.

## Hybrid Approaches (fast-moving area; verify before committing)

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
