# RL Library Comparison for Robotics

Library landscape checked 2026-11-18 against each project's GitHub repository (latest
release tag, last push, README). Versions move fast: re-check `pip index versions <pkg>`
or the release page before pinning, and pin the exact version in your lockfile.

## Summary Table

| Library | Checked version | Framework | Core algorithms | Pairs natively with | Pick it when |
|---|---|---|---|---|---|
| **rsl_rl** (`pip install rsl-rl-lib`) | v5.5.2 (2026-10-28) | PyTorch | PPO, SAC, student-teacher distillation; RND and symmetry augmentation as PPO options; more on the `extras` branch | Isaac Lab, legged_gym, mjlab, MuJoCo Playground 1.x | Locomotion / whole-body control with thousands of GPU envs. Small, readable, what most legged sim-to-real papers use. |
| **rl_games** | v1.6.5 (2026-02-20); 2.0.0 unreleased (Gymnasium-only, Python >= 3.11) | PyTorch | PPO, SAC, asymmetric actor-critic, multi-GPU, population-based training | Isaac Lab, MJLab, EnvPool, MyoSuite | Dexterous manipulation (DeXtreme and DexPBT were trained with it), or when an existing Isaac Lab task ships an rl_games config. |
| **skrl** | 2.1.2 (2026-11-15) | PyTorch, JAX, NVIDIA Warp | Broad set of on/off-policy algorithms, multi-agent | Gymnasium, PettingZoo, Isaac Lab, MuJoCo Playground, ManiSkill | You need algorithm breadth, a JAX backend, multi-agent, or training several env "scopes" in one run. |
| **Stable-Baselines3** + **sb3-contrib** | v2.9.2 (2026-10-26) both | PyTorch | PPO, A2C, SAC, TD3, DDPG, DQN, HER; contrib adds TQC, CrossQ, RecurrentPPO, TRPO, ARS, MaskablePPO, QR-DQN | Any Gymnasium env; Isaac Lab ships an SB3 wrapper | CPU Gymnasium envs, quick baselines, a first SAC on a real robot, tuned hyperparameters via RL Zoo. Not built for 4096 GPU envs: data goes through NumPy. |
| **CleanRL** | last release v1.0.0 (2022); repo pushed 2026-04 | PyTorch / JAX | Single-file PPO, SAC, TD3, DDPG, PQN, RPO, ... including `ppo_continuous_action_isaacgym` | Gymnasium, EnvPool, Isaac Gym | Reading or modifying algorithm internals. A reference, not a framework; treat as stable rather than actively developed. |
| **Brax training** | v0.14.2 (2026-03-15) | JAX | `brax.training.agents`: PPO, SAC, APG, ARS, ES, BC | Brax envs, MJX envs (including Playground's original JAX suite) | Entire loop in JAX on accelerators (env + learner jit-compiled together). |
| **MuJoCo Playground** | v1.0.1 (2026-11-09) | PyTorch (1.x) | Uses rsl_rl for large-batch RL | MuJoCo Warp, `reaf-sim`; sim and real SO-101 / YAM workflows | See the `mujoco` skill. Note: 1.x moved to PyTorch + MuJoCo Warp; the RSS 2025 JAX/MJX environments live under `rss2025/` and train with Brax. |
| **TorchRL** | v0.15.0 (2026-11-10) | PyTorch | Primitives (losses, collectors, replay buffers, TensorDict) | Gymnasium, Isaac Lab wrapper, others | Building your own algorithm from components. |
| **LeRobot HIL-SERL** | LeRobot v0.7.0 (2026-11-05) | PyTorch | Distributed SAC actor/learner, reward classifier, human interventions | LeRobot robots/teleoperators (SO-100/101, Koch, gamepad) | Real-robot manipulation RL on LeRobot hardware. See the `lerobot` skill. |
| **SERL / HIL-SERL** (rail-berkeley) | no recent release; pushed 2025-10 | JAX | RLPD-style SAC, reward classifiers, Franka impedance controller | Franka + their gym envs | Reproducing the papers; reference for design choices. |
| **RLPD** (ikostrikov/rlpd) | 2023 reference code | JAX | SAC + offline data recipe | D4RL-style data | The recipe matters more than the code; it is built into SERL/HIL-SERL. |
| **FastTD3 / FastSAC** (younggyoseo) | research code (2025) | PyTorch | Off-policy TD3/SAC tuned for thousands of parallel envs | HumanoidBench, Isaac Lab, MuJoCo Playground | You are trying off-policy RL in massively parallel sim (humanoids). Newer and less battle-tested than PPO pipelines. |

Isaac Lab v3.0.0 (2026-10-20) ships wrappers in `isaaclab_rl` for rsl_rl, rl_games, skrl,
SB3 and TorchRL, so on Isaac Lab the library choice is mostly about which algorithm and
config style you want. See the `isaac-sim` skill for task and wrapper details.

## Decision Guide

```
Massively parallel GPU sim (Isaac Lab / mjlab / MuJoCo Warp)?
├── Legged / humanoid locomotion, tracking  -> rsl_rl PPO (default); FastSAC/FastTD3 if wall-clock matters and you can tune
├── Dexterous in-hand manipulation          -> rl_games PPO (existing configs), or rsl_rl
├── Need JAX end to end                     -> Brax training (MJX) or skrl[jax]
└── Need an algorithm rsl_rl lacks          -> skrl, or TorchRL components
CPU Gymnasium env, tens of envs?
├── Baseline quickly                        -> SB3 (PPO for cheap sims, SAC/TQC/CrossQ for expensive ones)
└── Want to edit the algorithm              -> CleanRL single file
Real robot, hours of data?
├── LeRobot hardware                        -> LeRobot HIL-SERL
├── Franka / custom arm                     -> SERL/HIL-SERL recipe (port to your stack) or SB3 SAC as a sanity baseline
└── Already have a BC / diffusion / VLA policy -> residual RL, DSRL, or RECAP-style advantage conditioning (see hybrid-methods.md)
```

## Interop Gotchas

- **Observation dict vs tensor.** rsl_rl v5 routes named observation groups to actor and critic
  through `obs_groups` (Isaac Lab configs write e.g. `{"actor": ["policy"], "critic": ["policy"]}`);
  asymmetric critics add a privileged group to the critic list. Older stacks passed privileged
  observations differently (legged_gym uses `num_privileged_obs`). Check the rsl_rl major
  version before copying a config from a paper repo.
- **Time-out signal.** rsl_rl bootstraps on `extras["time_outs"]`; SB3 replay buffers read
  `info["TimeLimit.truncated"]` (`handle_timeout_termination=True` by default); Gymnasium vector
  envs return separate `terminated` / `truncated` arrays. A custom wrapper that drops any of
  these silently turns time-outs into terminal states.
- **Gymnasium 1.x autoreset.** Gymnasium v1.4.0 is current. Vector envs default to
  `AutoresetMode.NEXT_STEP`: the step after a done returns the reset observation, and that
  transition crosses episodes and must be masked. In `SAME_STEP` mode the true final
  observation is in `info["final_obs"]` (`"final_observation"` in Gym 0.x). Mixing modes is a
  classic source of wrong bootstrap targets.
- **Normaliser state.** SB3 `VecNormalize`, rsl_rl `obs_normalization`, and rl_games
  `normalize_input` each store statistics differently. Export them with the policy (or bake them
  into the exported ONNX/TorchScript graph) and assert they load on the robot.
- **Action clipping.** Gaussian PPO policies are unbounded; clipping happens in the env or
  wrapper. SAC-family policies are tanh-squashed to [-1, 1] and need rescaling to physical units.
