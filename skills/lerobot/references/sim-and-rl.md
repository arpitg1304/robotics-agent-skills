# LeRobot Simulation Evaluation and HIL-SERL

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Load this file to evaluate a policy
in simulation with `lerobot-eval`, or to run HIL-SERL (online RL) in LeRobot. Flag tables for
`lerobot-eval` are in [cli-reference.md](cli-reference.md).

## Simulation Evaluation (`lerobot-eval`)

```bash
pip install 'lerobot[pusht]'      # or aloha, metaworld, libero (Linux)
lerobot-eval --policy.path=<policy_repo_or_dir> --env.type=pusht \
  --eval.n_episodes=50 --eval.batch_size=10 --policy.device=cuda --output_dir=outputs/eval/pusht
```
Registered env types (verified) include `pusht`, `aloha`, `libero`, `libero_plus`, `metaworld`,
`robocasa`, `robotwin`, `vlabench`, `isaaclab_arena`, `envhub` and `unitree_g1_mujoco`. Some
need manual installs; see `docs/source/<env>.mdx`. `--rename_map` maps env camera names to
policy names. `--robot.type=sim` with `lerobot-rollout` runs the same rollout loop against a sim
env. For building the simulator itself, see `mujoco` / `isaac-sim`.

## HIL-SERL / RL in LeRobot

LeRobot ships HIL-SERL: a SAC-style actor/learner, a reward classifier and human interventions
by gamepad. It needs `lerobot[hilserl]`, an NVIDIA GPU, and a URDF for end-effector bounds. The
entry points are `python -m lerobot.rl.gym_manipulator --config_path rl/env_config.json`
(record, test), `lerobot-train --config_path reward_classifier_train_config.json`, and
`python -m lerobot.rl.learner` / `python -m lerobot.rl.actor --config_path rl/train_config.json`.
Get the example configs with
`hf download lerobot/config_examples rl/env_config.json rl/train_config.json --repo-type dataset --local-dir .`.
`lerobot-find-joint-limits` helps set the bounds. For algorithm choices, rewards and safety
envelopes, use `robot-reinforcement-learning`. For the exact config schema, read
`docs/source/hilserl.mdx` for your version.
