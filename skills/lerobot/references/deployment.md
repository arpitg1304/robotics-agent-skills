# LeRobot Deployment, Inference and Processors

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Detail behind the `lerobot-rollout`
section of SKILL.md: recorded evaluation, strategies, load-time overrides, inference from your
own Python loop, and the processor pipelines. Flag tables are in
[cli-reference.md](cli-reference.md). Simulation evaluation and HIL-SERL are in
[sim-and-rl.md](sim-and-rl.md).

## `lerobot-rollout` (base and episodic)

```bash
# Quick look, no recording
lerobot-rollout --strategy.type=base \
  --policy.path=${HF_USER}/act_so101_cube_to_bowl \
  --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID \
  --robot.cameras="$CAMERAS" \
  --task="Pick up the red cube and place it in the bowl" --duration=60 --display_data=true

# Scored evaluation episodes, recorded; leader drives resets
lerobot-rollout --strategy.type=episodic \
  --policy.path=${HF_USER}/act_so101_cube_to_bowl \
  --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID --robot.cameras="$CAMERAS" \
  --teleop.type=so101_leader --teleop.port=$LEADER_PORT --teleop.id=$LEADER_ID \
  --dataset.repo_id=${HF_USER}/rollout_act_cube_to_bowl \
  --dataset.single_task="Pick up the red cube and place it in the bowl" \
  --dataset.num_episodes=10 --dataset.episode_time_s=30 --dataset.reset_time_s=15
```
- Strategies (verified): `base` runs without recording. `episodic` records episodes like
  `lerobot-record`. `sentry` records continuously and uploads every N episodes. `highlight` keeps a
  ring buffer and saves when you press a key. `dagger` is human-in-the-loop correction and needs
  `--teleop.type`.
- Recorded rollout datasets must be named `rollout_*`, and a timestamp is appended.
  `--dataset.fps` must equal `--fps` (default 30).
- `--policy.<field>=` overrides the checkpoint config at load time. For example,
  `--policy.n_action_steps=20` re-plans more often. `--device` overrides the device.
  `--return_to_initial_position` (default true) moves the arm back to its startup pose on exit.
  `--interactive=true` waits for `/start` (base and sentry).

## Inference from your own Python loop

Inference from your own Python loop (verified API, from `examples/tutorial/act/act_using_example.py`):
```python
from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.policies import make_pre_post_processors
from lerobot.policies.act import ACTPolicy
from lerobot.policies.utils import build_inference_frame, make_robot_action

policy = ACTPolicy.from_pretrained(MODEL_ID).to("cuda").eval()
meta = LeRobotDatasetMetadata(DATASET_ID)           # features/names of the training data
pre, post = make_pre_post_processors(policy.config, pretrained_path=MODEL_ID)  # stats saved with the checkpoint
policy.reset()                                       # clear the action queue at every episode start
obs = robot.get_observation()
frame = build_inference_frame(observation=obs, ds_features=meta.features, device="cuda")
action = post(policy.select_action(pre(frame)))
robot.send_action(make_robot_action(action, meta.features))
```

## Processors (pipelines)

Since the normalisation refactor (PR #1452), policies no longer embed normalisation. Each checkpoint
ships a **preprocessor**: rename, add batch dim, move to device, normalise, plus policy steps such
as tokenisation. It also ships a **postprocessor**: unnormalise, move to CPU. The pattern is
`preprocessor(batch) -> policy.select_action -> postprocessor(action)`. Robot-side
`RobotProcessorPipeline`s sit between hardware and dataset. `make_default_processors()` returns
identity teleop-action, robot-action and robot-observation pipelines. The `so100_to_so100_EE`
example shows end-effector teleop with kinematics. A model trained before #1452 has stats in its
weights and must be migrated:
`python src/lerobot/processor/migrate_policy_normalization.py --pretrained-path <repo> --push-to-hub --branch migrated`.
See `docs/source/introduction_processors.mdx` and `debug_processor_pipeline.mdx`.
