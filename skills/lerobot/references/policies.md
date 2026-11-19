# LeRobot Policies

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Types come from
`@PreTrainedConfig.register_subclass` in `src/lerobot/policies/*/configuration_*.py`. Extras
come from `pyproject.toml`. VRAM groups come from `docs/source/hardware_guide.mdx` (indicative).
For *which algorithm to choose* and the theory behind chunking, diffusion and flow matching,
use the `imitation-learning` skill. This file covers the LeRobot specifics only.

List the types in your install:
```bash
python -c "import lerobot.policies; from lerobot.configs.policies import PreTrainedConfig as P; print(sorted(P.get_known_choices()))"
lerobot-train --policy.type=act --help     # every --policy.* field with defaults
```

## Registered policy types at this commit

| `--policy.type` | Extra | Group / VRAM (BS 8) | Notes |
|---|---|---|---|
| `act` | none | Light BC, ~2–6 GB | First choice for a single SO-101 task. ResNet18 + transformer + CVAE |
| `diffusion` | `diffusion` | ~8–14 GB | `n_obs_steps=2`, `horizon=64`, `n_action_steps=32` defaults |
| `multi_task_dit` | `multi_task_dit` | ~8–14 GB | Diffusion transformer, multi-task |
| `vqbet` | none | Light BC | |
| `tdmpc` | none | Light | Model-based RL/IL |
| `smolvla` | `smolvla` | ~10–16 GB | Fine-tune `lerobot/smolvla_base`. Cameras `camera1..3`. Images padded to 512×512. `max_state_dim=32` |
| `pi0` | `pi` | ~24–40 GB | `--policy.pretrained_path=lerobot/pi0_base` |
| `pi05` | `pi` | ~24–40 GB | `lerobot/pi05_base`. **QUANTILES** normalisation for state and action by default |
| `pi0_fast` | `pi` | ~24–40 GB | `lerobot/pi0fast-base`. Autoregressive FAST tokens |
| `groot` | `groot` | ~24–40 GB | **N1.7 only** (`--policy.base_model_path=nvidia/GR00T-N1.7-3B`). N1.5 needs `lerobot==0.5.1` |
| `xvla` | `xvla` | ~24–40 GB | `lerobot/xvla-base`. Expects `image`, `image2`, `image3` |
| `wall_x`, `wall_oss_05` | `wallx`, `wall_oss_05` | Large VLA | |
| `eo1` | `eo1` | Multimodal | Can generate text (`/vqa`, `/autosteer` at rollout) |
| `molmoact2` | `molmoact2` | Large | Radians / 0–1 gripper conventions (see the YAM docs) |
| `gaussian_actor` | none listed (workflow needs `hilserl`) | RL | Actor used by the SAC algorithm in `lerobot.rl` (HIL-SERL) |
| `cosmos3_edge`, `cosmos3_nano`, `dm05`, `evo1`, `fastwam`, `fineart_vla`, `flux3`, `g05`, `lawam`, `lingbot_va`, `lingbot_vla_v2`, `longwam`, `praxis`, `vla_jepa` | same-named extra | mostly large | Newer integrations. Read `docs/source/<name>.mdx` before using them |

The docs list `act`, `diffusion`, `multi_task_dit`, `smolvla`, `pi0`, `pi05`, `pi0_fast`, `groot`
and `xvla` as the common types. Treat the rest as newer and less battle-tested (advice).

## Fresh vs fine-tune
- `--policy.type=act` builds a new model. Input and output features are derived from the
  dataset, so cameras and state are picked up automatically. Backbones may still load ImageNet
  or VLM weights.
- `--policy.path=<repo|dir>` loads the config **and** weights. The type comes from the checkpoint.
  Features are fixed by the checkpoint, so you may need `--rename_map` and `--policy.empty_cameras=N`
  (when you have fewer cameras than the model expects).
- `--policy.type=pi05 --policy.pretrained_path=lerobot/pi05_base` is the form the pi docs use. It
  sets the type explicitly and points at base weights.
- `--rename_map` is only accepted with a pretrained policy. It renames **observation** keys
  only. The docs list PI0, PI05, PI0Fast, SmolVLA and XVLA as supported.

## ACT (verified defaults)
`chunk_size=100`, `n_action_steps=100`, `n_obs_steps=1`, `vision_backbone=resnet18`
(ImageNet weights), `dim_model=512`, `n_heads=8`, `n_encoder_layers=4`, `n_decoder_layers=1`,
`use_vae=true`, `latent_dim=32`, `kl_weight=10.0`, `dropout=0.1`, `optimizer_lr=1e-5`,
`optimizer_lr_backbone=1e-5`, `optimizer_weight_decay=1e-4`, `temporal_ensemble_coeff=None`.
- Temporal ensembling needs `n_action_steps=1`. Otherwise the config raises
  `NotImplementedError`. 0.01 is the value from the paper.
- No LR scheduler preset (`get_scheduler_preset` returns None), so there is no decay to retune
  when you shorten `--steps`.
- Remote inference needs temporal ensembling disabled.

## SmolVLA
```bash
pip install 'lerobot[smolvla]'
lerobot-train --policy.path=lerobot/smolvla_base --dataset.repo_id=<ds> --policy.repo_id=<you>/smolvla_x \
  --rename_map='{"observation.images.top": "observation.images.camera1", "observation.images.wrist": "observation.images.camera2"}' \
  --batch_size=64 --steps=20000 --policy.device=cuda
```
Defaults: `chunk_size=50`, `n_action_steps=50`, `freeze_vision_encoder=true`,
`train_expert_only=true`, `scheduler_warmup_steps=1000`, `scheduler_decay_steps=30000`.
Set `--policy.scheduler_decay_steps` near `--steps`. Normalisation is MEAN_STD for state and
action.

## pi0 / pi0.5 / pi0-FAST
```bash
pip install 'lerobot[pi]'
lerobot-train --dataset.repo_id=<ds> --policy.type=pi05 --policy.pretrained_path=lerobot/pi05_base \
  --policy.dtype=bfloat16 --policy.gradient_checkpointing=true --policy.device=cuda \
  --policy.repo_id=<you>/pi05_x --batch_size=8 --steps=20000
```
- pi0.5 normalises STATE and ACTION with QUANTILES. Datasets without `q01`/`q99` fail on the
  first batch with `QUANTILES normalization mode requires q01 and q99 stats`. Fix it with
  `lerobot-edit-dataset --operation.type recompute_stats --operation.overwrite true` (output goes
  to `$HF_LEROBOT_HOME/<id>`, so pass `--dataset.root` or push it), or override with
  `--policy.normalization_mapping='{"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}'`.
- Memory flags: `--policy.freeze_vision_encoder`, `--policy.train_expert_only`,
  `--policy.gradient_checkpointing`, `--policy.dtype=bfloat16`, `--policy.compile_model` (pi0).
- RTC-trained checkpoints: `--policy.rtc_training_max_delay=<n>` at train time, then
  `--inference.type=rtc --inference.rtc.mode=trained` at rollout (see `docs/source/rtc.mdx`).
- Use `--policy.n_action_steps` to choose how much of each chunk runs before re-planning.

## Deployment pairing
| Policy | Suggested rollout backend (advice, grounded in the docs) |
|---|---|
| ACT, diffusion | `sync`. Tune `--policy.n_action_steps` |
| SmolVLA, pi0, pi0.5 | `--inference.type=rtc` on a local GPU, or `remote` from a GPU server |
| Policies whose text heads are used for `/vqa`/`/autosteer` | `--interactive=true` |

## Training throughput knobs
- `--batch_size`, `--num_workers`, `--dataset.image_transforms.backend=gpu`,
  `--dataset.video_backend`, `--accelerator.mixed_precision=bf16`.
- Multi-GPU: `accelerate launch --num_processes=N $(which lerobot-train) ...` or `torchrun`. All
  settings stay in the LeRobot config (`docs/source/multi_gpu_training.mdx`).
- PEFT/LoRA: `--peft.*` or `--policy.use_peft=true`. It needs `--policy.path`
  (`docs/source/peft_training.mdx`).
- Logs split `dataloading_s`, `preprocessing_s` and `update_s`. If dataloading dominates, the
  GPU is starved.

## Old checkpoints
- Models trained before the processor refactor (PR #1452) have normalisation inside their
  weights. Migrate them with
  `python src/lerobot/processor/migrate_policy_normalization.py --pretrained-path <repo> --push-to-hub --branch migrated`.
- A checkpoint `train_config.json` with a `wandb` block is migrated to `trackio` automatically
  when loaded.
- Checkpoints from a different LeRobot minor version may not load. Pin the version that trained
  them (`pip install lerobot==X.Y.Z`) when you need to reproduce a result.
