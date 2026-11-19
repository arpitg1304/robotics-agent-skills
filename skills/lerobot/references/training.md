# LeRobot Training Details

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Detail behind the `lerobot-train`
command in SKILL.md: fresh vs fine-tune, Hub push, resume, checkpoint layout, Trackio and HF Jobs,
devices, the SmolVLA fine-tune command and ACT defaults. For the policy list, extras and VRAM
groups, see [policies.md](policies.md). For every flag, see [cli-reference.md](cli-reference.md).

## Notes on `lerobot-train`

- `--policy.type=X` builds a fresh policy whose input and output features come from the dataset.
  `--policy.path=<repo|dir>` fine-tunes a checkpoint and reads the type and config from it.
- `--policy.push_to_hub` defaults to **true**. It needs `--policy.repo_id`, or pass
  `--policy.push_to_hub=false`.
- An existing `--output_dir` without `--resume=true` raises `FileExistsError`. Resume with:
  ```bash
  lerobot-train --config_path=outputs/train/act_so101_cube_to_bowl/checkpoints/last/pretrained_model/train_config.json --resume=true
  lerobot-train --config_path=${HF_USER}/act_so101_cube_to_bowl --resume=true   # Hub, needs --save_checkpoint_to_hub=true during training
  ```
- Checkpoints go in `<output_dir>/checkpoints/<step>/pretrained_model/` (plus `last`). Each holds
  `config.json`, `model.safetensors`, `train_config.json`, and the pre/post-processor pipelines
  with their normalisation stats.
- Trackio is local by default; open it with `trackio show --project lerobot`. Use
  `--trackio.space_id=<user>/<space>` to sync to a Space. That is required for HF Jobs
  (`--job.target=a10g-small`). List hardware with `hf jobs hardware`.
- Devices: `cuda`, `mps`, `cpu`, `xpu`. On ROCm, `cuda` is used. Multi-GPU:
  `accelerate launch --num_processes=N $(which lerobot-train) ...`. **Advice:** MPS works for ACT
  but is slow (the docs estimate about 6–14 h for 5 epochs). For big VLAs, rent a GPU.
- SmolVLA fine-tune: `smolvla_base` expects cameras `camera1..3`, so map your names:
  ```bash
  lerobot-train --policy.path=lerobot/smolvla_base --policy.repo_id=${HF_USER}/smolvla_so101_cube \
    --dataset.repo_id=${HF_USER}/so101_cube_to_bowl_20261118_143012 \
    --rename_map='{"observation.images.top": "observation.images.camera1", "observation.images.wrist": "observation.images.camera2"}' \
    --batch_size=64 --steps=20000 --policy.device=cuda --trackio.enable=true
  ```
- Key ACT knobs (verified defaults): `chunk_size=100`, `n_action_steps=100`, `n_obs_steps=1`,
  `vision_backbone=resnet18`, `use_vae=true`, `kl_weight=10`, `optimizer_lr=1e-5`,
  `temporal_ensemble_coeff=None`.
