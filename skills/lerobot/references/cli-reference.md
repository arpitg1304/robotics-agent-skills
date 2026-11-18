# LeRobot CLI Reference

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Field names and defaults come from
the draccus config dataclasses in `src/lerobot/scripts/*.py`, `src/lerobot/configs/*.py` and
`src/lerobot/rollout/configs.py`. **These tables go stale.** For the installed version, run
`<command> --help`, or read the dataclass in `site-packages/lerobot/`
(`python -c "import lerobot, os; print(os.path.dirname(lerobot.__file__))"`).

## Console scripts (from `pyproject.toml`)

| Command | Purpose |
|---|---|
| `lerobot-info` | System and version report (works even with a broken install) |
| `lerobot-find-port` | Find a motor bus serial port by unplugging it |
| `lerobot-find-cameras [opencv\|realsense]` | List cameras and save test frames to `outputs/captured_images` (`--output-dir`, `--record-time-s`, `--warmup-s`) |
| `lerobot-setup-motors` | Write motor IDs and baudrate, one motor at a time (`--robot.*` or `--teleop.*`) |
| `lerobot-calibrate` | Calibrate a robot or teleop (`--robot.*` or `--teleop.*`) |
| `lerobot-teleoperate` | Leader → follower loop, with optional cameras and display |
| `lerobot-record` | Teleoperated data collection into a LeRobotDataset |
| `lerobot-replay` | Replay one dataset episode's actions on a robot |
| `lerobot-train` | Train a policy or reward model (locally or on HF Jobs) |
| `lerobot-rollout` | Deploy a policy on a robot, or on `--robot.type=sim` |
| `lerobot-policy-server` | Server side of remote inference (`lerobot[remote]`) |
| `lerobot-eval` | Evaluate a policy in a gym/sim env |
| `lerobot-env-server` | Serve a sim env to remote `lerobot-eval` clients |
| `lerobot-dataset-viz` | Rerun/Foxglove viewer for one episode |
| `lerobot-imgtransform-viz` | Preview image augmentations |
| `lerobot-edit-dataset` | Delete, split, merge, retask, recompute stats, re-encode, info |
| `lerobot-find-joint-limits` | Teleop while recording EE/joint bounds (needs `--urdf_path`) for HIL-SERL |
| `lerobot-setup-can` | Configure and test CAN interfaces (OpenArm, Damiao, Robstride) |
| `lerobot-annotate` | VLM-based language annotation pipeline (`lerobot[annotations]`) |
| `lerobot-train-tokenizer`, `lerobot-convert-dcp`, `lerobot-build-mp4-sidecar` | Tokenizer training, sharded-checkpoint conversion, streaming index |

Non-console entry points: `python -m lerobot.scripts.convert_dataset_v21_to_v30`,
`python src/lerobot/scripts/augment_dataset_quantile_stats.py`,
`python src/lerobot/processor/migrate_policy_normalization.py`, `python -m lerobot.rl.{gym_manipulator,learner,actor}`,
`python -m lerobot.robots.lekiwi.lekiwi_host`.

## Common robot / teleop flags (SO family)

| Flag | Default | Notes |
|---|---|---|
| `--robot.type` | — | `so101_follower`, `so100_follower`, `bi_so_follower`, `koch_follower`, ... |
| `--robot.port` | — | Serial port |
| `--robot.id` | — | Calibration file name. Keep it constant |
| `--robot.calibration_dir` | `$HF_LEROBOT_CALIBRATION/robots/<class name>` | Override the calibration location |
| `--robot.cameras` | `{}` | Dict of camera configs. Keys become `observation.images.<key>` |
| `--robot.use_degrees` | `true` (SO) | `false` → −100..100 normalised range. Gripper is always 0..100 |
| `--robot.max_relative_target` | `None` | Float or per-motor dict. Clips each step's target delta (reads the bus every step, so a little slower) |
| `--robot.disable_torque_on_disconnect` | `true` | |
| `--robot.num_read_retries` | `2` | Retries on a corrupted Feetech status packet |
| `--robot.position_p_coefficient` / `_i_` / `_d_` | `16` / `0` / `32` | STS3215 position PID, written at connect |
| `--teleop.type` / `.port` / `.id` / `.use_degrees` | — / — / — / `true` | `so101_leader`, `so100_leader`, `keyboard`, `gamepad`, `phone`, ... |

Camera config (`type: opencv`): `index_or_path`, `width`, `height`, `fps`, `color_mode`
(`rgb`/`bgr`), `rotation` (`0`, `90`, `180`, `-90`; 270° is spelled `-90`), `warmup_s` (1),
`fourcc` (4 chars, e.g. `MJPG`), `backend`. RealSense (`type: intelrealsense`):
`serial_number_or_name`, `width`, `height`, `fps`, `use_rgb`, `use_depth`, `rotation`,
`warmup_s`, `exposure`, `gain`, `white_balance`. There is also `type: zmq` (network camera, no
auto-discovery).

## `lerobot-teleoperate`

| Flag | Default |
|---|---|
| `--fps` | `60` |
| `--teleop_time_s` | `None` (until Ctrl-C) |
| `--display_data` | `false` |
| `--display_mode` | `rerun` (or `foxglove`) |
| `--display_ip` / `--display_port` | remote Rerun server, or Foxglove bind address |
| `--display_compressed_images` | `false` |

## `lerobot-record` (top-level + `--dataset.*` = `DatasetRecordConfig`)

| Flag | Default | Notes |
|---|---|---|
| `--resume` | `false` | Top level, not under `dataset` |
| `--display_data`, `--display_mode`, `--display_ip`, `--display_port` | `false`, `rerun` | |
| `--play_sounds` | `true` | Spoken prompts |
| `--dataset.repo_id` | `""` | `user/name`. A `_YYYYMMDD_HHMMSS` stamp is appended on creation |
| `--dataset.no_stamp` | `false` | Keep the exact name |
| `--dataset.single_task` | `""` | Task string stored per frame |
| `--dataset.root` | `$HF_LEROBOT_HOME/<repo_id>` | The dataset directory itself |
| `--dataset.fps` | `30` | Loop rate = dataset fps |
| `--dataset.episode_time_s` | `60` | |
| `--dataset.reset_time_s` | `60` | No reset after the last episode |
| `--dataset.num_episodes` | `50` | On resume: *additional* episodes |
| `--dataset.video` | `true` | `false` stores PNG frames |
| `--dataset.push_to_hub` | `true` | |
| `--dataset.private` | `None` | |
| `--dataset.tags` | `None` | e.g. `'["so101","pick"]'` |
| `--dataset.num_image_writer_processes` | `0` | PNG path only |
| `--dataset.num_image_writer_threads_per_camera` | `4` | PNG path only |
| `--dataset.video_encoding_batch_size` | `1` | Encode every N episodes |
| `--dataset.streaming_encoding` | `false` | Encode during capture. `save_episode` becomes near-instant |
| `--dataset.encoder_queue_maxsize` | `30` | Per camera. When full, frames are **dropped** with a warning |
| `--dataset.encoder_threads` | `None` | Per encoder |
| `--dataset.rgb_encoder.{vcodec,pix_fmt,g,crf,preset,...}` | `libsvtav1`, `yuv420p`, `g=2` | `vcodec=auto` picks a HW encoder (e.g. `h264_videotoolbox`) |
| `--dataset.depth_encoder.{depth_min,depth_max,shift,use_log,...}` | | Depth quantisation |

- Defaults: `fps=30`, `episode_time_s=60`, `reset_time_s=60`, `num_episodes=50`, `video=true`,
  `push_to_hub=true`, `streaming_encoding=false`. Use `--dataset.private=true` and
  `--dataset.tags='["so101"]'` as needed.
- Episode keys (`→`/`n`, `←`/`r`, `ESC`/`q`): if no global key backend exists (Wayland, SSH, macOS
  without Accessibility), the same keys are read from the focused terminal.

## `lerobot-replay`

`--robot.*`, `--dataset.repo_id`, `--dataset.episode`, `--dataset.root`, `--dataset.fps` (30), `--play_sounds`.

## `lerobot-train` (`TrainPipelineConfig`)

| Flag | Default | Notes |
|---|---|---|
| `--dataset.repo_id` | — | Full stamped name |
| `--dataset.root` | `None` | Local dataset dir |
| `--dataset.episodes` / `--dataset.exclude_episodes` | `None` | e.g. `'[0,1,2]'` |
| `--dataset.revision` | `None` | Hub revision |
| `--dataset.image_transforms.enable` | `false` | `.backend=dataloader\|gpu`, `.max_num_transforms=3`, `.random_order=false` |
| `--dataset.use_imagenet_stats` | `true` | Image normalisation with ImageNet stats |
| `--dataset.video_backend` | platform default | torchcodec, or pyav as fallback |
| `--dataset.streaming` | `false` | Stream from the Hub (`StreamingLeRobotDataset`) |
| `--dataset.eval_split` | `0.0` | Held-out fraction (pair with `--eval_steps`) |
| `--dataset.repo_type` | `dataset` | `bucket` for HF Storage Buckets |
| `--policy.type` / `--policy.path` | — | New policy, or fine-tune or load a checkpoint |
| `--policy.device` | auto | `cuda`, `mps`, `cpu`, `xpu` |
| `--policy.dtype`, `--policy.use_amp` | `None`, `false` | |
| `--policy.repo_id`, `--policy.push_to_hub` (true), `--policy.private`, `--policy.tags`, `--policy.license` | | Hub upload of the final model |
| `--policy.pretrained_revision` | `None` | Pin a checkpoint revision or step tag |
| `--output_dir` | `outputs/train/<date>/<time>_<job>` | Must not exist unless resuming |
| `--job_name` | `<policy.type>` | |
| `--resume` + `--config_path` | `false` | Local `train_config.json` or Hub repo id |
| `--seed` | `1000` | |
| `--batch_size` | `8` | |
| `--num_workers` | `4` | DataLoader. `--dataloader_multiprocessing_context` defaults to `spawn` |
| `--steps` | `100000` | Size from epochs |
| `--save_freq` | `20000` | |
| `--log_freq` | `200` | |
| `--env_eval_freq` | `20000` | Only when `--env.type` is set |
| `--eval_steps`, `--max_eval_samples` | `0` | Held-out loss |
| `--tolerance_s` | `1e-4` | Video timestamp tolerance |
| `--use_policy_training_preset` | `true` | Otherwise set `--optimizer.*` and `--scheduler.*` |
| `--rename_map` | `{}` | Observation key remap. Needs a pretrained policy |
| `--trackio.enable` / `.project` (`lerobot`) / `.space_id` / `.private` / `.run_name` / `.capture_console` | `false` | Replaces `--wandb.*` |
| `--job.target` / `.timeout` (`2d`) / `.tags` / `.detach` / `.image` | local | HF Jobs flavor, e.g. `a10g-small` |
| `--save_checkpoint_to_hub` | `false` | Push every checkpoint and tag it with its step. Enables Hub resume |
| `--peft.*` / `--policy.use_peft` | | LoRA fine-tuning. Needs `--policy.path` |
| `--ema.type` | `None` | EMA weights |
| `--accelerator.mixed_precision` | | `fp16` on CPU is rejected |
| `--parallelism.dp_shard` | | Sharded (FSDP) training |

## `lerobot-rollout` (`RolloutConfig`)

| Flag | Default | Notes |
|---|---|---|
| `--strategy.type` | — | `base`, `episodic`, `sentry`, `highlight`, `dagger` (plus third-party) |
| `--policy.path` | required (unless remote) | Plus `--policy.<field>=` overrides and `--policy.pretrained_revision` |
| `--inference.type` | `sync` | `rtc` (`--inference.rtc.execution_horizon`, `.max_guidance_weight`, `.mode`, `.prefix_attention_schedule`), `remote` (`--inference.endpoint`, `.deployment`, `.mode`, `.chunk_merge`, `.semantics`, `.blend_*`) |
| `--fps` | `30` | Must equal `--dataset.fps` when recording |
| `--duration` | `0` (infinite) | Seconds. In interactive mode, per segment |
| `--device` | from policy, else auto | |
| `--task` | `""` | Propagated to and from `--dataset.single_task` |
| `--interpolation_multiplier` | `1` | Commands at `fps × N` |
| `--interactive` | `false` | `/start /subtask /vqa /autosteer /reset /stop /help` (base, sentry) |
| `--return_to_initial_position` | `true` | |
| `--rename_map` | `{}` | Camera/state key remap |
| `--use_torch_compile` | `false` | `compile_warmup_inferences=2` (RTC needs ≥ 3) |
| `--status_interval_s` | `5` | Health line |
| `--resume` | `false` | Resume a rollout dataset |
| `--dataset.*` | | As in record. `repo_id` must start with `rollout_` |
| `--teleop.*` | | Required for `dagger`. Optional for `episodic` resets |
| `--planner.model_id`, `.api_base`, `.api_key_env`, `.instructions` | | External VLM planner for `/vqa` and `/autosteer` |

Strategy-specific flags: `sentry` takes `--strategy.upload_every_n_episodes` (5) and
`--strategy.target_video_file_size_mb`. `highlight` takes `--strategy.ring_buffer_seconds` (10),
`--strategy.ring_buffer_max_memory_mb` (1024), `--strategy.save_key` (s) and
`--strategy.push_key` (h). `dagger` takes `--strategy.num_episodes`,
`--strategy.record_autonomous`, `--strategy.input_device` (keyboard/pedal) and
`--strategy.smooth_handover`. `episodic` takes `--strategy.reset_to_initial_position` and
`--strategy.smooth_leader_to_follower_handover`. DAgger keys: `Space` pauses, `Tab` starts or
stops a correction, `Enter` pushes, `ESC` stops.

## `lerobot-eval` (`EvalPipelineConfig`)

`--env.type` (required), `--policy.path`, `--eval.n_episodes` (50), `--eval.batch_size` (0 = auto),
`--eval.use_async_envs` (true), `--eval.max_episode_steps`, `--eval.recording` (false),
`--eval.recording_repo_id`, `--output_dir`, `--seed` (1000), `--rename_map`, `--trust_remote_code`
(EnvHub), and `--env.server.endpoint` for a served env. Per-env options such as `--env.task` and
`--env.task_ids` are listed by `lerobot-eval --env.type=<env> --help`.

## `lerobot-edit-dataset`

Top level: `--repo_id`, `--root`, `--new_repo_id`, `--new_root`, `--push_to_hub` (false), `--operation.type`.

| `--operation.type` | Operation flags |
|---|---|
| `delete_episodes` | `--operation.episode_indices "[0, 2, 5]"` |
| `split` | `--operation.splits '{"train": 0.8, "val": 0.2}'` or episode lists. Output: `<repo>_<split>` |
| `merge` | `--operation.repo_ids "['a/x', 'a/y']"`, `--operation.roots`, `concatenate_videos`, `concatenate_data` |
| `remove_feature` | `--operation.feature_names "['observation.images.top']"` |
| `modify_tasks` | `new_task`, `episode_tasks '{"0": "..."}'`, `task_replacements '{"old": "new"}'`. **In place** |
| `convert_image_to_video` | `output_dir`, `rgb_encoder.*`, `depth_encoder.*`, `episode_indices`, `num_workers` (4), `max_episodes_per_batch`, `max_frames_per_batch` |
| `recompute_stats` | `skip_image_video` (true), `relative_action`, `relative_exclude_joints`, `chunk_size` (50), `num_workers`, `overwrite` |
| `reencode_videos` | `rgb_encoder.*`, `depth_encoder.*`, `num_workers`, `encoder_threads`, `overwrite` |
| `info` | `show_features` (false) |

Adding features works only from Python (`lerobot.datasets.add_features` / `modify_features`).

## `lerobot-dataset-viz`

`--repo-id`, `--episode-index`, `--root`, `--mode` (`local` default, or `distant` to serve over gRPC), `--display-mode`
(`rerun`/`foxglove`), `--save`, `--output-dir`, `--web-port`, `--grpc-port`, `--host`,
`--tolerance-s`, `--batch-size`, `--num-workers`, `--display-compressed-images`, `--no-autoplay`.
