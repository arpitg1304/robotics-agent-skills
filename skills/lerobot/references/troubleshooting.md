# LeRobot Troubleshooting

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. This is the full failure table, the
anti-pattern list and the step-by-step debugging playbook. SKILL.md keeps only the most common
rows. Load this file when a symptom is not in SKILL.md's short table, or when you need to work
through a failure from the bus up.

## Failure Modes

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | Follower holds a constant offset from the leader, the gripper never fully closes, or calibration runs again on every command | Different `--robot.id`/`--teleop.id` between commands (each id is its own JSON), calibration not done at mid-range, or files deleted | Use one id everywhere (put it in env vars). `ls ~/.cache/huggingface/lerobot/calibration/*/*/`. Recalibrate *both* arms, moving to true mid-range |
| 2 | A policy that worked yesterday is now off by a few degrees per joint | Recalibrated after recording, so the new zero no longer matches the data | Treat calibration JSON as part of the dataset: back it up, never recalibrate mid-project. If you must, restore the old file (ENTER at the prompt writes it to the motors) or re-record |
| 3 | Replayed or deployed motions are scaled or flipped, e.g. a joint goes to ±100 instead of ±30° | Units mismatch: SO arms default to `use_degrees=true` (degrees, gripper 0–100). `use_degrees=false` gives −100..100. Koch/OMX default to `false`. Pre-PR #777 data used a different zero | Check `meta.stats["observation.state"]` min/max, then set `--robot.use_degrees`/`--teleop.use_degrees` to match the dataset. For pre-#777 data, see `examples/backward_compatibility/replay.py` |
| 4 | Cadence summary below target, jerky video, or duplicate frames | Slow cameras or USB saturation. `get_observation()` uses `read_latest()`, which returns a buffered frame up to 500 ms old, so a stalling camera **duplicates frames silently** | Fix it before recording 50 episodes: `fourcc: MJPG`, 640×480, separate USB ports, `--display_data=false`, `--dataset.streaming_encoding=true`. Check "Encoder queue full ... dropped N frame(s)" warnings |
| 5 | `FrameTimestampError: One or several query timestamps unexpectedly violate the tolerance` while training | A video stream is shorter than or out of sync with the parquet (dropped encoder frames, crash during encoding, edited file) | Open that episode in the visualiser. Remove it with `lerobot-edit-dataset --operation.type delete_episodes`. Raising `--tolerance_s` (default `1e-4`) only hides the problem |
| 6 | The dataset will not load after a crash, or parquet errors | Recording died before `finalize()` (power loss, kill -9). `lerobot-record` finalizes on normal exit and on Ctrl-C | Try `--resume=true` with the full stamped `repo_id`/`root`. If the files stay unreadable, restore the last pushed Hub revision or re-record the session. In custom scripts, call `finalize()` in a `finally:` block |
| 7 | `BackwardCompatibilityError ... in v2.1 format` | v2.1 dataset read by v3 code | `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=<id>`. If it is already converted on the Hub, delete the stale local cache |
| 8 | `ForwardCompatibilityError` | Dataset is newer than your LeRobot | Upgrade LeRobot |
| 9 | `ValueError: QUANTILES normalization mode requires q01 and q99 stats` (pi0.5) | Older dataset stats have no quantiles | `lerobot-edit-dataset --operation.type recompute_stats --operation.overwrite true`, then train with `--dataset.root` pointing at the result. Or pass `--policy.normalization_mapping='{"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}'` |
| 10 | Fine-tuned policy outputs saturated or tiny actions | Wrong stats: processors built from another dataset, a merged dataset with unrecomputed stats, or an old model with embedded normalisation | Load processors from the checkpoint (`pretrained_path=`). Run `recompute_stats` after merges. Migrate pre-#1452 models |
| 11 | `Visual feature mismatch between policy and robot hardware` at rollout, or a feature mismatch at train | Camera names differ (`front` vs `top`, or `camera1` for SmolVLA base) | Reuse the exact `$CAMERAS` string. Otherwise add `--rename_map='{"observation.images.top": "observation.images.camera1"}'` (observation keys only) |
| 12 | Deployed policy behaves well for about a second, then drifts or overshoots | Open-loop chunk execution (ACT 100 steps), or sync inference stalls at chunk boundaries | `--policy.n_action_steps=20–50`, temporal ensembling, RTC, or remote inference. Check that `infer` in the rollout cadence fits the budget |
| 13 | Motors hot, gripper weak after minutes, overload shutdowns | Gripper squeezing an object or a joint held against a hard stop. Only the gripper gets torque limits in `configure()` (Max_Torque_Limit 500, Protection_Current 250) | Don't grasp past contact during demos. Let the arm rest between sessions (`disable_torque_on_disconnect=true` is the default). Use a power supply that matches the STS3215 voltage variant (advice) |
| 14 | `Incorrect status packet!` or intermittent read failures | Loose 3-pin daisy chain, weak supply, many joints moving at once | The SO configs retry `num_read_retries=2` times. Reseat the cables, check power. Raise `--robot.num_read_retries` only once the wiring is known good |
| 15 | `Repository not found` / dataset not found at train time | Used the unstamped `repo_id` | Use the `_YYYYMMDD_HHMMSS` name, or record with `--dataset.no_stamp=true` |
| 16 | `'repo_id' argument missing` at train | `policy.push_to_hub` defaults to true | Add `--policy.repo_id=...` or `--policy.push_to_hub=false` |
| 17 | Old tutorial command fails on `--policy.path` with `lerobot-record`, `eval_` prefix, or `--wandb.*` | API moved in 0.6/0.7 | Use `lerobot-rollout`, `rollout_*` names and `--trackio.*` |
| 18 | Language policy ignores the instruction, or success drops | `--task` at deploy differs from `single_task` in the data | Copy the exact task string. `lerobot-edit-dataset --operation.type modify_tasks` fixes typos in the data |

## Anti-patterns

- **Recording 50 episodes before checking one.** Record 2, replay one, open it in the visualiser,
  read the cadence summary, then scale up.
- **Changing camera placement, resolution or names between sessions of one dataset.** Resume
  rejects the feature change, and a policy cannot learn from a moving mount.
- **Training on the default 100k steps** for a 45k-frame dataset with a schedule tuned for longer runs.
- **Hand-editing `meta/*.json`/parquet.** Use `lerobot-edit-dataset` or `lerobot.datasets.dataset_tools`.
  They keep offsets, stats and `info.json` consistent.
- **Copying flags from memory or from a blog post.** Run `<command> --help` for the installed version.

## Debugging Playbook

1. **Environment:** run `lerobot-info`. Paste it in issues, and check torch, torchcodec and
   ffmpeg pairing (torchcodec import failures show up here).
2. **Bus:** run `lerobot-find-port`, check the power LED, and run `lerobot-teleoperate` with no
   cameras. If that fails, the problem is motors or calibration, not ML.
3. **Calibration:** `cat ~/.cache/huggingface/lerobot/calibration/robots/so_follower/$FOLLOWER_ID.json`.
   No `range_min == range_max` is allowed (normalise raises "min and max are equal").
4. **Cameras:** run `lerobot-find-cameras opencv` and look at `outputs/captured_images`. Then
   teleoperate with `--display_data=true` and read the cadence `observe` share.
5. **Dataset:** `lerobot-edit-dataset --repo_id <id> --operation.type info --operation.show_features true`.
   Then open the visualiser and check `meta.stats` ranges against the expected units.
6. **Training:** watch `dataloading_s` against `update_s` in the logs. High dataloading means
   video decoding or workers; try `--num_workers` or `--dataset.image_transforms.backend=gpu`.
   Overfit 1–2 episodes first as a pipeline sanity check.
7. **Deployment:** compare `policy.config.input_features` with `robot.observation_features`. Run
   `lerobot-replay` on a training episode: if replay fails, the policy will too. Read the rollout
   status line (`infer` ms, `starved`).
