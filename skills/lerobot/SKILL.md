---
name: lerobot
description: >
  Hugging Face LeRobot end to end: install, hardware bring-up, dataset recording, training, and
  deploying robot learning policies. Use this skill when the user mentions LeRobot, lerobot-record,
  lerobot-train, lerobot-rollout, lerobot-teleoperate, lerobot-calibrate, lerobot-setup-motors,
  lerobot-find-port, lerobot-find-cameras, lerobot-eval, lerobot-edit-dataset, LeRobotDataset,
  SO-100, SO-101, SO-ARM100, Koch, LeKiwi, OMX, OpenArm, Feetech STS3215 servos, leader/follower
  teleoperation, calibration files, ACT, Diffusion Policy, SmolVLA, pi0/pi0.5, GR00T or X-VLA
  fine-tuning in LeRobot, robot datasets on the Hugging Face Hub, dataset v2.1 to v3.0 conversion,
  RTC or remote (async) inference, or HIL-SERL in LeRobot. Also trigger on LeRobot errors such as
  calibration mismatch, FrameTimestampError or timestamp tolerance violations, camera or feature
  name mismatch between dataset and policy, missing q01/q99 stats, and Feetech "Incorrect status
  packet".
---

# LeRobot Skill

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18.

That is the `main` branch. The newest PyPI release on that date was **0.7.0**. LeRobot changes
quickly between minor versions, so check what is installed before trusting any flag in this file
(see [Check what you have](#check-what-you-have-first)). This file marks what was read in the
source or docs as **verified**. It marks recommendations that come from field experience as
**advice**.

## When to Use This Skill
- Installing LeRobot, or picking extras for a robot, policy or simulator
- Bringing up an SO-100/SO-101 (or Koch, OMX, LeKiwi, OpenArm, ...) arm: ports, motor IDs,
  calibration, teleoperation, cameras
- Recording teleoperated demonstrations with `lerobot-record`, resuming a recording, pushing it to the Hub
- Inspecting, loading, visualising, editing, merging or converting `LeRobotDataset` data
- Training ACT / Diffusion / SmolVLA / pi0.5 / other policies with `lerobot-train`, and resuming runs
- Deploying a policy on the real robot with `lerobot-rollout` (sync, RTC, remote inference)
- Evaluating in simulation with `lerobot-eval`
- Debugging calibration, camera, timestamp, normalisation and feature-name failures

**Use a sibling skill instead for:** choosing an IL algorithm, action representation or data
budget (`imitation-learning`). Auditing demo quality, outliers or label noise (`robot-data-quality`).
RL theory and reward design (`robot-reinforcement-learning`). Building MuJoCo or Isaac scenes
(`mujoco`, `isaac-sim`).

## Mental Model

```
 HARDWARE                      DATA                           LEARNING                 DEPLOY
 so101_leader ─┐
               ├─ lerobot-record ─► LeRobotDataset v3 ─► lerobot-train ─► checkpoint ─► lerobot-rollout (robot)
 so101_follower┘   (teleop loop)    parquet + mp4 + meta/  (draccus CLI)   config.json     base|episodic|sentry|
 + cameras                          local cache <-> Hub                    model.safetensors highlight|dagger
 (calibration JSON per id)          ~/.cache/huggingface/lerobot/<repo>    pre/postprocessor ─► lerobot-eval (sim)
                                                                           train_config.json
```

Six things tie the stages together. When one changes, the next stage breaks:

| Contract | Set where | Breaks when |
|---|---|---|
| `--robot.id` / `--teleop.id` | every hardware command | a different id loads a different calibration file, or none |
| Calibration file | `$HF_LEROBOT_HOME/calibration/...` | you recalibrate after recording, so positions shift |
| Camera **names** | `--robot.cameras` dict keys | they become `observation.images.<name>`, and the policy expects exactly those |
| Camera resolution and `fps` | camera config, `--dataset.fps`, `--fps` | deployment differs from training, or the loop cannot keep the rate |
| Joint units (`use_degrees`) | robot/teleop config | dataset in one unit, robot in another |
| Normalisation stats | dataset `meta/stats.json`, copied into checkpoint processors | stats missing, or taken from the wrong dataset |

**Config system (verified):** every CLI is a [draccus](https://github.com/dlwh/draccus) dataclass.
Nested fields become dotted flags: `--robot.port`, `--dataset.repo_id`, `--policy.chunk_size`,
`--strategy.type`. `type` fields choose a registered subclass (`--robot.type=so101_follower`,
`--policy.type=act`). Dicts and lists are passed as YAML/JSON strings
(`--robot.cameras="{ top: {...} }"`, `--dataset.episodes='[0,1,2]'`). `lerobot-train` also takes
`--config_path=<file.json|yaml|Hub repo>`, and CLI flags override the file. **Every command
supports `--help`, and that is the ground truth for the installed version.**

## Check What You Have First

```bash
lerobot-info                                  # LeRobot, Python, torch, CUDA/MPS, ffmpeg, torchcodec versions
pip show lerobot | head -3                    # installed version
lerobot-record --help | grep -A2 streaming    # does this build have the flag I'm about to use?
# Registered types. A draccus ChoiceRegistry only knows the modules that have been imported, so import the CLI module first:
python -c "import lerobot.scripts.lerobot_record; from lerobot.robots import RobotConfig; print(sorted(RobotConfig.get_known_choices()))"
python -c "import lerobot.policies; from lerobot.configs.policies import PreTrainedConfig as P; print(sorted(P.get_known_choices()))"
```

Changes that make older tutorials wrong (verified in 0.7.x docs and source):
- **Policy deployment moved from `lerobot-record --policy.path` to `lerobot-rollout`.** In 0.7,
  `lerobot-record` needs a teleop and rejects dataset names starting with `eval_`. The cheat sheet
  says `lerobot-rollout` does not exist in v0.5.x and earlier.
- **Weights & Biases was replaced by Trackio.** Use `--trackio.enable=true` instead of
  `--wandb.enable=true`. Old checkpoints with a `wandb` section are migrated when loaded.
- **Python >= 3.12 is required** (`requires-python = ">=3.12"`).
- `lerobot-record` **appends `_YYYYMMDD_HHMMSS` to `--dataset.repo_id`** unless you pass `--dataset.no_stamp=true`.
- The dataset format is **v3.0**. A v2.1 dataset must be converted before 0.4+ can read it.
- Feetech and Dynamixel buses go through `rustypot` (extras `feetech` / `dynamixel`).

## Install (verified)

```bash
conda create -y -n lerobot python=3.12 && conda activate lerobot
conda install ffmpeg -c conda-forge            # TorchCodec video decoding; system ffmpeg is OK only with torch >= 2.10
pip install 'lerobot[core_scripts,training,feetech]'   # SO-100/101: record + train
# Policy extras as needed: smolvla | pi (pi0/pi0.5/pi0-FAST) | diffusion | groot | xvla | ...
pip install 'lerobot[smolvla]'
# From source (for reading or patching code):
git clone https://github.com/huggingface/lerobot && cd lerobot && pip install -e ".[core_scripts,training,feetech]"
```

- `core_scripts` = `dataset` + `hardware` + `viz` (record, replay, calibrate, Rerun/Foxglove).
  `training` = `dataset` + `accelerate` + `trackio`. `intelrealsense` is needed for RealSense cameras.
  Sim envs are extras too: `pusht`, `aloha`, `libero` (Linux only), `metaworld`.
- Linux torch wheels from PyPI are CUDA 13 builds and need NVIDIA driver >= 580.65. For an older
  driver, install torch from the `cu126` index first (see `docs/source/installation.mdx`).
- Apple Silicon: MPS needs macOS 15+. On macOS 14, LeRobot falls back to CPU.
- Load [references/policies.md](references/policies.md) for the policy-to-extra table.

## SO-101 Bring-Up (follower + leader, wrist + overhead camera)

Do these steps in order. Each step relies on the one before it. Keep the shared values in one
env file (advice) so every command uses the same ports, ids and camera dict:
```bash
# so101.env   (source it in every shell: `source so101.env`)
export FOLLOWER_PORT=/dev/ttyACM0  LEADER_PORT=/dev/ttyACM1        # fill in from step 1
export FOLLOWER_ID=so101_follower_01  LEADER_ID=so101_leader_01   # never change after calibrating
```

### 1. Ports
```bash
lerobot-find-port          # unplug the board when prompted; it prints that board's port
# macOS: /dev/tty.usbmodem5A7A0185331   Linux: /dev/ttyACM0
sudo usermod -aG dialout "$USER"   # Linux, then log out/in (docs show `sudo chmod 666 /dev/ttyACM0`, which resets on replug)
ls -l /dev/serial/by-id/           # Linux advice: stable names; ttyACM0/1 can swap after reboot or replug
```
**Advice:** put the `/dev/serial/by-id/...` path in your variables. Then the leader and follower
cannot swap ports without you noticing.

### 2. Motor IDs and baudrate (once per arm, when new or reused)
```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=$FOLLOWER_PORT
lerobot-setup-motors --teleop.type=so101_leader  --teleop.port=$LEADER_PORT
```
The script asks for one motor at a time, starting with `gripper` (ID 6) and ending at
`shoulder_pan` (ID 1). **Only that motor can be connected to the board.** It writes the ID and
baudrate to EEPROM. On a Waveshare board, set both jumpers to channel `B` (USB). The SO-101
leader uses mixed gear ratios: 1/191 for pan and elbow, 1/345 for shoulder lift, 1/147 for the
wrist and gripper. The follower uses 6 × STS3215 at 1/345. A wrong-geared motor on the leader
feels stiff or sags.

### 3. Calibration
```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID
lerobot-calibrate --teleop.type=so101_leader  --teleop.port=$LEADER_PORT  --teleop.id=$LEADER_ID
```
Procedure (verified from `SerialMotorsBus.record_calibration`):
1. Torque turns off. Move **every joint to the middle of its range** and press ENTER. That pose
   becomes the zero (half-turn homing offset).
2. Sweep each joint through its **full** range, then press ENTER. `wrist_roll` is treated as a
   full-turn motor and is not swept.
3. The result is written to the motors and saved as JSON:
   - follower: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<robot.id>.json`
   - leader: `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<teleop.id>.json`

   The directory comes from the class `name` (`so_follower`/`so_leader`), not from the `--robot.type`
   string. Override the root with `HF_LEROBOT_CALIBRATION` or `HF_LEROBOT_HOME`, or set
   `--robot.calibration_dir`.

If a file for that id exists, connect/calibrate asks: *"Press ENTER to use provided calibration file
... or type 'c' ... to run calibration"*. ENTER writes the saved file to the motors. When the motor
registers do not match the file, `connect()` starts calibration on its own. **Commit or back up
these JSON files next to your datasets.** A policy is only valid with the calibration its data was
recorded under.

### 4. Teleoperate (check calibration before adding cameras)
```bash
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID \
  --teleop.type=so101_leader  --teleop.port=$LEADER_PORT  --teleop.id=$LEADER_ID
```
Things to check (advice): the follower tracks the leader with no constant offset. The gripper opens
and closes fully. Nothing buzzes at rest. `lerobot-teleoperate` loops at `--fps=60` by default and
prints a cadence summary when it exits.

### 5. Cameras
```bash
lerobot-find-cameras opencv        # lists ids + default profile, saves frames to outputs/captured_images
lerobot-find-cameras realsense     # needs lerobot[intelrealsense]; macOS may need sudo and is unstable
```
Define the camera dict **once** and reuse it in every command. Names and resolution must stay the
same from recording to deployment:
```bash
# append to so101.env. Linux paths shown; on macOS use the integer ids from lerobot-find-cameras (index_or_path: 0 / 1)
export CAMERAS='{ wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, fourcc: MJPG}, top: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, fourcc: MJPG} }'
lerobot-teleoperate ... --robot.cameras="$CAMERAS" --display_data=true   # Rerun viewer
```
- OpenCV config fields (verified): `index_or_path` (int or path), `fps`, `width`, `height`,
  `color_mode` (rgb), `rotation` (0/90/180/-90), `warmup_s`, `fourcc`, `backend`. If the camera
  rejects the requested `fps`, `width` or `height`, connect raises `RuntimeError: ... failed to set
  fps=...` (or `capture_width`). A rejected `fourcc` only logs a warning ("Continuing with
  default format"), so check the log when you rely on MJPG for bandwidth. Use the exact
  fps/resolution profile that `lerobot-find-cameras` reports as supported.
- On Linux, `find_cameras` scans `/dev/video*`. **Advice:** a UVC camera often registers two
  nodes, so a second camera is usually `/dev/video2`, not `/dev/video1`. Use
  `/dev/v4l/by-id/...` or `/dev/v4l/by-path/...` so names survive a reboot. On macOS, indices
  move when an iPhone Continuity Camera or a new webcam shows up. Run `lerobot-find-cameras`
  again after any USB change.
- **Advice (USB bandwidth):** two uncompressed YUYV 640×480@30 streams on one hub can saturate it.
  Use `fourcc: MJPG`, put the cameras on separate root ports, and keep 640×480. The docs also
  call 640×480 at 30 fps enough for most tasks.
- RealSense: `{type: intelrealsense, serial_number_or_name: "233522074606", width: 640, height: 480, fps: 30, use_depth: true}`
  adds a `<name>_depth` feature.
- Mount both cameras rigidly and light the scene evenly. A wrist camera mount that moves between
  sessions is a silent distribution shift.

Load [references/robots.md](references/robots.md) for SO-100, bimanual SO, Koch, OMX, LeKiwi,
OpenArm and other robots.

## Recording Datasets

```bash
hf auth login                                   # write token
HF_USER=$(NO_COLOR=1 hf auth whoami | awk -F': *' 'NR==1 {print $2}')
lerobot-record \
  --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID \
  --robot.cameras="$CAMERAS" \
  --teleop.type=so101_leader --teleop.port=$LEADER_PORT --teleop.id=$LEADER_ID \
  --dataset.repo_id=${HF_USER}/so101_cube_to_bowl \
  --dataset.single_task="Pick up the red cube and place it in the bowl" \
  --dataset.num_episodes=50 --dataset.episode_time_s=30 --dataset.reset_time_s=15 \
  --dataset.fps=30 \
  --dataset.streaming_encoding=true --dataset.encoder_threads=2 \
  --display_data=true
```
- Keys (verified): `→`/`n` ends the episode or reset early. `←`/`r` re-records the episode.
  `ESC`/`q` stops, encodes and uploads. If no global key backend exists (Wayland, SSH, macOS
  without Accessibility), the same keys are read from the focused terminal.
- The saved name is `${HF_USER}/so101_cube_to_bowl_YYYYMMDD_HHMMSS`. Find it with
  `ls ~/.cache/huggingface/lerobot/${HF_USER}` and use **that full name** for resume, train and
  replay.
- Defaults: `fps=30`, `episode_time_s=60`, `reset_time_s=60`, `num_episodes=50`, `video=true`,
  `push_to_hub=true`, `streaming_encoding=false`. Use `--dataset.private=true` and
  `--dataset.tags='["so101"]'` as needed.
- **Resume** an interrupted session or add episodes. Here `num_episodes` means *additional*
  episodes:
  ```bash
  lerobot-record ...same robot/teleop/cameras... --resume=true \
    --dataset.repo_id=${HF_USER}/so101_cube_to_bowl_20261118_143012 \
    --dataset.root=$HOME/.cache/huggingface/lerobot/${HF_USER}/so101_cube_to_bowl_20261118_143012 \
    --dataset.num_episodes=10
  ```
  On resume, robot_type, fps and the feature dict must match the existing dataset. A camera rename
  or resolution change fails `sanity_check_dataset_robot_compatibility`.
- **Read the cadence summary** printed at the end of each episode and of the session. Frame
  timestamps come from the frame index, so a loop that ran at 25 Hz still produces a dataset that
  *claims* 30 Hz. If `observe` takes most of the work time, the cameras are the bottleneck.
  "pacing headroom" near zero means the loop is saturated.
- Replay to check repeatability:
  `lerobot-replay --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID --dataset.repo_id=<full_name> --dataset.episode=0`

## Dataset Format and Tools (v3.0)

```
<root>/                        # default $HF_LEROBOT_HOME/<repo_id> = ~/.cache/huggingface/lerobot/<repo_id>
  meta/info.json               # codebase_version "v3.0", fps, robot_type, features (dtype/shape/names), path templates
  meta/stats.json              # mean/std/min/max (+ quantiles like q01/q99) per feature -> normalisation
  meta/tasks.parquet           # task string <-> task_index
  meta/episodes/chunk-000/file-000.parquet   # per-episode length, tasks, offsets into shared files, per-episode stats
  data/chunk-000/file-000.parquet            # many episodes per file: action, observation.state, timestamp, indices
  videos/observation.images.top/chunk-000/file-000.mp4   # many episodes per mp4, one stream per camera
```
SO-101 features (verified pattern): `action` and `observation.state` are float32 vectors with
`names` `[shoulder_pan.pos, shoulder_lift.pos, elbow_flex.pos, wrist_flex.pos, wrist_roll.pos, gripper.pos]`.
Images are `observation.images.<camera_name>`. Bookkeeping columns are `timestamp`, `frame_index`,
`episode_index`, `index` and `task_index`.

```python
from lerobot.datasets import LeRobotDataset, LeRobotDatasetMetadata

meta = LeRobotDatasetMetadata("user/so101_cube_to_bowl_20261118_143012")   # metadata only, cheap
print(meta.fps, meta.robot_type, meta.total_episodes, meta.total_frames, meta.camera_keys)
print(meta.features["action"]["names"], meta.stats["action"]["mean"])

ds = LeRobotDataset(meta.repo_id, episodes=[0, 1],
                    delta_timestamps={"action": [i / meta.fps for i in range(50)]})  # 50-step action chunks
item = ds[0]          # dict of tensors; images (C,H,W) float in [0,1]; item["action"].shape == (50, 6)
```
- Visualise in the browser at `https://huggingface.co/spaces/lerobot/visualize_dataset?path=/<repo_id>/episode_0`.
  Locally, run `lerobot-dataset-viz --repo-id <id> --episode-index 0`, adding
  `--root <dir> --mode local` for a local dataset or `--display-mode foxglove` for Foxglove.
- Edit with `lerobot-edit-dataset --repo_id <id> --operation.type <op>`. The ops are
  `delete_episodes`, `split`, `merge`, `remove_feature`, `modify_tasks`,
  `convert_image_to_video`, `recompute_stats`, `reencode_videos` and `info`. Add `--new_repo_id`
  to keep the original, and `--push_to_hub true` to upload. `modify_tasks` always works in
  place. Merging needs identical features.
- Convert v2.1 to v3.0 with `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=<id>`.
  It also takes `--root`, `--push-to-hub=false` and `--force-conversion`.
- If you write datasets from Python, call **`dataset.finalize()` before `push_to_hub()`**.
  Without it the parquet footers are never written and the files are corrupt.

Load [references/dataset-format.md](references/dataset-format.md) for `info.json` fields,
streaming, image transforms, editing recipes and the Python editing API.

## Training

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/so101_cube_to_bowl_20261118_143012 \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=${HF_USER}/act_so101_cube_to_bowl \
  --output_dir=outputs/train/act_so101_cube_to_bowl \
  --job_name=act_so101_cube_to_bowl \
  --batch_size=8 --steps=40000 --save_freq=5000 \
  --trackio.enable=true
```
- **Size `--steps` from epochs, not the 100k default.** The docs say 5–10 epochs is typical.
  50 episodes × 30 s × 30 fps = 45k frames. At batch 8 that is about 5.6k steps per epoch, so
  28k–56k steps. When you shorten the run, also shorten `--save_freq`. For policies with a decay
  schedule (smolvla, pi0/pi05, ...), set `--policy.scheduler_decay_steps≈--steps`. Otherwise the
  LR never decays.
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

Load [references/policies.md](references/policies.md) for the policy list, extras, VRAM groups and
policy-specific gotchas. Load [references/cli-reference.md](references/cli-reference.md) for
the full flag tables.

## Deployment on the Real Robot (`lerobot-rollout`)

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
- **Action chunks:** ACT runs `n_action_steps` actions open-loop. At the default 100 steps and
  30 fps that is 3.3 s blind. For reactive tasks, lower it, or use temporal ensembling with
  `--policy.temporal_ensemble_coeff=0.01 --policy.n_action_steps=1`, which queries every step.
  For slow VLAs (pi0/pi0.5/SmolVLA), use `--inference.type=rtc` to compute the next chunk while
  the current one executes. You can also move the model to a GPU server with
  `lerobot-policy-server` + `--inference.type=remote`. That needs `lerobot[remote]` on both
  machines, and ACT must have temporal ensembling disabled. Read `docs/source/remote_inference.mdx`
  before configuring it.
- **Advice:** for the first run of a new policy, set `--robot.max_relative_target=10`. It caps how
  far any joint target can jump in one step (units follow `use_degrees`). Keep a hand on the
  follower's power switch.

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

## Failure Modes and Anti-Patterns

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

Anti-patterns:
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

## Experiment Checklist

- [ ] `lerobot-info` saved. LeRobot version and commit recorded alongside each dataset and checkpoint
- [ ] Ports use stable names. `FOLLOWER_ID`/`LEADER_ID`/`CAMERAS` defined once in an env file
- [ ] Both arms calibrated at true mid-range. Calibration JSONs backed up with the dataset
- [ ] Teleop has no offset. Gripper opens and closes fully. No buzzing or heat after 5 minutes
- [ ] Cameras rigidly mounted, even lighting, 640×480@30 with MJPG, cadence ≥ 99% of target
- [ ] Task string written down and reused verbatim. Episodes visualised. Bad ones deleted
- [ ] `--steps` from 5–10 epochs, `save_freq` and scheduler decay adjusted. Trackio enabled
- [ ] First deployment uses `max_relative_target`, `--duration`, and a hand on the power switch
- [ ] Evaluation recorded with `episodic`, with a fixed number of trials and a success criterion written down beforehand

## Related Skills
- `imitation-learning`: algorithm choice (ACT vs diffusion vs VLA), action chunking theory, data
  scaling, evaluation protocol
- `robot-data-quality`: auditing recorded episodes, outliers, sync and label QA before training
- `robot-reinforcement-learning`: SAC/HIL-SERL concepts, reward classifiers, safety during online RL
- `mujoco`, `isaac-sim`: building simulated SO-101 scenes and envs (LeRobot EnvHub/IsaacLab Arena consume them)
- `robot-bringup`, `robotics-testing`: general hardware bring-up discipline and test strategy
- `robot-app-harness`: closed-loop development of scripted robot apps against a simulated SO-101
  (ARMATURE). It has no LeRobot hardware backend yet, so it does not drive a real arm

## References (load on demand)
- [references/cli-reference.md](references/cli-reference.md): flag tables for record, teleoperate,
  replay, train, rollout, eval, edit-dataset and dataset-viz. Load it when building a non-trivial command.
- [references/robots.md](references/robots.md): per-robot notes (SO-100 vs SO-101, bimanual, Koch,
  OMX, LeKiwi, OpenArm, HopeJr, Reachy2, Unitree G1, YAM, reBot, EarthRover) and teleop devices.
  Load it for any robot other than a single SO-101.
- [references/dataset-format.md](references/dataset-format.md): v3 internals, Python API,
  streaming, transforms, editing and merging recipes. Load it for dataset surgery or custom loaders.
- [references/policies.md](references/policies.md): registered policy types, extras, VRAM and
  policy-specific flags. Load it when choosing or fine-tuning a policy.
