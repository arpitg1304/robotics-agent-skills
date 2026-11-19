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

**Use a sibling skill instead for:** choosing an IL algorithm, action representation or data
budget (`imitation-learning`). Auditing demo quality, outliers or label noise (`robot-data-quality`).
RL theory and reward design (`robot-reinforcement-learning`). Building MuJoCo or Isaac scenes
(`mujoco`, `isaac-sim`). Full scope list, pipeline diagram and draccus flag syntax:
[overview.md](references/overview.md).

**Flags (verified):** every CLI is a draccus dataclass. Nested fields are dotted flags
(`--robot.port`, `--policy.chunk_size`), `type` fields pick a registered subclass
(`--robot.type=so101_follower`), and dicts and lists are passed as YAML/JSON strings.
**`<command> --help` is the ground truth for the installed version.**

## Contracts Between Stages

Six things tie the stages together. When one changes, the next stage breaks:

| Contract | Set where | Breaks when |
|---|---|---|
| `--robot.id` / `--teleop.id` | every hardware command | a different id loads a different calibration file, or none |
| Calibration file | `$HF_LEROBOT_HOME/calibration/...` | you recalibrate after recording, so positions shift |
| Camera **names** | `--robot.cameras` dict keys | they become `observation.images.<name>`, and the policy expects exactly those |
| Camera resolution and `fps` | camera config, `--dataset.fps`, `--fps` | deployment differs from training, or the loop cannot keep the rate |
| Joint units (`use_degrees`) | robot/teleop config | dataset in one unit, robot in another |
| Normalisation stats | dataset `meta/stats.json`, copied into checkpoint processors | stats missing, or taken from the wrong dataset |

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
- Load [policies.md](references/policies.md) for the policy-to-extra table.

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
baudrate to EEPROM. Waveshare jumpers, gear ratios: [so101-hardware.md](references/so101-hardware.md).

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

**Commit or back up these JSON files next to your datasets.** A policy is only valid with the
calibration its data was recorded under. Dir overrides, reuse prompt: [so101-hardware.md](references/so101-hardware.md).

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
- Mount both cameras rigidly and light the scene evenly. A wrist camera mount that moves between
  sessions is a silent distribution shift.
- Camera fields, stable `/dev/v4l` names, USB bandwidth, RealSense: [so101-hardware.md](references/so101-hardware.md).
  Other robots (SO-100, bimanual, Koch, OMX, LeKiwi, OpenArm, ...): [robots.md](references/robots.md).

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
  `ESC`/`q` stops, encodes and uploads.
- The saved name is `${HF_USER}/so101_cube_to_bowl_YYYYMMDD_HHMMSS`. Find it with
  `ls ~/.cache/huggingface/lerobot/${HF_USER}` and use **that full name** for resume, train and
  replay.
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

## Dataset Format (v3.0)

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
Images are `observation.images.<camera_name>`. Bookkeeping columns, Python loading, visualiser,
`lerobot-edit-dataset` ops, v2.1 conversion and `finalize()`: [dataset-format.md](references/dataset-format.md).

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

Resume, checkpoints, devices, SmolVLA fine-tune (camera renaming) and ACT defaults:
[training.md](references/training.md). Policy choice: [policies.md](references/policies.md).

## Deployment on the Real Robot (`lerobot-rollout`)

```bash
# Quick look, no recording
lerobot-rollout --strategy.type=base \
  --policy.path=${HF_USER}/act_so101_cube_to_bowl \
  --robot.type=so101_follower --robot.port=$FOLLOWER_PORT --robot.id=$FOLLOWER_ID \
  --robot.cameras="$CAMERAS" \
  --task="Pick up the red cube and place it in the bowl" --duration=60 --display_data=true
```
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

Recorded `episodic` evaluation, other strategies, overrides, Python inference loop, processors:
[deployment.md](references/deployment.md). Sim eval and HIL-SERL:
[sim-and-rl.md](references/sim-and-rl.md).

## Common Failure Modes

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | Follower holds a constant offset from the leader, the gripper never fully closes, or calibration runs again on every command | Different `--robot.id`/`--teleop.id` between commands (each id is its own JSON), calibration not done at mid-range, or files deleted | Use one id everywhere (put it in env vars). `ls ~/.cache/huggingface/lerobot/calibration/*/*/`. Recalibrate *both* arms, moving to true mid-range |
| 2 | A policy that worked yesterday is now off by a few degrees per joint | Recalibrated after recording, so the new zero no longer matches the data | Treat calibration JSON as part of the dataset: back it up, never recalibrate mid-project. If you must, restore the old file (ENTER at the prompt writes it to the motors) or re-record |
| 4 | Cadence summary below target, jerky video, or duplicate frames | Slow cameras or USB saturation. `get_observation()` uses `read_latest()`, which returns a buffered frame up to 500 ms old, so a stalling camera **duplicates frames silently** | Fix it before recording 50 episodes: `fourcc: MJPG`, 640×480, separate USB ports, `--display_data=false`, `--dataset.streaming_encoding=true`. Check "Encoder queue full ... dropped N frame(s)" warnings |
| 5 | `FrameTimestampError: One or several query timestamps unexpectedly violate the tolerance` while training | A video stream is shorter than or out of sync with the parquet (dropped encoder frames, crash during encoding, edited file) | Open that episode in the visualiser. Remove it with `lerobot-edit-dataset --operation.type delete_episodes`. Raising `--tolerance_s` (default `1e-4`) only hides the problem |
| 11 | `Visual feature mismatch between policy and robot hardware` at rollout, or a feature mismatch at train | Camera names differ (`front` vs `top`, or `camera1` for SmolVLA base) | Reuse the exact `$CAMERAS` string. Otherwise add `--rename_map='{"observation.images.top": "observation.images.camera1"}'` (observation keys only) |
| 12 | Deployed policy behaves well for about a second, then drifts or overshoots | Open-loop chunk execution (ACT 100 steps), or sync inference stalls at chunk boundaries | `--policy.n_action_steps=20–50`, temporal ensembling, RTC, or remote inference. Check that `infer` in the rollout cadence fits the budget |
| 14 | `Incorrect status packet!` or intermittent read failures | Loose 3-pin daisy chain, weak supply, many joints moving at once | The SO configs retry `num_read_retries=2` times. Reseat the cables, check power. Raise `--robot.num_read_retries` only once the wiring is known good |
| 15 | `Repository not found` / dataset not found at train time | Used the unstamped `repo_id` | Use the `_YYYYMMDD_HHMMSS` name, or record with `--dataset.no_stamp=true` |

Row numbers match the full table in [troubleshooting.md](references/troubleshooting.md), which has
the other rows (units, crash recovery, v2.1/v3, q01/q99 stats, overheating, old flags, task
strings), more anti-patterns and the debugging playbook.

Anti-patterns:
- **Recording 50 episodes before checking one.** Record 2, replay one, open it in the visualiser,
  read the cadence summary, then scale up.
- **Changing camera placement, resolution or names between sessions of one dataset.** Resume
  rejects the feature change, and a policy cannot learn from a moving mount.

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

## Reference Files (load on demand)
- [so101-hardware.md](references/so101-hardware.md): when an SO-101 motor, calibration
  or camera step misbehaves (gear ratios, calibration dirs, camera fields, USB bandwidth).
- [troubleshooting.md](references/troubleshooting.md): when a symptom is not in the
  short table, or to debug from the bus up (18 failure rows, anti-patterns, playbook).
- [training.md](references/training.md): before resuming, fine-tuning SmolVLA,
  training off-GPU or on HF Jobs.
- [deployment.md](references/deployment.md): for recorded evaluation, other rollout
  strategies, a custom Python control loop, or processor/normalisation questions.
- [sim-and-rl.md](references/sim-and-rl.md): for `lerobot-eval` in simulation or HIL-SERL.
- [cli-reference.md](references/cli-reference.md): when building a non-trivial command (flag
  tables and defaults for the main CLIs).
- [robots.md](references/robots.md): for any robot other than a single SO-101, and for
  teleop devices.
- [dataset-format.md](references/dataset-format.md): for loading from Python, editing,
  merging, converting, streaming, transforms or custom loaders.
- [policies.md](references/policies.md): when choosing a policy or its extras, VRAM and
  policy-specific flags.
- [overview.md](references/overview.md): for the pipeline diagram, full scope list or
  draccus dotted-flag syntax.
