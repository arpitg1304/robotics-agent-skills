# Demonstrations, Mimic, GR00T and the ROS 2 Bridge

Checked against Isaac Lab `v3.0.0` (`docs/source/features/imitation-learning/*.rst`,
`scripts/imitation_learning/`, `scripts/tools/`) and the Isaac Sim 6.2.0 ROS 2 install page,
2026-11-18. Not executed. Script flags change between releases, so run `--help` on each script
before scripting a pipeline.

## Quick pipeline (moved from SKILL.md)

```bash
# Teleop-record -> annotate subtasks -> multiply with Mimic (keyboard | spacemouse; XR: --xr on IK-Abs)
uv run --extra teleop,isaacsim isaaclab teleop record --task IsaacContrib-Stack-Cube-Franka-IK-Rel \
    --viz kit --teleop_device spacemouse --dataset_file ./datasets/dataset.hdf5 --num_demos 10
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --viz kit --auto \
    --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --num_envs 300 --generation_num_trials 1000 \
    --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5
```

- **Mimic needs task-space (end-effector) actions.** It replays subtask segments under rigid
  transforms relative to a reference object, so convert joint-space data with FK first.
  Mimic task IDs still end in `-v0`.
- Only successful generated trials are kept. **Heuristic:** a success/attempt ratio below about
  10% usually means bad annotations or subtasks that are too long.
- **Format:** robomimic-style HDF5 (`data/demo_<i>/actions`, `obs/...`, `attrs["success"]`).
  Train BC with `scripts/imitation_learning/robomimic/train.py`. Merge and convert with
  `scripts/tools/` (`merge_hdf5_datasets.py`, `hdf5_to_mp4.py`, `mp4_to_hdf5.py` for Cosmos
  augmentation).
- **GR00T N1.5:** the humanoid locomanipulation SDG workflow fine-tunes it in a separate uv env
  and converts HDF5 to LeRobot format. RLinf (`--rl_library rlinf`) does RL post-training of
  VLAs.
- Full pipeline (replay, visuomotor, SkillGen, XR, GR00T steps): the sections below. For
  dataset QA use `robot-data-quality`, for conversion `lerobot`, and for policy choice
  `imitation-learning`.

## Teleoperation devices

- `--teleop_device keyboard | spacemouse`, available through `isaaclab teleop run|record|replay`
  and `scripts/environments/teleoperation/teleop_se3_agent.py`.
- Gamepad and Haply devices live under `isaaclab.devices` (`gamepad`, `haply`).
- XR hand tracking (Meta Quest, Pico, Apple Vision Pro through CloudXR) goes through
  `isaaclab_teleop` (`IsaacTeleopDevice`/`IsaacTeleopCfg`, which replace the deprecated
  `OpenXRDevice`). Use the **absolute** IK task variant and pass `--xr` instead of
  `--teleop_device`.
- CloudXR runs as an extra Compose service:
  `./docker/container.py start --files docker-compose.cloudxr-runtime.patch.yaml --env-files .env.cloudxr-runtime`.
- `--sensitivity` scales SE(3) device deltas.

## Franka stacking pipeline (the reference workflow)

```bash
# Inspect the scene
uv run --extra isaacsim isaaclab zero_agent --task IsaacContrib-Stack-Cube-Franka-IK-Rel --viz kit --num_envs 1

# Live teleop (no recording)
uv run --extra teleop,isaacsim isaaclab teleop run --task IsaacContrib-Stack-Cube-Franka-IK-Rel \
    --viz kit --num_envs 1 --sensitivity 4 --teleop_device keyboard

# Record human demos (only successful episodes are exported by default)
uv run --extra teleop,isaacsim isaaclab teleop record --task IsaacContrib-Stack-Cube-Franka-IK-Rel \
    --viz kit --dataset_file ./datasets/dataset.hdf5 --num_demos 10 --teleop_device spacemouse
# XR variant
uv run --extra teleop,isaacsim isaaclab teleop record --task IsaacContrib-Stack-Cube-Franka-IK-Abs \
    --viz kit --dataset_file ./datasets/dataset.hdf5 --num_demos 10 --xr

# Replay to check the recording reproduces
uv run --extra teleop,isaacsim isaaclab teleop replay --task IsaacContrib-Stack-Cube-Franka-IK-Rel \
    --viz kit --num_envs 1 --reset_sim_buffer_each_episode --dataset_file ./datasets/dataset.hdf5

# Annotate subtask end points (--auto uses the task's subtask signals; omit for manual annotation)
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --viz kit --auto \
    --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5

# Generate: small visual check first, then headless at scale
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --viz kit --num_envs 20 --generation_num_trials 10 \
    --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset_small.hdf5
uv run --extra isaacsim,mimic python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --num_envs 1000 --generation_num_trials 1000 \
    --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5

# Train and evaluate a robomimic BC policy (robomimic comes with the source-checkout mimic extra)
uv run --extra isaacsim,mimic python scripts/imitation_learning/robomimic/train.py \
    --task IsaacContrib-Stack-Cube-Franka-IK-Rel --algo bc --epochs 1000 --dataset ./datasets/generated_dataset.hdf5
uv run --extra isaacsim,mimic python scripts/imitation_learning/robomimic/play.py \
    --task IsaacContrib-Stack-Cube-Franka-IK-Rel --viz kit --num_rollouts 50 --checkpoint /PATH/TO/model.pth
```

Other `generate_dataset.py` flags:

- `--max_num_failures` stops a run that keeps failing.
- `--pause_subtask` lets you step through subtasks while debugging.
- `--use_skillgen` adds motion-planned transitions (SkillGen; not validated on DGX Spark).
- `--disable_dataset_compression`.

`consolidated_demo.py` interleaves teleop and generation in one session.

Visuomotor variants (`...-Visuomotor`, `...-Visuomotor-Cosmos`) add cameras.

The Cosmos visual-augmentation round trip:

1. `scripts/tools/hdf5_to_mp4.py`
2. Cosmos Transfer
3. `scripts/tools/mp4_to_hdf5.py`
4. `scripts/tools/merge_hdf5_datasets.py --input_files a.hdf5 b.hdf5 --output_file merged.hdf5`

Robustness evaluation: `scripts/imitation_learning/robomimic/robust_eval.py`.

## What Mimic assumes

- **Task-space actions.** Generation applies rigid transforms to end-effector targets relative
  to each subtask's reference object, so joint-space action datasets must first be converted
  with forward kinematics. Use IK-Rel/IK-Abs or OSC action terms.
- **Subtasks tied to one object frame each.** A new subtask starts when the reference object
  changes. Good annotations end a subtask at a stable, unambiguous moment (grasp closed,
  object lifted).
- **Success is checked by the env's termination/success terms.** Failed trials are discarded,
  so a weak success check silently admits bad demos.
- Multi-end-effector embodiments (humanoids) are supported through the Mimic env config, but
  each arm's subtasks are defined separately.
- **Heuristics:** a low success/attempt ratio points to over-long subtasks, annotation errors,
  or object placements outside the range the source demos can reach. Contact-rich insertion
  with tight tolerances transfers poorly by pure transformation.

## Dataset format

`isaaclab/utils/datasets/hdf5_dataset_file_handler.py` writes robomimic-style HDF5:

```
/ (attrs: format_version)
└── data (attrs: total, env_args = JSON with env_name, ...)
    ├── demo_0 (attrs: num_samples, seed, success)
    │   ├── actions            (T, action_dim)
    │   ├── obs/<term>         (T, ...)
    │   ├── states/...         articulation / rigid object states for replay
    │   └── initial_state/...
    └── demo_1 ...
```

Inspect it with `h5py` before training. Check for `success` flags, equal `T` across keys,
NaNs, and action ranges. `robot-data-quality` covers the QA in depth. `lerobot` covers
converting to LeRobot datasets.

## GR00T and VLA workflows

- **Humanoid locomanipulation SDG:** `scripts/imitation_learning/locomanipulation_sdg/` generates
  G1 data. Its `gr00t/` folder fine-tunes **GR00T N1.5**, pinned in the docs to Isaac-GR00T
  commit `4af2b622...`.
- Put GR00T in a **separate uv venv**, because its NumPy/Transformers pins conflict with Isaac
  Lab. Install with `uv pip install --no-config --python "$GR00T_PYTHON" ...` and run with
  `uv run --no-project --python "$GR00T_PYTHON" ...`. Rollout talks to a policy server on
  localhost (`serve_policy.py`, `rollout_policy.py`).
- **Convert to LeRobot format** for GR00T with `gr00t/convert_dataset.py <dir_of_sdg_hdf5>
  <output_dir>`. The input dir must contain only the SDG HDF5 files.
- **RLinf** (`--extra rlinf`, `--rl_library rlinf`) is RL post-training of VLAs, for example
  `--config_name isaaclab_ppo_gr00t_assemble_trocar --model_path <base VLA> --checkpoint <rl ckpt>`.
  `--model_path` is the base model, and `--checkpoint` is the RL-finetuned weights.
- These pins are release-specific. Re-read `docs/source/features/imitation-learning/humanoids_imitation.rst`
  and `docs/source/experimental-features/rlinf_vla_posttraining.rst` in your checkout.

## ROS 2 bridge (Isaac Sim 6.2)

Summary (moved from SKILL.md):

- `isaacsim.ros2.bridge` in Isaac Sim 6.2 supports **Jazzy on Ubuntu 24.04 (recommended)** and
  **Humble on 22.04**. Humble on 24.04 is unsupported.
- Libraries: source your ROS 2 before launching, or leave `ROS_DISTRO` unset to use the bundled
  libraries.
- Match `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on both sides. Containers and remote hosts
  need a UDP-only FastDDS profile.
- OmniGraph publishers and subscribers (`ROS2PublishClock`, `ROS2PublishJointState`,
  `ROS2SubscribeJointState`, `ROS2CameraHelper`, ...) run **only while the sim is playing**.
  Publish `/clock` and use `use_sim_time:=true`.
- Deploy learned policies as an ONNX export inside a ROS 2 node at the training control rate,
  and use the bridge for single-robot integration tests, not for parallel RL. Details below;
  ROS side: `ros2`, `docker-ros2-development`.

Details:

- Extensions: `isaacsim.ros2.bridge`, which depends on `isaacsim.ros2.core`, `.nodes`, `.ui`
  and `.examples`. Related extensions are `isaacsim.ros2.control`, `isaacsim.ros2.sim_control`,
  `isaacsim.ros2.tf_viewer` and `isaacsim.ros2.urdf`.
- Distros:
  - Jazzy on Ubuntu 24.04 is recommended. Humble on Ubuntu 22.04 is supported.
  - Humble on 24.04 is **not** supported. Other natively installed distros are experimental.
  - On Windows 11, use Jazzy via Pixi. Humble via WSL2 is deprecated.
- Bundled libraries are under `exts/isaacsim.ros2.core/<distro>/lib` (with Cyclone DDS; Zenoh is
  not bundled on Linux).
  - **Option A:** source your system ROS 2 in the terminal before `isaac-sim.sh`.
  - **Option B:** use a clean terminal with `ROS_DISTRO` unset, and the launcher selects the
    bundled libraries.
  - Setting `ROS_DISTRO` without the matching `LD_LIBRARY_PATH` breaks automatic setup.
- Middleware: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (examples), `rmw_cyclonedds_cpp`, or
  `rmw_zenoh_cpp`. All participants share `ROS_DOMAIN_ID`. Across containers, WSL or hosts, point
  `FASTRTPS_DEFAULT_PROFILES_FILE` at a UDP-only profile; shared memory does not cross those
  boundaries.
- Graphs: publishers and subscribers are OmniGraph (Action Graph) nodes, active only while the
  timeline is playing.
  - Typical graph: `OnPlaybackTick` → `ROS2PublishClock`, `ROS2PublishJointState`,
    `ROS2SubscribeJointState` → articulation controller, `ROS2PublishTransformTree`, camera
    helpers.
  - Build graphs from Tools > Robotics > ROS 2 OmniGraphs, or in Python with `og.Controller.edit`
    after enabling the extension.
- Time: publish `/clock` and run ROS nodes with `use_sim_time:=true`. Otherwise TF and
  controllers mix wall time and sim time.
- Smoke test: Tools > Robotics > ROS 2 OmniGraphs > Clock, press Play, then
  `ros2 topic echo /clock` in a sourced terminal.
- `isaacsim.ros2.control` integrates `ros2_control`. Check its extension docs for the exact
  hardware-interface setup in your version.
- Isaac Lab Docker: the `ros2` profile builds the base image plus ROS 2 Humble
  (`ROS2_APT_PACKAGE` in `.env.ros2`, default `ros-base`). It defaults to FastRTPS and supports
  CycloneDDS; XML profiles are in `docker/.ros`.
- Pattern for learned policies: train in Isaac Lab without ROS. Export `policy.onnx`. Run it in
  a ROS 2 node at the training control rate, with the same observation normalisation and joint
  ordering. Then test against Isaac Sim through the bridge before hardware. The bridge is for
  single-robot integration testing, not for thousands of parallel envs. See `ros2` and
  `docker-ros2-development` for the ROS side.
