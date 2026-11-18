# LeRobotDataset v3.0 Internals and Recipes

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Sources:
`docs/source/lerobot-dataset-v3.mdx`, `using_dataset_tools.mdx`, `porting_datasets_v3.mdx`,
`src/lerobot/datasets/{utils,dataset_metadata,lerobot_dataset,dataset_writer,dataset_tools}.py`.
This file covers the format and the tooling. For *auditing* quality (outliers, sync, label noise),
use the `robot-data-quality` skill.

## Layout

```
<root>/                                   # $HF_LEROBOT_HOME/<repo_id> unless --dataset.root / root=
├── meta/
│   ├── info.json                         # schema + counters + path templates
│   ├── stats.json                        # global stats per feature
│   ├── tasks.parquet                     # task string ↔ task_index
│   └── episodes/chunk-000/file-000.parquet   # one row per episode: length, tasks, data/video offsets, per-episode stats
├── data/chunk-000/file-000.parquet       # frames of many episodes
└── videos/<video_key>/chunk-000/file-000.mp4  # e.g. videos/observation.images.wrist/...
```
Path templates (verified constants): `data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet`,
`videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4`, and
`meta/episodes/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet`. Default limits: 1000 files
per chunk, about 100 MB per data file, about 200 MB per video file. Image (non-video) datasets
use `images/<key>/episode-XXXXXX/frame-XXXXXX.png`.

**v2.1 difference:** one parquet and one mp4 *per episode*, plus `meta/episodes.jsonl` and
`episodes_stats.jsonl`. v3 packs many episodes per file and finds episode boundaries through
`meta/episodes/*`. **Never locate an episode by filename in v3.**

## `meta/info.json` fields (`DatasetInfo`)
`codebase_version` ("v3.0"), `fps`, `features`, `total_episodes`, `total_frames`, `total_tasks`,
`chunks_size`, `data_files_size_in_mb`, `video_files_size_in_mb`, `data_path`, `video_path`,
`storage_format` (None = parquet/mp4, "lance" = Lance backend), `robot_type`, `splits`, `tools`.

Each feature entry looks like `{"dtype": "float32"|"video"|"image"|..., "shape": [...], "names": [...] | null, "info": {...}}`.
Video features carry codec and pixel-format info. Default columns that every dataset has:
`timestamp` (float32), `frame_index`, `episode_index`, `index`, `task_index` (int64).

**Timestamps are synthetic:** the writer sets `timestamp = frame_index / fps`. It does not use
wall-clock time. If the recording loop ran slower than `fps`, the data still claims `fps`. Read
the cadence summary at record time, because the dataset cannot tell you afterwards.

## Stats
`meta/stats.json` holds `mean`, `std`, `min`, `max` (and quantiles such as `q01`/`q99` when
computed) for every numeric and visual feature. Policies copy these into their normaliser at
training time, and the checkpoint keeps its own copy.
- Recompute after editing or merging, or when a policy needs quantiles:
  `lerobot-edit-dataset --repo_id <id> --operation.type recompute_stats --operation.overwrite true`.
  `skip_image_video=true` is the default, so image stats are kept.
- Recording, resuming and merging build quantiles from per-episode summaries, so they are a
  conservative envelope. For histogram estimates over the whole dataset, run
  `python src/lerobot/scripts/augment_dataset_quantile_stats.py --repo-id=<id> --overwrite --skip-images`
  (from a source checkout).
- `--dataset.use_imagenet_stats=true` (the training default) replaces image stats with ImageNet
  mean/std.

## Loading

```python
from lerobot.datasets import LeRobotDataset, LeRobotDatasetMetadata, StreamingLeRobotDataset

meta = LeRobotDatasetMetadata("user/ds")          # downloads meta/ only
meta.features, meta.stats, meta.fps, meta.robot_type
meta.total_episodes, meta.total_frames, meta.camera_keys, meta.video_keys

ds = LeRobotDataset(
    "user/ds",
    root=None,                 # local dir. Default $HF_LEROBOT_HOME/user/ds
    episodes=[0, 1, 2],        # subset (only those files are downloaded)
    delta_timestamps={         # seconds relative to t; must be multiples of 1/fps (within tolerance_s)
        "observation.images.top": [-0.1, 0.0],
        "action": [i / meta.fps for i in range(50)],
    },
    tolerance_s=1e-4,
    revision=None,             # pin a Hub revision for reproducibility
    video_backend=None,        # torchcodec by default, pyav as fallback
)
len(ds), ds.num_episodes, ds.fps
item = ds[123]                 # images (C,H,W) float32 in [0,1]; *_is_pad masks for padded deltas
cols = ds.select_columns("action")   # Hugging Face datasets-style column view (used by replay)

stream = StreamingLeRobotDataset("user/ds")       # no full download
```
- `delta_timestamps` values that are not multiples of `1/fps` fail `check_delta_timestamps`.
- Querying past an episode edge pads and returns `<key>_is_pad` masks. Policies use these.
- A `FrameTimestampError` at decode time means the mp4 has no frame within `tolerance_s` of the
  requested timestamp. Treat it as a corrupt or dropped-frame episode. Do not widen the tolerance
  to hide it.

## Writing from Python

```python
from lerobot.datasets import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features

features = {**hw_to_dataset_features(robot.action_features, "action"),
            **hw_to_dataset_features(robot.observation_features, "observation")}
ds = LeRobotDataset.create(repo_id="user/my_ds", fps=30, features=features,
                           robot_type=robot.name, use_videos=True, image_writer_threads=4)
try:
    for ep in range(num_episodes):
        for _ in range(steps):
            frame = {...}                 # every user feature + "task"; NOT timestamp/frame_index
            frame["task"] = "Pick up the red cube"
            ds.add_frame(frame)
        ds.save_episode()                 # or ds.clear_episode_buffer() to discard
finally:
    ds.finalize()                         # closes parquet writers; without it files are corrupt
ds.push_to_hub()
```
`LeRobotDataset.resume(repo_id, root=...)` reopens a dataset to append episodes. The
`lerobot-record` API example in `docs/source/il_robots.mdx` shows the full loop with keyboard
events and `record_loop`.

## Editing (Python API, `lerobot.datasets`)

```python
from lerobot.datasets import (LeRobotDataset, delete_episodes, split_dataset, merge_datasets,
                              add_features, remove_feature, modify_features, modify_tasks, recompute_stats)
ds = LeRobotDataset("user/ds")
clean = delete_episodes(ds, episode_indices=[3, 17], repo_id="user/ds_clean")
splits = split_dataset(ds, splits={"train": 0.9, "val": 0.1})
merged = merge_datasets([ds_a, ds_b], output_repo_id="user/ds_merged")   # identical features required
no_cam = remove_feature(ds, feature_names="observation.images.top", repo_id="user/ds_no_top")
```
`add_features(ds, features={"reward": (array_or_fn, {"dtype": "float32", "shape": (1,), "names": None})}, repo_id=...)`
adds columns. That is only available from Python, not from the CLI. The full example is
`examples/dataset/use_dataset_tools.py`.

Merging rules (advice plus verified constraints): features must match exactly, including the
camera names and shapes, the joint `names` and the dtype. fps and units should match too (degrees
vs range). Recompute stats after merging. Task strings are merged through `tasks.parquet`.

## Conversion and compatibility
- v2.1 → v3.0: `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=<id>`. Options:
  `--root <dir>` (local, converts in place), `--push-to-hub=false`, `--branch`,
  `--data-file-size-in-mb`, `--video-file-size-in-mb`, `--force-conversion`. It pushes to `main`
  and tags `v3.0`.
- Error text: a v2.1 dataset read by v3 code raises `BackwardCompatibilityError`, whose message
  includes the convert command. A dataset newer than the code raises `ForwardCompatibilityError`
  (upgrade LeRobot). A missing version tag gives a message telling you to `create_tag` the
  `codebase_version`.
- If the Hub copy is already converted but a stale v2.1 cache is still local, delete
  `$HF_LEROBOT_HOME/<repo_id>` (and LeRobot's revision cache `$HF_LEROBOT_HOME/hub`) and reload.
- Porting non-LeRobot datasets (e.g. DROID, OXE) is covered in `docs/source/porting_datasets_v3.mdx`
  and `examples/port_datasets/`.

## Video encoding
- Defaults: `libsvtav1`, `yuv420p`, GOP `g=2`. `--dataset.rgb_encoder.vcodec=auto` picks a
  hardware encoder if one is available (e.g. `h264_videotoolbox` on macOS).
- `--dataset.streaming_encoding=true` encodes during capture. If the CPU cannot keep up, the
  per-camera queue (`encoder_queue_maxsize=30`) fills and **frames are dropped** with
  `Encoder queue full for <camera>, dropped N frame(s)`. Watch for it: dropped frames make
  videos shorter than the parquet.
- Re-encode an existing dataset with `lerobot-edit-dataset --operation.type reencode_videos --operation.rgb_encoder.vcodec h264 ...`.
- Depth (RealSense `use_depth: true`) is quantised to 12-bit codes and stored as its own video.
  `--dataset.depth_encoder.{depth_min,depth_max,shift,use_log}` set the quantisation, which is
  kept in the metadata for dequantisation.

## Image augmentation (training time only)
`--dataset.image_transforms.enable=true` turns it on. Configure it with `max_num_transforms` (3),
`random_order` (false), and `tfs` (a dict of `{type, weight, kwargs}`, where `type` is any
torchvision v2 class or a LeRobot class such as `SharpnessJitter`, `GaussianNoise`,
`MotionBlur`, `RandomShadow`). `backend=gpu` augments on the policy device. Preview with
`lerobot-imgtransform-viz`. Raw frames are never modified on disk.

## Other backends
- Lance (`lerobot[lancedb]`): read-only, random access from object storage. Convert with
  `lerobot-lance-convert` from the separate `lerobot-lancedb` package.
- HF Storage Buckets: `--dataset.repo_type=bucket` or `--dataset.root=hf://buckets/<org>/<bucket>`.

## Quick start (load, visualise, edit, convert)

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
