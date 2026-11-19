# forge Per-Format Notes and Conversion Pitfalls

Load this file before converting with forge, when a converted dataset looks wrong, or when
quality results on a given source format look implausible (missing state, zero rewards,
black frames, wrong FPS). It records what each forge reader and writer actually does.

Verified by reading the source at forge commit `12a3167` (2026-11-18); file:line references are
to `forge/formats/` unless stated. Items marked *(code reading)* were inferred from the code
path and not exercised; confirm them on your data with the round-trip check at the end.

## The model everything passes through

Every reader yields `Episode` objects of `Frame`s (`forge/core/models.py`). A frame carries
`timestamp`, `images` (lazy), one `state` vector, one `action` vector, `reward`,
`is_first`/`is_last`/`is_terminal`, and `extras`. Episodes carry `language_instruction`,
`success`, `fps`, `cameras`, `metadata`.

Consequences:

- **One state vector and one action vector.** Anything a reader does not put into `state`,
  `action` or `images` is gone after conversion. Each reader picks those by key-name
  heuristics (below), so the first job after `forge inspect` is to confirm which source keys
  became state and action.
- **No unit, frame, or rotation conversion anywhere.** Radians stay radians, degrees stay
  degrees, quaternion order is whatever the source used. forge cannot fix a units mismatch for
  you; it also will not introduce one.
- **Writers drop what the model does not carry**: per-step `success`, `discount`, most
  episode metadata, joint names.

Format detection order (`registry.py:122-132`): groot, lerobot-v3, lerobot-v2, robodm, rlds,
zarr, hdf5, mcap, rosbag; first match wins. Pass `--source-format` when detection is in doubt.

## Read support summary

| Format | Episodes from | State | Action | Timestamps | Language |
|---|---|---|---|---|---|
| lerobot-v3 | `episode_index` column / episode metadata | first column containing `state` or `observation` (not image) | first column containing `action` | parquet `timestamp` | first task of episode |
| lerobot-v2 | one parquet per episode | last column containing `state` | last column containing `action` | parquet `timestamp` | not read |
| groot | delegated to the lerobot-v3 reader | as v3 | as v3 | as v3 | as v3; `annotation.*` dropped |
| rlds (TFDS) | TFDS episodes | first of `state`, `proprio`, `proprioception`, `robot_state` | `action` | none | episode-level `language_instruction` only |
| zarr | `episode_ends` (exclusive); whole store = 1 episode if absent | exactly `state`, `robot_state` or `robot0_eef_pos` | key containing `action` | none | none |
| hdf5 robomimic | `/data/demo_*` of the **first file only** | concat of `robot0_joint_pos` and `robot0_eef_pos` | `actions` | none | none |
| hdf5 ALOHA | one file per episode | `qpos` | `action` | none | none |
| mcap | topic config strategy (`single`, `marker`, `time_gap`, `segment`) | `observation.state` field | `action` field | message `log_time` (absolute) | task description / first task msg |
| rosbag | whole bag = one episode | **none** | **none** | receive time (absolute) | none |
| robodm | one `.vla` per episode | concat of state-like keys | concat of action-like keys | synthesised at 30 Hz | none |

## LeRobot v3 (read and write)

Upstream layout (LeRobot docs, "LeRobotDataset v3.0"): `meta/info.json`, `meta/stats.json`,
`meta/tasks.parquet`, `meta/episodes/` (chunked parquet with per-episode lengths, tasks and
offsets), `data/` parquet shards and `videos/` MP4 shards per camera, each holding many
episodes. Episode boundaries live in metadata, not file names.

Reader (`lerobot_v3/reader.py`):

- Detected when `codebase_version` major >= 3, **or** `meta/tasks.jsonl` exists, or
  `meta/tasks.parquet` exists (:262-273). LeRobot v2.x datasets have `tasks.jsonl`, so they are
  read by the v3 reader, not the v2 reader.
- State is the first array column whose name contains `state` or `observation` and not
  `image` (:1344-1361). A dataset with `observation.environment_state` before
  `observation.state` gets the environment state as "state". Check `forge inspect` output.
- Only the first task string of an episode is used; per-frame `task_index` changes are ignored.
- Image columns stored inline in parquet (feature dtype `image`) are not turned into frames;
  only MP4 video features are.
- Not read: `next.reward`, `next.done`, extra observation columns.
- A missing requested video frame raises `ValueError` (no black-frame substitution).
- Cloud streaming path (s3/gs inspect and ingest) reads only `observation.state` and `action`.

Writer (`lerobot_v3/writer.py`), `codebase_version: "v3.0"`:

- Layout: `data/chunk-NNN/file-000.parquet`, `videos/observation.images.<cam>/chunk-NNN/file-000.mp4`,
  `meta/info.json`, `meta/episodes/chunk-NNN/file-000.parquet`, `meta/stats.json`,
  `meta/tasks.parquet`. **Each episode gets its own chunk** (:450-456); `chunks_size` and file
  size settings are not enforced. Valid, but many small files for large datasets.
- Video: libx264, CRF 23, preset medium, yuv420p (lossy; `forge/video/encoder.py`). Frames go
  through `rgb24`. Float images <= 1.0 are scaled by 255; other integer types are cast to uint8
  (uint16 depth wraps). Encoder size comes from the first frame's declared size.
- `fps` in info.json is cast to **int** (:540): 29.97 becomes 29, 15.5 becomes 15.
- State and action `names` are written as `None` (:373, :384): joint names are lost.
- Feature shapes come from the first frame seen; no check that later episodes match
  *(code reading: ragged dims will most likely fail at `finalize`)*.
- `stats.json`: min, max, mean, std, count for `observation.state` and `action` only (no
  quantiles, no image stats).
- Tasks: `language_instruction`, or `"default"` when missing. Run `forge lint` on the output.
- Absolute timestamps (MCAP, rosbag sources) are copied as-is, not rebased to 0.
- Dropped: `Episode.metadata`, `success`, `reward`, done flags, extras.
- Writer knobs via YAML `writer_config:` (any attribute of `LeRobotV3WriterConfig`):
  `video_codec`, `video_crf`, `video_preset`, `repo_id`, `robot_type`, `fps`.

Train with LeRobot after conversion only once `lerobot` itself loads the output; see the
`lerobot` skill for loading and the `lerobot` dataset tooling.

## LeRobot v2

- Reader only; **there is no v2 writer** (`forge formats` shows lerobot-v2 write as `-`; the
  forge README table claiming v2 write is wrong).
- Reached only when there is no `tasks.jsonl`/`tasks.parquet` and version < 3.
- Language, fps, robot type, rewards and done flags are not read.
- LazyImage size is hard-coded 480x640, and **a failed video decode returns a black frame**
  (`lerobot_v2/reader.py:476-484`). Black frames are a silent quality hazard: run
  `forge quality --video` and look for `under_exposed` and `frozen_frames` on v2-read data.

## RLDS

Upstream semantics (google-research/rlds): `is_first`, `is_last` mandatory; on the
`is_last` step `action`, `reward`, `discount` are not valid; `is_terminal` false on the last
step means the episode was truncated. Episode metadata may carry `invalid` for episodes to
discard.

Reader (`rlds/reader.py`):

- Tries TFDS first, then falls back to raw TFRecord parsing.
- TFDS path: images from observation keys containing `image`, `rgb`, `camera`, `img`, `frame`;
  state from the first of `state`, `proprio`, `proprioception`, `robot_state`; every other
  observation key is dropped.
- Missing reward becomes `0.0`. `discount` and `episode_metadata` are dropped.
- Language is read only from episode-level `language_instruction` (:493-499). Open-X datasets
  usually store language per step, so expect `language_coverage: 0` from forge on many OXE
  datasets and verify with `forge inspect`/`forge lint` before trusting a converted copy.
- No timestamps and no FPS: pass `--fps` to `convert` and `quality`.
- Raw fallback path: image bytes that PIL cannot decode are reshaped as raw 480x640x3.
- The final-step action is passed through even though RLDS says it is not meaningful; trim or
  mask it if your policy trains on (obs_t, action_t) pairs from converted RLDS.

Writer (`rlds/writer.py`):

- Writes `<dataset_name>-train.tfrecord-XXXXX-of-YYYYY`, `dataset_info.json`, `features.json`.
- Serialises a flat `tf.train.Example` with `steps/...` keys rather than a nested RLDS
  sequence (the code comment says this is a simplification). Whether
  `tfds.builder_from_directory` and Open-X loaders accept it is unverified: test-load the
  output in the consumer before training.
- Images JPEG quality 95 by default (lossy); non-uint8 images are always multiplied by 255
  (wrong for 0-255 floats).
- `reward` defaults to 0.0; `discount` is constant 1.0; fps and robot type are not written to
  `dataset_info.json`; `success` is lost.

## Zarr (Diffusion Policy, UMI)

- Read only. Episodes from `meta/episode_ends` or root `episode_ends` (exclusive ends); if
  absent, the whole store becomes one episode.
- State is taken only from a key named exactly `state`, `robot_state` or `robot0_eef_pos`
  (`zarr/reader.py:461`), with no concatenation. For UMI-style stores the state is just the
  3-D end-effector position; `robot0_eef_rot_axis_angle`, `robot0_gripper_width` and similar
  are dropped.
- Image keys: name contains `image`, `rgb`, `camera`, `img`, `frame`, `video`, or 4-D uint8
  with 1/3/4 channels. Camera names have underscores removed. Images assumed HWC.
- **No timestamps, no FPS** on episodes. Example: the Push-T replay store in forge's
  `sample_data` has no fps attribute, so `forge inspect` reports `missing_required: ["fps", ...]`
  and `forge quality` falls back to 30 FPS (report `provenance.timebase: inferred_from_fps`).
  Diffusion Policy's Push-T is commonly run at 10 Hz; at the wrong FPS every time-based threshold
  (pause duration, gripper chatter rate, LDLJ) is off by 3x. Always pass `--fps`.
- Units vary by dataset: Push-T state is in pixels of a 512x512 workspace, so the default
  `--pause-speed 0.001` (units/s) is meaningless there; supply `--state-scales`.

## HDF5 (robomimic, ALOHA)

- Read only. Layout chosen from the first file. For robomimic **only the first file** is read.
- robomimic: demos re-numbered by sort position (ids are positions, not `demo_N` numbers).
  State is `robot0_joint_pos` and `robot0_eef_pos` concatenated in HDF5 key order
  (`hdf5/reader.py:552-556`); `eef_quat`, `gripper_qpos`, `object`, `joint_vel`, rewards,
  dones, `/mask/train` and `/mask/valid`, and demo attributes are dropped. Save the masks
  yourself if you need the official split.
- ALOHA: state is `qpos` only (qvel, effort dropped). Images passed through raw; there is no
  JPEG decode, so compressed-image ALOHA files will most likely fail *(code reading)*.
- No timestamps; FPS only from robomimic `env_kwargs.control_freq` at inspect time.

## MCAP

- Read and write; no ROS install needed. Reading is driven by a topic config; generate a
  starter with `forge inspect file.mcap --generate-config topics.yaml` and edit it. The
  auto-config maps JointState topics matching `cmd|command|target|commanded|desired|action|goal`
  to action and the highest-rate remaining JointState to state; ambiguous cases get TODOs.
- Episode strategies: `single`, `marker` (marker topic starts an episode), `time_gap`,
  `segment` (PELT; falls back to one episode if `ruptures` is missing). Post-filters
  `drop_first_n_frames`, `min_length_frames`.
- One frame per primary-topic message; other streams aligned by `nearest`, `hold` (zero-order)
  or `interpolate` (linear, also on quaternions, no slerp). Skew beyond `max_skew_ms` logs a
  warning; beyond 10x the frame is **dropped**, leaving index gaps. Default `max_skew_ms` is 50
  in the dataclass, 150 from auto-generated configs.
- Timestamps are absolute `log_time` seconds (not rebased).
- `forge inspect` FPS on MCAP is total messages across all channels divided by duration
  *(code reading)*, which overstates the frame rate for multi-topic files; the converter
  copies that inspect FPS into the writer. Pass `--fps` explicitly when converting MCAP.
- Raw images: no BGR swap. Compressed images decoded with OpenCV and converted to RGB; on
  decode failure the raw bytes are returned silently.
- `segment` strategy boundaries are computed on the signal topic's sample indices and applied
  to the primary topic *(code reading)*; check boundaries when the topics have different rates.

MCAP writer: one file; state and action as `sensor_msgs/JointState` with positions only and
empty joint names, images as uncompressed `sensor_msgs/Image`, per-episode topic suffix
`/ep_XXXX`. **Language is not written** (despite the docstring), nor rewards, metadata or FPS.

## Rosbag (ROS1 `.bag`, ROS2 `.db3` directories)

- Read only. **The whole bag becomes one episode.**
- **Only the first image topic is read; state and action are never filled**
  (`rosbag/reader.py:480-490`). Joint and command topics only appear in `inspect`. forge's
  rosbag path is therefore useful for looking at a camera stream, not for producing training
  data. For ROS2 data, record or convert to MCAP and use the MCAP reader with a topic config.
- Raw `sensor_msgs/Image`: no BGR swap, row padding ignored; `CompressedImage` messages
  produce frames without images.

## RoboDM (`.vla`)

Read and write (manual RoboDM install). Reader concatenates state-like and action-like keys in
dict order and synthesises timestamps at a hard-coded 30 Hz. Writer drops language, timestamps
and per-episode metadata; `codec_options` in the writer config is never used.

## Conversion pitfalls at a glance

forge converts through one canonical `Episode`/`Frame` model: one state vector, one action
vector, images, timestamps, language, and a few flags per frame. Anything else is dropped,
and the reader decides which source keys become state and action by name heuristics. forge
performs **no unit, rotation or frame-convention conversion**. The most consequential
behaviours at this commit (details per format below):

| Area | What forge does | What to do |
|---|---|---|
| State/action selection | Name heuristics per reader; YAML `fields:` mapping is parsed but **not applied** | Check `forge inspect` dims; preprocess the source if the wrong key wins |
| Dropped observation keys | robomimic: only `robot0_joint_pos` + `robot0_eef_pos`; Zarr: only `state`/`robot_state`/`robot0_eef_pos`; ALOHA: `qpos` only | Expect a smaller state; concatenate upstream if you need more |
| rosbag | Whole bag is one episode; only first image topic; **no state or action** | Use MCAP with a topic config for training data |
| FPS | Zarr, HDF5, RLDS carry none (fallback 30); MCAP inspect FPS counts all topics; LeRobot v3 writer casts FPS to int | Always pass `--fps` to `convert` and `quality` |
| Timestamps | MCAP and rosbag timestamps are absolute and not rebased | Rebase in your loader if it assumes t0 = 0 |
| Episode boundaries | Zarr without `episode_ends` = one episode; MCAP needs a strategy (`marker`, `time_gap`, `segment`) | Verify episode count and length distribution after conversion |
| Images | LeRobot v3 output is H.264 CRF 23 yuv420p (lossy); RLDS output JPEG q95 and multiplies non-uint8 by 255; raw ROS/MCAP images have no BGR swap; uint16 depth wraps when cast to uint8 | Compare frames visually; keep depth out of RGB paths |
| Decode failures | LeRobot v2 reader returns black frames silently; MCAP compressed decode failure returns raw bytes; LeRobot v3 reader raises | Run `--video` checks on the source and the output |
| Language | RLDS reads only episode-level `language_instruction` (Open-X stores it per step); LeRobot v2 reader reads none; MCAP writer writes none; LeRobot v3 writer writes `"default"` when missing | `forge lint` the output |
| Metadata | Joint names (`names: None`), success labels, rewards, done flags, episode metadata dropped by the LeRobot v3 writer; RLDS writer sets reward 0.0 and discount 1.0 | Keep a sidecar with labels and splits keyed by episode index |
| v2 vs v3 detection | A LeRobot v2.x dataset with `meta/tasks.jsonl` is detected and read as `lerobot-v3` | Pass `--source-format` if results look wrong |
| robomimic splits | `/mask/train`, `/mask/valid` dropped; only the first HDF5 file is read | Save masks before converting; convert files separately |
| RLDS last step | The `is_last` step's action is meaningless per RLDS but passed through | Drop or mask it for (obs, action) training pairs |
| YAML `video:` settings | Not applied; use `writer_config: {video_crf: 20}` | Check the written config with `--dry-run` |

```yaml
# conversion.yaml: keys the converter really applies
target_format: lerobot-v3
fps: 15
robot_type: franka
cameras:
  agentview_image: front
  robot0_eye_in_hand_image: wrist
writer_config:
  video_crf: 18
```

```bash
forge convert ./demo.hdf5 ./out --config conversion.yaml --dry-run
forge convert ./demo.hdf5 ./out --config conversion.yaml --workers 4
```

After every conversion run the round-trip verification at the end of this file:
episode counts, lengths, dims, per-dim stats, FPS, task coverage and camera names must match,
and video checks must not show new defects.

## Conversion config (YAML) reality check

`forge inspect ds --generate-config c.yaml`, then `forge convert ds out --config c.yaml`.
What the converter actually applies (`forge/convert/converter.py`, `forge/config/models.py`):

| YAML key | Applied? |
|---|---|
| `target_format`, `source_format`, `fps`, `robot_type`, `fail_on_error`, `num_workers` | Yes |
| `cameras:` (source: target) | Yes, for writers with `camera_name_mapping` (lerobot-v3, rlds, robodm) |
| `writer_config:` | Yes: each key set on the writer config if the attribute exists |
| `fields:` / `action_field` / `state_field` | **No.** Parsed and shown in `--dry-run`, but no reader or writer consumes them |
| `fields.*.transform` | No (documented as a future feature) |
| `video: {codec, crf, compress}` | **No.** Use `writer_config: {video_crf: 20}` for lerobot-v3 |
| `include_depth`, `skip_existing`, index mappings | No consumer found |

So you cannot choose which source key becomes state through the YAML. If the reader's
heuristic picks the wrong key, preprocess the source (or write a small script against the
reader API) before converting.

## Round-trip verification (do this after every conversion)

```bash
forge inspect src  --output json > src.json
forge inspect out  --output json > out.json      # episodes, frames, dims, cameras, fps, language
forge stats   src  --output src_stats.json
forge stats   out  --output out_stats.json        # per-dim min/max/mean/std must match closely
forge lint    out                                  # task strings survived?
forge quality src --fps <hz> --export q_src.json
forge quality out --export q_out.json              # same flags on the same episodes?
forge visualize src --compare out                  # interactive; blocks while serving
```

Check: episode count and per-episode lengths equal; state/action dims equal and per-dim stats
equal to float tolerance (a mismatch means a different key was picked, or units changed); FPS
equal (watch the int cast); task coverage equal; camera names mapped as intended; first and last
frames of a few episodes visually identical apart from compression; no new `under_exposed`,
`frozen_frames` or `video_decode_failure` flags in the output.
