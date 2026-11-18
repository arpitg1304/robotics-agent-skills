---
name: robot-data-quality
description: >
  Judge and improve the quality of robot-learning datasets (teleoperated demonstrations,
  episodes) and use forge, the Robotics Data Toolkit, to inspect, lint, score, visualise,
  filter, dedup, and convert them. Use this skill when the user mentions dataset quality,
  data cleaning or curation, episode inspection, bad or noisy demos, idle frames, jerky or
  spiky actions, gripper chatter, timestamp gaps or frame drops, camera blur or exposure,
  duplicate episodes, task-string hygiene, normalisation stats, train/val splits by episode,
  coverage or diversity of demonstrations, or why a policy will not start, stalls, or averages
  modes because of its data. Also trigger on forge commands (forge inspect, lint, stats,
  quality, filter, dedup, visualize, convert, catalog, ingest, curate) and on conversion
  between RLDS, LeRobot v2/v3, Zarr, HDF5, MCAP, rosbag, or RoboDM, including
  hf:// datasets on the Hugging Face Hub.
---

# Robot Data Quality Skill

Verified against forge 0.2.0 (commit 12a3167), 2026-11-18. The version is from
`pyproject.toml`; `forge version` prints `v0.1.0` at this commit because `forge/__init__.py`
was not bumped. Every forge command and flag here was checked against the Typer definitions in
`forge/cli.py` and `forge/catalog/cli.py` and against `forge <cmd> --help` from an editable
install of that commit.

## When to Use This Skill

- Auditing a teleoperated dataset before training (LeRobot, ACT, Diffusion Policy, VLA fine-tuning)
- Explaining a policy failure that might come from the data (won't start, hesitates, jitters,
  averages between strategies, ignores the language instruction)
- Deciding which episodes to drop, trim, relabel, or re-collect
- Running `forge` to inspect, lint, score, review, filter, or dedup a dataset
- Converting between RLDS, LeRobot, Zarr, HDF5, MCAP, rosbag and checking nothing was lost
- Writing a data-quality report for a collection team

Hand off: recording and training commands go to `lerobot`; algorithm choice, action chunking
and why demonstrations need to be consistent go to `imitation-learning`; reward and rollout
data go to `robot-reinforcement-learning`; simulated data generation goes to `mujoco` and
`isaac-sim`.

## Mental Model

A behaviour-cloning policy learns the conditional distribution of actions given observations
**exactly as recorded**. It cannot tell an operator's hesitation from a deliberate wait, a
logging glitch from a real correction, or a mislabelled camera from a different viewpoint.
Data quality is therefore about three things:

1. **Integrity**: is the recorded signal what happened? (timestamps, sync, decode, units,
   camera mapping, NaNs, conversion losses). Integrity bugs are silent and systematic; fix them
   first, because they invalidate every later check.
2. **Behaviour quality**: is each demonstration something you want imitated? (idle spans,
   aborted takes, jerk, regrasps, saturation, failures). These are per-episode decisions.
3. **Distribution**: does the set cover what the policy will see, consistently? (initial
   conditions, object positions, operator strategies, task balance, task strings, splits,
   normalisation stats). These are dataset-level and often need new data, not filtering.

Three rules follow:

- **Scores are triage, not verdicts.** forge's 0-10 score is a heuristic over motion
  signals; it says nothing about task success. Every flag is a review candidate.
- **Look at the episode before deleting it.** Ten minutes in a viewer beats any threshold.
- **Filtering cannot create coverage.** If a region of the workspace has no demos, the fix is
  collection, not cleaning.

## Quality Checklist: What to Check and the Failure It Prevents

Thresholds are **starting points to calibrate on your robot, task and FPS**, not standards.
Print the distribution first, then choose the cut. Recipes for checks forge does not run are in
`references/custom-checks.md`.

### Episode-level

| Check | How (forge or recipe) | Starting-point threshold | Downstream failure it prevents |
|---|---|---|---|
| Length outliers | `forge stats` distribution; robust z-score recipe | abs robust z > 3.5 | Aborted takes teach "stop midway"; marathon takes teach dithering |
| Idle head/tail | `motion_pause` events; `active_span` recipe | head idle > 1 s or > 10% of episode | **Policy that never starts**: initial observation maps to "do nothing" |
| Idle mid-episode | `motion_pause`, `mostly_static` flag (> 50% observed time) | review if pause > 1-2 s not explained by the task | Hesitation, freezing mid-task |
| Held or repeated commands | `held_command` events, `dead_actions` flag (> 10%) | > 10% of frames | Teleop dropout learned as "hold"; also flags frozen logging |
| Truncated or aborted | short length + gripper never releases + object not at goal | any | Policy learns partial skills; success rate ceiling |
| Success / failure labels | `forge stats` success-label coverage; your labels | label every episode | Failures imitated as if correct |
| Gripper events | `--gripper-dims`, `gripper_chatter` (> 2 transitions/s); event-count recipe | pick-place: about 1 close + 1 open | Chatter -> oscillating grasp; regrasps -> hesitant grasps |
| Jerk and spikes | `jerky` (LDLJ < -25), `psd_high_band_chatter` (> 20% energy above 8 Hz); spike recipe | step > 8x median step | Jittery policy, controller faults, normalisation blow-up |
| Action saturation | `--action-bounds` -> `saturated` (> 30% near limits) | > 5% per dim worth review | Policy pinned at limits; clipping hides intent |
| Joint-limit saturation (state) | `limit_occupancy` recipe with URDF limits | > 5% of frames within 2% of a limit | Policy drives into limits; real-arm faults |
| Action-state alignment and lag | `--tracking-pairs`, `tracking_stall_candidate`; `estimate_lag` recipe | lag stable across sessions | Off-by-k shift makes the policy react late or "predict the past" |
| Timestamp monotonicity, gaps | `invalid_timestamps`, `timestamp_gaps` (dt > 2x period), `timestamp_jitter` (std/mean > 0.1), `sample_rate_mismatch` (> 10%) | any invalid; gaps > 1-2 frames | Wrong velocities, misaligned chunks, corrupted temporal context |
| Camera-proprio sync | gripper-close onset in video vs proprio; forge does **not** align clocks | offset < 1 frame | Visual feedback lags action; policy overshoots |
| NaN / non-finite | `nan_in_actions`, `nan_in_states`, `nan_in_timestamps` | any | Training NaNs, corrupted stats |
| Duplicates | `forge dedup` (pHash, 0.10 default) | inspect clusters | Leakage across splits, over-weighted takes |

### Visual

| Check | forge flag (needs `--video`) | Failure it prevents |
|---|---|---|
| Blur | `blurry` (> 30% of frames below var-of-Laplacian 100 at 128 px) | Policy ignores a camera or learns motion-blur cues |
| Exposure | `over_exposed`, `under_exposed` (> 25% of frames) | Object invisible in some sessions; lighting overfit |
| Frozen / repeated frames | `frozen_frames`, `video_frame_drops`, `video_decode_failure`, `video_camera_missing` | Stale observations; also black-frame substitution from bad decoders |
| Shot cuts / camera bumps | `cut_detected`, `shaky` (`--video-level motion`) | Viewpoint shift mid-episode |
| Occlusion | not in forge: review wrist/scene views manually | Policy acts on occluded state |
| Camera moved between sessions | not in forge: compare mean first frame per session | Distribution shift by session; works Monday, fails Tuesday |
| Wrong camera mapping | not in forge: tile one frame per camera key per session | Wrist and scene swapped; policy trained on nonsense |

### Dataset-level

| Check | How | Failure it prevents |
|---|---|---|
| Initial-condition and object coverage | first-frame tiles; `initial_state_spread` recipe | Fails outside the demonstrated region |
| Operator consistency / multimodality | split metrics by operator or session; watch a few per operator | **Mode averaging**: two strategies averaged into one that fails both (see `imitation-learning`) |
| Task-string hygiene | `forge lint` (`task.missing`, `task.generic`, `task.too_short` < 25 chars); `task_hygiene` recipe | Language-conditioned policy ignores or confuses instructions |
| Task and class balance | `forge query` on a catalog, or count tasks | Rare task never learned |
| Train/val split by episode | `split_episodes` recipe, group by session | Frame-level leakage: val loss looks great, robot fails |
| Normalisation stats | `norm_stat_report` recipe; recompute after filtering | One spike squashes all real motion into a sliver of [-1, 1] |
| Units and conventions | `forge stats` per-dim ranges | Degrees vs radians, gripper 0-1 vs 0-100, mixed datasets |

### How data defects show up in policy behaviour

| Policy symptom | Look for in the data |
|---|---|
| Will not start, waits forever at the home pose | Idle heads; many `motion_pause` events starting at frame 0 |
| Starts then freezes midway | Mid-episode pauses, operator hesitation, aborted takes left in |
| Jitters or buzzes at standstill | Spikes, `psd_high_band_chatter`, teleop noise, gripper chatter |
| Goes to the average of two places | Multimodal strategies (left vs right approach) with few demos each |
| Grasps late or overshoots | Action-state lag, camera-proprio offset, wrong FPS |
| Works in one session's lighting only | Exposure differences, camera moved, session imbalance |
| Ignores the language instruction | Generic or missing task strings, one string for several tasks |
| Hits joint limits | Saturated state/actions in demos, min/max stats set by outliers |
| Great val loss, bad on robot | Frame-level split, duplicates across splits, val from same sessions |

### Drop, trim, relabel, or re-collect

- **Trim**: idle head/tail, operator setup motion before the task. Keep a few static frames at
  the start so the initial observation is represented. forge has no trim command; trim by
  index in your writer or data loader.
- **Drop**: integrity failures (NaN, invalid timestamps, decode failures, black frames), aborted
  or failed takes (unless you train failure-aware or RL methods), exact duplicates, episodes
  with an unfixable camera mapping.
- **Relabel**: wrong or generic task strings, swapped camera names, success labels. Cheaper
  than deleting.
- **Re-collect**: coverage holes, a camera that moved, a systematic sync or calibration error,
  inconsistent operator strategies (agree a protocol first), too few demos per variation.
- **Keep**: legitimate waits (pouring, settling), deliberate slow approaches, contact-induced
  tracking error. These look "bad" to motion heuristics and are often essential.

As a rule of thumb, dropping more than about 20% of a dataset by heuristics means the
collection process is the problem; report that instead of silently filtering.

## forge Workflows

### Install and sanity check

```bash
pip install forge-robotics                     # toolkit (LeRobot v3 read/write)
pip install "forge-robotics[video,hdf5,zarr]"  # add what your formats need
# Catalog, search, curation and Studio are newer than the PyPI release:
#   git clone https://github.com/arpitg1304/forge && pip install -e "forge[all]"
forge formats                                  # read/write matrix on this install
```

Extras: `[rlds]`, `[lerobot]`, `[mcap]`, `[hdf5]`, `[zarr]`, `[rosbag]`, `[video]`
(needed for `--video`, dedup, export-video), `[segment]`, `[rerun]`, `[s3]`, `[gcs]`,
`[catalog]`, `[embed]`, `[all]`.

Path arguments accept local paths, `hf://org/repo` (optionally `@revision`), bare `org/repo`,
registry ids (`forge registry list`), and `s3://` / `gs://`. Outputs must be local.

Write support: `lerobot-v3`, `rlds`, `mcap`, `robodm` only. HDF5, Zarr, rosbag, LeRobot v2 and
GR00T are read-only. `filter` and `dedup` write the source format, so convert first when the
source is read-only.

### 1. Inspect and lint

```bash
forge inspect ./ds --output json > inspect.json     # format, dims, cameras, fps, language coverage
forge inspect hf://lerobot/pusht --quick            # Hub metadata only, no download
forge lint ./ds --export lint.json                  # hygiene; non-zero exit on errors
forge lint ./ds --strict                            # CI gate: warnings fail too
```

Read `missing_required` (for example `["fps", "robot_type"]`), `has_timestamps`,
`language_coverage`, `observation_schema`, `action_schema`, and `cameras`. Confirm which source
keys became state and action: readers pick them by name heuristics (see
`references/forge-format-notes.md`).

### 2. Distributions

```bash
forge stats ./ds --output stats.json                # lengths, per-dim action/state stats, coverage
forge stats ./ds --sample 100                       # first 100 episodes only
```

Look for unit mistakes (joint ranges > 2*pi suggest degrees), constant dims, gripper scale,
and success-label/reward coverage.

### 3. Proprioception quality (configure it; defaults know nothing about your robot)

```bash
# Minimal run. Gripper and physical limits are NOT inferred; those subscores are omitted.
forge quality ./ds --export quality.json

# Configured run for a 7-DoF arm + gripper at action column 7, actions in [-1, 1]:
forge quality ./ds \
  --gripper-dims 7 --action-bounds=-1,1 \
  --state-dims 0,1,2,3,4,5,6 \
  --pause-speed 0.01 --event-min-duration 0.5 \
  --fps 15 --workers 8 \
  --export quality.json --export-flagged flagged.json

# Absolute joint-position commands with matching state units: enable tracking-stall checks
forge quality ./ds --action-semantics position \
  --tracking-pairs 0:0,1:1,2:2,3:3,4:4,5:5,6:6 \
  --tracking-error 0.05 --tracking-lag 0.1 --export quality.json

# Bimanual (ALOHA-style 14-D) grippers
forge quality ./ds --gripper-dims 6,13 --export quality.json
```

Configuration rules:

- `--fps` is only a fallback when timestamps are missing (Zarr, HDF5, RLDS, RoboDM). If you
  leave it at 30 on a 10 Hz dataset, every duration and rate threshold is wrong. Check
  `provenance.timebase` in the export (`stored` vs `inferred_from_fps`).
- `--pause-speed` and `--tracking-error` are in **scaled state units**. Defaults (0.001, 0.05)
  assume radians or metres. For mixed units pass `--state-scales` with one positive divisor
  per original state column (for example to convert pixels or millimetres).
- `--gripper-dim` and `--gripper-dims` are mutually exclusive; `--tracking-pairs` requires
  `--action-semantics position`. Vector shape never establishes correspondence.
- `--action-bounds` applies one range to all dims. Per-dim limits exist only in Python
  (`QualityConfig(action_limits=...)`).
- `--sample N` and `--quick` (= 50) take the **first** N episodes, not a random subset. Early
  episodes are often from one session; score all episodes for decisions.
- `overall_score` can be `null` (invalid data, irregular timebase, no evaluable motion). Treat
  null as "needs review", never as "fine".
- Scores are comparable only across runs with the same configuration and scorer version
  (`q-v2-proprio`). Adding a gripper column or bounds changes which subscores exist.

Reading the export:

```python
import json
r = json.load(open("quality.json"))
eps = r["per_episode"]
worst = sorted((e for e in eps if e["overall_score"] is not None), key=lambda e: e["overall_score"])[:10]
unscored = [e["episode_id"] for e in eps if e["overall_score"] is None]
for e in worst:
    pauses = [ev for ev in e["events"] if ev["kind"] == "motion_pause"]
    head = [ev for ev in pauses if ev["start_frame"] == 0]
    print(e["episode_id"], round(e["overall_score"], 2), e["flags"],
          "head_idle_s=%.2f" % (head[0]["end_time_s"] - head[0]["start_time_s"]) if head else "")
print("unscored:", unscored)
print(r["flagged_episodes"].keys())
```

`direction_reversal` events are descriptive (cyclic and corrective motion produce many); they
are not a retry detector. `net_to_path_ratio` is low for closed-loop tasks by nature.

### 4. Video integrity and pixel checks

```bash
forge quality ./ds --video --video-stride 4 --export quality.json
forge quality ./ds --video --video-level motion --video-stride 4 \
  --video-max-frames 200 --video-cameras observation.images.top --export quality.json
```

- Pixel thresholds are whole-image heuristics at 128 px; calibrate per camera (dark scenes and
  textureless backgrounds trigger blur and exposure flags).
- Frame-drop estimates come from episode timestamps and expected FPS, not camera PTS. A gap
  warns when it hides >= 3 observations, > 0.2 s beyond one period, or the estimated drop
  fraction exceeds 5% (`--video-drop-frames`, `--video-gap-seconds`, `--video-drop-fraction`).
- Strided pixel events describe sampled frames only.
- Video flags never change the proprioception score; filter on them explicitly.

### 5. Look at the evidence

```bash
forge quality ./ds --sample 20 --video --visualize   # event lanes, click-to-seek playback
forge visualize ./ds --backend rerun --episode 12 --segment
forge export-video ./ds -e 12 -o ep12.mp4 --grid     # non-interactive: produce a file to view
forge export-video ./ds --all -o ./videos/
forge segment ./ds --export segments.json --plot timeline.png   # phases via PELT ([segment])
```

`--visualize` and `forge visualize --backend web` **serve a local page and block until
Ctrl+C**. In a non-interactive agent session, run them in the background or use
`export-video`, `--plot`, and the JSON exports instead. Viewer colours: green = no event from
the enabled checks (not success), amber = review candidate, red = tracking-stall candidate,
gray = gap or unavailable.

### 6. Filter (always from a configured report)

```bash
forge filter ./ds --from-report quality.json --exclude-flags nan_in_actions,invalid_timestamps   # dry-run
forge filter ./ds ./ds_clean --from-report quality.json \
  --exclude-flags nan_in_actions,nan_in_states,invalid_timestamps,video_decode_failure \
  --exclude-episodes 3,17,42
forge filter ./ds ./ds_clean --from-report quality.json --min-quality 5.0
```

- Episode ids are reader-specific strings (`"0"`, `"1"` for Zarr; file-derived names for
  others). Copy them from the report's `episode_id` fields rather than guessing.
- Without `--from-report`, filter re-scores with only `--gripper-dim`, `--fps` and
  `--action-bounds`: your state dims, scales, tracking pairs and event thresholds are lost, so
  flags differ from the report you reviewed.
- `--min-quality` **keeps** null-score episodes, and episodes missing from the report pass
  quality filters with a warning. Exclude unscored ids explicitly if you want them out.
- The quality recommendations may print `forge filter --max-gripper-chatter`; that flag does not
  exist. Use `--exclude-flags gripper_chatter`.
- The output is a new dataset in the same format (needs a writer). Recompute normalisation
  stats for training on the filtered output.

### 7. Duplicates

```bash
forge dedup ./ds                                     # dry-run: clusters
forge dedup ./ds ./ds_dedup --threshold 0.05 --cameras observation.images.top
```

Keeps the earliest episode per cluster; an episode is a duplicate only if every shared camera
matches. Static-scene tasks with near-identical demos can cluster falsely: inspect clusters
before writing. Run dedup **before** splitting so duplicates cannot straddle train and val.

### 8. At corpus scale: the catalog (install from source)

```bash
forge catalog init ./cat
forge ingest ./session_a ./session_b hf://org/ds -c ./cat      # idempotent by content hash
forge query "SELECT task, count(*) n, avg(overall_score) s FROM episodes
             JOIN v_latest_quality USING (episode_id) GROUP BY task ORDER BY n" -c ./cat
forge embed -c ./cat && forge catalog dedup -c ./cat --threshold 0.97
forge search "regrasp after a failed pick" -c ./cat --top 20 --save sel.json
forge curate -c ./cat --where "overall_score > 6" --dedup 0.97 \
  --dedup-policy keep-higher-quality --label approved
forge studio -c ./cat -o studio.html
```

`forge ingest` scores with the default analyser (no gripper, bounds or state scales; not
configurable on the CLI), so catalog scores are not comparable with a configured
`forge quality` run. Curation only labels episodes; exporting a curated selection to a training
dataset is on forge's roadmap, not implemented.

Full option tables, flag triggers and JSON fields: `references/forge-cli-reference.md`.

## Conversion Pitfalls

forge converts through one canonical `Episode`/`Frame` model: one state vector, one action
vector, images, timestamps, language, and a few flags per frame. Anything else is dropped,
and the reader decides which source keys become state and action by name heuristics. forge
performs **no unit, rotation or frame-convention conversion**. Before converting, load
`references/forge-format-notes.md`; the most consequential behaviours at this commit:

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

After every conversion run the round-trip verification in `references/forge-format-notes.md`:
episode counts, lengths, dims, per-dim stats, FPS, task coverage and camera names must match,
and video checks must not show new defects.

## Triage Workflow

Run in this order; each step can invalidate the next.

1. **Pin provenance.** Record dataset path or `hf://org/repo@<revision>`, forge commit, and the
   exact commands. Work on a copy; never filter in place.
2. **Integrity (15 min).** `forge inspect --output json`, `forge lint`, `forge stats`. Confirm
   FPS, dims and units, state/action key choice, camera list, timestamp presence, task
   coverage. Stop and fix anything wrong here before scoring.
3. **Configure and score everything.** `forge quality` with gripper columns, bounds, state
   dims/scales, real FPS, `--export`. Then `--video --video-stride 4` on all cameras.
4. **Rank and look.** Sort by score and by flag; watch the 10 worst, 5 median and 5 best
   episodes, plus every null-score episode. Calibrate thresholds from what you see: a flag
   that fires on good episodes is mis-set, not informative.
5. **Custom checks.** Idle head/tail, length outliers, gripper event counts, lag, joint
   limits, normalisation stats (`references/custom-checks.md`).
6. **Dataset level.** First-frame tiles per session and camera, task-string table, per-task
   and per-operator counts and score distributions, initial-condition spread.
7. **Duplicates.** `forge dedup` dry-run; review clusters.
8. **Decide.** For every episode: keep, trim (with indices), relabel, drop (with reason). For
   the dataset: what to re-collect.
9. **Apply and verify.** `forge filter --from-report` (or your own trim), recompute stats,
   episode-level split by session, re-run steps 2-3 on the output.
10. **Report.**

### Report template

```markdown
## Data quality report: <dataset> @ <revision>   (forge 12a3167, <date>)
Scope: N episodes, M frames, FPS, cameras, state/action dims, tasks.
Config: forge quality <exact flags>

### Integrity (blocking)
- <issue>, <evidence: command/output>, <episodes affected>, <fix>

### Episode findings
| Issue | Episodes | Evidence | Action (drop / trim / relabel / keep) |

### Dataset-level findings
- Coverage, operator/session differences, task balance, task strings, duplicates.

### Decisions applied
- Dropped K (list + reason), trimmed T (indices), relabelled R. Remaining: N' episodes.
- Split: by <session|episode>, salt <s>, val ids <file>.
- Normalisation stats recomputed on: <dataset>.

### Recommended re-collection
- <region/condition>, <why>, <how many demos>.

### Caveats
- Heuristic thresholds used, calibrated on: <episodes reviewed>. Unscored episodes: <ids>.
```

State the evidence for each claim and say which checks were not run. "Score 4.2" is not a
finding; "operator B's 31 episodes have a 2.4 s median idle head (motion_pause from frame 0)"
is.

## Anti-Patterns

### 1. Filtering by score alone
```bash
# BAD: drops legitimate slow or waiting demos, keeps null-score (possibly corrupt) episodes
forge filter ./ds ./clean --min-quality 7
```
```bash
# GOOD: drop integrity failures by flag, behaviour issues by reviewed ids, keep a record
forge filter ./ds ./clean --from-report quality.json \
  --exclude-flags nan_in_actions,nan_in_states,invalid_timestamps \
  --exclude-episodes 4,19,23
```

### 2. Splitting train/val by frame
```python
# BAD: neighbouring frames are near-identical; val loss measures memorisation
train, val = train_test_split(all_frames, test_size=0.1)
```
Split by episode, or by session/scene for a generalisation estimate, and dedup first.

### 3. Trusting default units and FPS
Running `forge quality` on a 10 Hz Zarr dataset with pixel-unit state and default
`--fps 30 --pause-speed 0.001` produces confident numbers that mean nothing. Pass `--fps` and
`--state-scales`, and check `provenance` in the export.

### 4. Treating flags as failures
`mostly_static` on a pouring task, `direction_reversal` counts on wiping, and
`tracking_stall_candidate` during contact are expected. Review before dropping.

### 5. Computing normalisation stats before cleaning
Stats from the unfiltered set still contain the spike you removed. Recompute after every
filter and prefer quantile-based ranges when outliers exist.

### 6. Converting without verifying
Assuming `forge convert` preserved everything. It drops keys, metadata, labels and joint names
by design and may pick a different state key than you expect. Run the round-trip check.

### 7. Scoring a sample and deciding for the whole dataset
`--sample 50` is the first 50 episodes, usually one session. Use samples for calibration only.

### 8. Mixing configurations in one comparison
Comparing scores from a configured run with catalog ingest scores, or q-v1 reports with
q-v2, or runs with and without `--gripper-dims`.

### 9. Blocking the agent on a viewer
Calling `forge quality --visualize` or `forge visualize` in a non-interactive shell hangs until
Ctrl+C. Background it, or export videos and plots.

### 10. Cleaning away the multimodality problem
Deleting "odd" demos until the remaining ones agree can hide a real two-strategy task.
Decide with the team whether to standardise the strategy (re-collect) or use a policy class
that handles multimodality (see `imitation-learning`).

## Gaps (not in forge yet)

Useful checks that forge does not provide at commit 12a3167; use the recipes in
`references/custom-checks.md` or other tools:

- Trimming idle heads and tails, or any episode-level crop, when writing.
- Episode-length outlier flags (stats prints the distribution only).
- Estimating action-to-state lag (`--tracking-lag` is an input) and camera-to-proprio clock
  alignment.
- Joint-limit saturation on measured state from URDF limits; per-dim action limits on the CLI.
- Success/failure detection, aborted-take detection, gripper event counting (rate-based chatter only).
- Camera-moved-between-sessions and wrong-camera-mapping detection; occlusion.
- Normalisation-stat outlier reports; quantile stats in the LeRobot v3 writer's `stats.json`.
- Episode-level train/val/test splitting (on the roadmap) and task-string canonicalisation.
- Configurable quality settings for `forge ingest`; exporting a curated catalog selection.
- Pushing datasets to the Hugging Face Hub (forge downloads and searches only).
- Honouring YAML `fields:` and `video:` conversion settings; rosbag state/action extraction;
  per-step RLDS language; LeRobot v3 writer joint names and fractional FPS.

## Checklist

- [ ] Provenance recorded: source revision, forge commit, exact commands
- [ ] `forge inspect` dims, FPS, cameras, timestamps and language coverage confirmed
- [ ] State and action keys confirmed to be the intended source fields, units known
- [ ] `forge lint` clean or warnings understood; task strings canonical
- [ ] `forge quality` run on **all** episodes with gripper columns, bounds, state scales, real FPS
- [ ] Video checks run on every camera; thresholds calibrated on reviewed episodes
- [ ] Worst, median, best and all null-score episodes reviewed visually
- [ ] Idle heads/tails measured and trimmed or justified
- [ ] Length outliers, gripper events, spikes, saturation, lag checked
- [ ] Duplicates reviewed with `forge dedup` before splitting
- [ ] Coverage of initial conditions and object placements visualised; gaps listed
- [ ] Per-operator / per-session differences checked for multimodality and shift
- [ ] Split by episode (or session), salted and recorded
- [ ] Filter applied from the configured report; unscored episodes handled explicitly
- [ ] Normalisation stats recomputed on the final dataset
- [ ] Conversions round-trip verified (counts, dims, stats, FPS, tasks, cameras, frames)
- [ ] Report written with evidence, decisions, re-collection plan, and caveats

## Reference Files

- `references/forge-cli-reference.md`: load when you need any forge flag, default, flag
  trigger, JSON field, catalog table column, or Python API call not shown above.
- `references/forge-format-notes.md`: load before any `forge convert`, when a reader's output
  looks wrong (missing state, no language, black frames, wrong FPS), or to run the round-trip
  verification.
- `references/custom-checks.md`: load to run checks forge lacks: idle trim spans, length
  outliers, spikes, lag, joint-limit occupancy, gripper events, normalisation stats, coverage,
  task-string hygiene, episode-level splits.

## Related Skills

- `lerobot`: recording, dataset tooling, loading, and training with LeRobot; the consumer of
  most cleaned datasets.
- `imitation-learning`: why consistency matters, multimodality, action chunking, and how data
  volume and diversity trade off.
- `robot-reinforcement-learning`: reward, success and failure data, offline RL datasets.
- `mujoco`, `isaac-sim`: generating or replaying demonstrations in simulation, scripted
  variation for coverage.
- `robot-perception`: camera calibration, image pipelines and sensor issues behind visual defects.
- `robotics-testing`: turning `forge lint --strict` and quality gates into CI checks.
