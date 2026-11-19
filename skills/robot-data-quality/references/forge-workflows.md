# forge Workflows in Detail

Load this file when running a forge step beyond the short sequence in SKILL.md: install
extras, path arguments and write support, configuring `forge quality`, reading the export,
video checks, viewers, filter and dedup details, the catalog, or when you need to know what
forge does not do yet.

## Install and sanity check

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

## 1. Inspect and lint

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

## 2. Distributions

```bash
forge stats ./ds --output stats.json                # lengths, per-dim action/state stats, coverage
forge stats ./ds --sample 100                       # first 100 episodes only
```

Look for unit mistakes (joint ranges > 2*pi suggest degrees), constant dims, gripper scale,
and success-label/reward coverage.

## 3. Proprioception quality (configure it; defaults know nothing about your robot)

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

## 4. Video integrity and pixel checks

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

## 5. Look at the evidence

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

## 6. Filter (always from a configured report)

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

## 7. Duplicates

```bash
forge dedup ./ds                                     # dry-run: clusters
forge dedup ./ds ./ds_dedup --threshold 0.05 --cameras observation.images.top
```

Keeps the earliest episode per cluster; an episode is a duplicate only if every shared camera
matches. Static-scene tasks with near-identical demos can cluster falsely: inspect clusters
before writing. Run dedup **before** splitting so duplicates cannot straddle train and val.

## 8. At corpus scale: the catalog (install from source)

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
