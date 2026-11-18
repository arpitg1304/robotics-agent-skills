# forge Command Reference (quality-relevant subset)

Load this file when you need an exact flag, default, or output field for a forge command
beyond the workflows in SKILL.md.

Verified against forge 0.2.0 (`pyproject.toml`; `forge version` still prints `v0.1.0` from
`forge/__init__.py` at this commit), commit `12a3167`, 2026-11-18. Source of truth: the Typer
definitions in `forge/cli.py` and `forge/catalog/cli.py`, cross-checked with
`forge <cmd> --help` from an editable install of that commit. Where forge's own docs
(`docs/cli.md`, module READMEs) disagree with the code, this file follows the code and says so.

## Path arguments (every dataset-taking command)

Resolution order (`_resolve_dataset_path` in `forge/cli.py`):

1. `hf://org/repo`, `hf://org/repo@revision`, `hf://org/repo:subset@revision`
   (also `huggingface://`). Reuses the HF Hub cache, then the LeRobot cache
   (`$LEROBOT_HOME` or `~/.cache/huggingface/lerobot/<org>/<repo>`), then downloads.
2. `s3://...` / `gs://...`: downloaded to a temp dir (needs the `[s3]` / `[gcs]` extra).
   `forge inspect` and `forge ingest` stream LeRobot v3 and Zarr instead of downloading.
3. An existing local path.
4. A registry id with no `/` (for example `pusht`, `droid_100`): see `forge registry list`.
5. A bare `org/repo` that is not a local path: treated as a Hub repo id.

A local directory named like a registry id wins over the registry. Downloads go to the
standard HF Hub cache unless `FORGE_CACHE_DIR` is set (then `$FORGE_CACHE_DIR/datasets`).
`docs/cli.md` still says `~/.cache/forge/datasets`; the code uses the HF cache by default.

Outputs must be local: `convert`, `filter`, `dedup` reject `s3://` / `gs://` outputs.
Catalogs can live on `s3://` / `gs://`.

`--sample N` on `quality`, `stats`, `segment` takes the **first N** episodes in reader order,
not a random sample (`docs/cli.md` says "random subset" for segment; the code breaks at
`i >= sample`).

## forge inspect PATH

| Option | Default | Meaning |
|---|---|---|
| `-f, --format` | auto | Format hint |
| `-o, --output` | `text` | `text` or `json` |
| `-d, --deep` | off | Scan all episodes |
| `-s, --samples` | 5 | Episodes sampled |
| `-g, --generate-config PATH` | | Write a YAML conversion template (for MCAP: a topic config) |
| `-q, --quick` | off | Hub datasets: metadata only, no download |

JSON output keys include `format`, `format_version`, `num_episodes`, `total_frames`,
`cameras`, `observation_schema`, `action_schema`, `has_timestamps`, `has_language`,
`language_coverage`, `has_rewards`, `inferred_fps`, `inferred_gripper_index`,
`missing_required` (for example `["fps", "robot_type"]`), `sample_episode_id`.

## forge lint PATH

| Option | Default | Meaning |
|---|---|---|
| `-e, --export PATH` | | JSON report |
| `--strict` | off | Non-zero exit on warnings too |
| `--max-examples` | 5 | Per-code episode examples printed (0 = all) |

Exit code is non-zero on any ERROR. Codes on the CLI (metadata) path: `dataset.empty` (error),
`task.missing`, `task.partial_coverage`, `task.generic`, `task.too_short` (< 25 chars),
`camera.none`, `camera.ambiguous_name`, `action.missing` (warn), `camera.low_resolution`
(< 640x480), `camera.too_few_views` (< 2) (info). The Python per-episode path
(`DatasetLinter().lint_episodes(episodes)`) adds `episode.too_short` (error) and
`task.too_long` (info, > 60 chars) among others. Thresholds: `forge/lint/config.py`
(`LintConfig`).

## forge stats PATH

| Option | Default | Meaning |
|---|---|---|
| `-p, --plot` | off | matplotlib distribution plots (opens a window) |
| `-o, --output PATH` | | JSON |
| `-s, --sample` | 0 (all) | First N episodes |
| `-q, --quality` | off | Also run quality metrics |
| `--gripper-dim` | | Gripper action index (for `--quality`) |
| `--fps` | 30.0 | Fallback FPS |

Reports episode length distribution, per-dim action/state min/max/mean/std, and coverage of
language instructions, success labels (`Episode.success`), and rewards.

## forge quality PATH

Proprioception (always) and video (opt-in) diagnostics. Scorer version `q-v2-proprio`;
video version `video-v2-integrity`.

Proprioception options:

| Option | Default | Meaning |
|---|---|---|
| `-g, --gripper-dim INT` | none | One gripper action column (-1 = last). Mutually exclusive with `--gripper-dims` |
| `--gripper-dims 6,13` | none | Several gripper action columns. Omit both for no gripper scoring |
| `--action-semantics` | `unknown` | `unknown`, `position`, `velocity`, `delta` |
| `--tracking-pairs 0:0,1:1` | none | action:state column pairs; requires `--action-semantics position` |
| `--state-dims 0,1,2` | all | Position-state columns used for motion diagnostics |
| `--state-scales a,b,...` | all 1 | Positive divisor for **every** original state column |
| `--pause-speed` | 0.001 | Pause threshold, scaled position units per second |
| `--event-min-duration` | 0.5 | Seconds; minimum pause/hold/stall length |
| `--tracking-error` | 0.05 | Scaled state units |
| `--tracking-lag` | 0.0 | Seconds of command-to-state latency |
| `--action-bounds min,max` | none | Same bounds for all action dims; enables saturation scoring |
| `-f, --fps` | 30.0 | Fallback FPS when timestamps are absent |
| `-s, --sample` | 0 | First N episodes |
| `-q, --quick` | off | Sets `--sample 50` (does not skip metrics, despite `docs/cli.md`) |
| `-w, --workers` | 1 | Process pool |
| `-e, --export PATH` | | Full JSON report |
| `--export-flagged PATH` | | `{flag: [episode_ids]}` JSON |
| `--visualize` | off | Serves the web viewer after scoring; **blocks until Ctrl+C** |

Per-dimension action limits exist only in the Python API: `QualityConfig(action_limits=((lo, hi), ...))`.

Video options (all require `--video` and the `[video]` extra):

| Option | Default | Meaning |
|---|---|---|
| `--video` | off | Camera integrity + pixel metrics |
| `--video-level` | `pixel` | `pixel` or `motion` (adds Farneback optical flow) |
| `--video-downscale` | 128 | Longest side of analysis frame (px) |
| `--video-stride` | 1 | Every Nth observation |
| `--video-max-frames` | 0 | Cap decode attempts per camera (0 = all) |
| `--video-cameras a,b` | all | Camera subset |
| `--video-fps` | episode FPS | Expected camera FPS override |
| `--video-drop-frames` | 3 | Missing observations in one gap that count as significant |
| `--video-gap-seconds` | 0.2 | Missing time beyond one frame that counts as significant |
| `--video-drop-fraction` | 0.05 | Estimated drop fraction that warns |
| `--video-freeze-seconds` | 0.5 | Near-identical span counted as a freeze candidate |

### Episode flags

Proprioception (`forge/quality/analyzer.py`, `metrics.py:composite_score`):

| Flag | Default trigger (from `QualityConfig`) |
|---|---|
| `nan_in_actions`, `nan_in_states`, `nan_in_timestamps` | Any non-finite value; score unavailable |
| `invalid_timestamps` | Timestamps not strictly increasing; score unavailable |
| `timestamp_gaps` | Any dt > 2x expected period |
| `sample_rate_mismatch` | Effective FPS differs from expected by > 10% |
| `timestamp_jitter` | dt std / dt mean > 0.1 |
| `dead_actions` | > 10% zero or repeated commands |
| `jerky` | LDLJ < -25 |
| `gripper_chatter` | > 2 transitions/s on a configured gripper column |
| `saturated` | > 30% occupancy near supplied action limits |
| `mostly_static` | > 50% of observed time in motion pauses |
| `low_entropy` / `high_entropy` | normalised action entropy < 0.2 / > 0.95 |
| `psd_high_band_chatter` | > 20% of action energy above 8 Hz |
| `tracking_stall_candidate` | Low state speed while mapped position command stays > tracking error away |

Video (`forge/quality/video/analyzer.py`): `blurry`, `over_exposed`, `under_exposed`,
`frozen_frames`, `video_camera_missing`, `video_decode_failure`, `video_invalid_pixels`,
`video_resolution_change`, `video_frame_drops`, `video_invalid_timestamps`, and with
`--video-level motion`: `no_motion`, `shaky`, `cut_detected`.

Events (localised, in `per_episode[].events` with `start_frame` inclusive, `end_frame`
exclusive, `start_time_s`, `end_time_s`, `channels`): `timestamp_gap`, `held_command`,
`motion_pause`, `direction_reversal`, `tracking_stall_candidate`, plus camera-scoped video
events.

### Report JSON (top level)

`dataset_path`, `num_episodes`, `overall_score`, `computed_at`, `subscores`, `per_episode`,
`flags`, `flagged_episodes`, `recommendations`. Each `per_episode` entry has `episode_id`,
`num_frames`, `scorer_version`, `overall_score` (nullable), `subscores`, `flags`, `events`,
`diagnostics`, `metric_status`, `provenance` (config + `fps` + `timebase`:
`stored` or `inferred_from_fps`), and metric fields (`ldlj`, `sparc`, `dead_fraction`,
`gripper_chatter_rate`, `overall_saturation`, `mean_entropy`, `psd_high_fraction`,
`state_action_consistency`, `static`, `timestamps`, `video`, ...).

Composite weights: smoothness 0.25; dead actions, gripper health, static fraction 0.15 each;
timestamp regularity, saturation, entropy 0.10 each, renormalised over available components.
Changing which components are available (gripper configured or not, bounds supplied or not)
changes the score scale; compare scores only across runs with the same config.

## forge filter SOURCE [OUTPUT]

Dry-run when OUTPUT is omitted. Writes the **same format** as the source, so the source format
needs a writer (`lerobot-v3`, `rlds`, `mcap`, `robodm`); otherwise forge raises "No writer for
format ... Convert to a writable format first".

| Option | Meaning |
|---|---|
| `-q, --min-quality F` | Drop if `overall_score < F`. **Episodes with a null score are kept** |
| `--exclude-flags a,b` | Drop if any flag matches (proprio or video flag names) |
| `--min-sharpness`, `--max-frozen`, `--max-overexposed`, `--max-underexposed` | Video thresholds; trigger live video analysis unless `--from-report` |
| `--min-motion` | Optical-flow px/frame; triggers motion-level analysis |
| `--include-episodes` / `--exclude-episodes` | Explicit ids (mutually exclusive) |
| `-r, --from-report PATH` | Use a `forge quality --export` JSON instead of re-analysing |
| `-g, --gripper-dim`, `-f, --fps`, `--action-bounds` | Config for live analysis only |
| `--video-stride` | Live video analysis stride |

Live analysis inside `filter` builds `QualityConfig(gripper_dim, fps, action_bounds)` only:
no `--gripper-dims`, `--state-dims`, `--state-scales`, tracking, or event thresholds. Use
`--from-report` with a report produced by a fully configured `forge quality` run. Episodes
missing from the report pass the quality filters with a logged warning.

The quality recommendation text can suggest `forge filter --max-gripper-chatter`; that flag
does not exist. Use `--exclude-flags gripper_chatter`.

## forge dedup SOURCE [OUTPUT]

Perceptual-hash duplicate detection on images (no model). Dry-run without OUTPUT.

| Option | Default | Meaning |
|---|---|---|
| `-m, --method` | `phash` | `phash`, `dhash`, `ahash` (`clip` is rejected) |
| `-t, --threshold` | 0.10 | Max normalised Hamming distance (0 = exact) |
| `--keyframes` | 16 | Uniformly spaced keyframes per episode per camera |
| `--hash-size` | 8 | Hash side; hash is hash_size^2 bits |
| `--cameras` | all | Camera subset |

Episode distance is the max over shared cameras of the mean keyframe distance; no shared
camera means never duplicate. Union-find clustering; keeps the earliest episode per cluster.
Writing reuses the filter engine, so the same writer requirement applies.

## forge visualize PATH

| Option | Default | Meaning |
|---|---|---|
| `-b, --backend` | `web` | `web`, `matplotlib`, `opencv`, `rerun` |
| `-c, --compare PATH` | | Second dataset side by side (not with opencv) |
| `-e, --episode` | 0 | Start episode |
| `-s, --samples` | 10 | Max episodes loaded |
| `--segment` | off | Overlay PELT phases |
| `--port` | 0 (auto) | Web viewer port |

`docs/cli.md` says matplotlib is the default; the code default is `web`. The web viewer
blocks while serving. Rerun recordings are saved under `~/.cache/forge/recordings/`.

## forge export-video PATH

`-o, --output`, `-e, --episode` (default 0), `-c, --camera`, `-a, --all` (all episodes to a
directory), `-f, --fps`, `-g, --grid`. Useful for non-interactive review: export the worst
episodes and look at them, or tile first frames.

## forge segment PATH

`--signal` (`observation.state` default; `qpos`, `action`, `joint_positions`,
`joint_velocities`), `-p, --penalty` (`bic` default, `aic`, or a number), `--cost-model`
(`rbf` default, `l2`, `l1`, `normal`, `ar`), `--min-segment-length` (10 frames),
`--normalize/--no-normalize`, `-s, --sample`, `-e, --export`, `--plot PNG`,
`--label/--no-label`, `-f, --format`. Needs the `[segment]` extra (`ruptures`).

## forge convert SOURCE OUTPUT

| Option | Default | Meaning |
|---|---|---|
| `--config PATH` | | YAML config (see format notes reference) |
| `-f, --format` | `lerobot-v3` | Target: `lerobot-v3`, `rlds`, `mcap`, `robodm` |
| `-s, --source-format` | auto | Source format |
| `--fps` | | Override FPS (also fills a missing FPS) |
| `-r, --robot-type` | | Robot type in output metadata |
| `-c, --camera src=dst` | | Camera rename, repeatable; overrides config |
| `--fail-on-error` | off | Stop at first failed episode (default: skip and continue) |
| `-v, --visualize` | off | Open a compare viewer after converting |
| `-n, --dry-run` | off | Show the plan, write nothing |
| `-w, --workers` | 1 | Parallel episode workers |

## forge hub / local / registry / demo / formats

- `forge hub [QUERY] [-a AUTHOR] [-n LIMIT] [-d REPO_ID]`: search or download Hub datasets.
  There is no upload or push command in forge; publish with `huggingface_hub` or LeRobot's
  tooling (see the `lerobot` skill).
- `forge local [-p PATH] [-a] [-o text|json] [--detect/--no-detect]`: list datasets in the HF cache.
- `forge registry list [-f FORMAT] [-e EMBODIMENT] [-t TAG] [--demo] [--html]`,
  `forge registry info ID`, `forge registry search QUERY`, `forge registry validate`.
- `forge demo [DATASET_ID] [--skip-quality]`: download, inspect and score a demo dataset.
- `forge formats`: read/write/visualise matrix.

## Catalog commands (data engine; install from source with `[catalog]`)

- `forge catalog init PATH [--exist-ok]`
- `forge ingest SOURCES... -c CATALOG [--batch-id] [--batch-size 200] [--strict] [--embed] [--embed-model]`
  Content-hash idempotent. Scores with the **default** `QualityAnalyzer()` (no gripper, bounds,
  or state scales; not configurable from the CLI), so catalog scores are not comparable to a
  configured `forge quality` run.
- `forge query SQL -c CATALOG [-f table|json|csv]` over `episodes`, `quality_scores`,
  `v_latest_quality` (plus `v_curation` and `embeddings`/`dedup_edges`/`curation_labels`
  after the relevant steps).
- `forge catalog stats -c CATALOG`
- `forge embed -c CATALOG [-m siglip-so400m] [--device auto] [--cameras] [--sample-hz 1.0] [--pooling mean|first-mid-last] [--batch-size 64]` (`[embed]` extra)
- `forge search [QUERY] -c CATALOG [--like EPISODE_ID] [-k 20] [--level episode|instruction] [--camera] [--save sel.json]`
- `forge catalog dedup -c CATALOG [-t 0.97] [-m MODEL]` (cosine over embeddings)
- `forge curate -c CATALOG [--where SQL] [--ids a,b] [--from sel.json] [--label approved] [--reason] [--dedup 0.97] [--dedup-policy keep-higher-quality|keep-longer|keep-first] [--by cli]`
- `forge studio -c CATALOG [-o studio.html] [-t 0.97] [--max-thumbnails 120]`

`episodes` columns: `episode_id`, `content_hash`, `source_uri`, `source_format`, `robot`,
`task`, `language_instruction`, `operator_id`, `scene_id`, `num_frames`, `duration_s`, `fps`,
`cameras`, `state_dim`, `action_dim`, `ingest_batch_id`, `ingested_at`, `extra`.
`operator_id` and `scene_id` are filled only if the reader puts them in episode metadata.
`quality_scores` columns: `episode_id`, `scorer_version`, `overall_score`, `ldlj`, `sparc`,
`dead_fraction`, `gripper_chatter_rate`, `joint_path_length`, `overall_saturation`,
`mean_entropy`, `psd_high_fraction`, `state_action_consistency`, `static_fraction`,
`jitter_ratio`, `flags`, `computed_at`.

## Python API (selected)

```python
import forge
info = forge.inspect("./ds")                      # DatasetInfo
forge.convert("./ds", "./out", target_format="lerobot-v3")

from forge.quality import QualityAnalyzer, QualityConfig
from forge.quality.video import VideoQualityConfig
qa = QualityAnalyzer(config=QualityConfig(gripper_dims=(6,), action_bounds=(-1, 1)),
                     video_config=VideoQualityConfig(sample_stride=4))
report = qa.analyze_dataset("./ds", sample=0)
report.to_json("quality.json")
eq = qa.analyze_episode_arrays("ep0", actions=A, states=S, timestamps=T, fps=30.0)

from forge.filter import FilterEngine, FilterConfig
FilterEngine(FilterConfig(from_report="quality.json", exclude_flags=["jerky"])).filter("./ds", output="./clean")

from forge.dedup import DedupEngine, DedupConfig
res = DedupEngine(DedupConfig(method="phash", threshold=0.10)).analyze("./ds")

from forge.lint import DatasetLinter
lint = DatasetLinter().lint_dataset("./ds")
```

Note `QualityAnalyzer.analyze_dataset` resolves `hf://` and `s3://`/`gs://` itself, but
`FormatRegistry.detect_format` and readers need a local path.
