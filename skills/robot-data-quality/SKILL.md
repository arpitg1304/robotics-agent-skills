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

### Episode-level (most important)

| Check | How (forge or recipe) | Starting-point threshold | Downstream failure it prevents |
|---|---|---|---|
| Length outliers | `forge stats` distribution; robust z-score recipe | abs robust z > 3.5 | Aborted takes teach "stop midway"; marathon takes teach dithering |
| Idle head/tail | `motion_pause` events; `active_span` recipe | head idle > 1 s or > 10% of episode | **Policy that never starts**: initial observation maps to "do nothing" |
| Idle mid-episode | `motion_pause`, `mostly_static` flag (> 50% observed time) | review if pause > 1-2 s not explained by the task | Hesitation, freezing mid-task |
| Gripper events | `--gripper-dims`, `gripper_chatter` (> 2 transitions/s); event-count recipe | pick-place: about 1 close + 1 open | Chatter -> oscillating grasp; regrasps -> hesitant grasps |
| Jerk and spikes | `jerky` (LDLJ < -25), `psd_high_band_chatter` (> 20% energy above 8 Hz); spike recipe | step > 8x median step | Jittery policy, controller faults, normalisation blow-up |
| Action-state alignment and lag | `--tracking-pairs`, `tracking_stall_candidate`; `estimate_lag` recipe | lag stable across sessions | Off-by-k shift makes the policy react late or "predict the past" |
| Timestamp monotonicity, gaps | `invalid_timestamps`, `timestamp_gaps` (dt > 2x period), `timestamp_jitter` (std/mean > 0.1), `sample_rate_mismatch` (> 10%) | any invalid; gaps > 1-2 frames | Wrong velocities, misaligned chunks, corrupted temporal context |
| NaN / non-finite | `nan_in_actions`, `nan_in_states`, `nan_in_timestamps` | any | Training NaNs, corrupted stats |
| Duplicates | `forge dedup` (pHash, 0.10 default) | inspect clusters | Leakage across splits, over-weighted takes |

Full episode-level table (held commands, truncated takes, success labels, saturation, joint
limits, camera-proprio sync), the **visual** table (blur, exposure, frozen frames, cuts, camera
mapping) and the **dataset-level** table (coverage, operator multimodality, task strings,
balance, splits, normalisation stats, units): `references/quality-checklist.md`.

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

## forge Workflow (core sequence)

```bash
forge inspect ./ds --output json > inspect.json     # format, dims, cameras, fps, language coverage
forge lint ./ds --export lint.json                  # hygiene; non-zero exit on errors
forge stats ./ds --output stats.json                # lengths, per-dim action/state stats, coverage
# Configured run for a 7-DoF arm + gripper at action column 7, actions in [-1, 1]:
forge quality ./ds \
  --gripper-dims 7 --action-bounds=-1,1 \
  --state-dims 0,1,2,3,4,5,6 \
  --pause-speed 0.01 --event-min-duration 0.5 \
  --fps 15 --workers 8 \
  --export quality.json --export-flagged flagged.json
forge quality ./ds --video --video-stride 4 --export quality.json
forge export-video ./ds -e 12 -o ep12.mp4 --grid     # non-interactive: produce a file to view
forge filter ./ds ./ds_clean --from-report quality.json \
  --exclude-flags nan_in_actions,nan_in_states,invalid_timestamps,video_decode_failure \
  --exclude-episodes 3,17,42
forge dedup ./ds                                     # dry-run: clusters
```

Install (`pip install forge-robotics` plus extras), path arguments (`hf://`, `s3://`), write
support, configuration rules for `forge quality`, reading the JSON export, video thresholds,
viewers, dedup details and the catalog: `references/forge-workflows.md`. Every flag, default
and JSON field: `references/forge-cli-reference.md`.

### forge behaviours that bite

- `overall_score` can be `null` (invalid data, irregular timebase, no evaluable motion). Treat
  null as "needs review", never as "fine".
- `--min-quality` **keeps** null-score episodes, and episodes missing from the report pass
  quality filters with a warning. Exclude unscored ids explicitly if you want them out.
- The quality recommendations may print `forge filter --max-gripper-chatter`; that flag does not
  exist. Use `--exclude-flags gripper_chatter`.
- `--visualize` and `forge visualize --backend web` **serve a local page and block until
  Ctrl+C**. In a non-interactive agent session, run them in the background or use
  `export-video`, `--plot`, and the JSON exports instead.
- YAML `fields:` mapping is parsed but **not applied**; YAML `video:` settings are not applied
  either (use `writer_config: {video_crf: 20}`).
- Without `--from-report`, filter re-scores with only `--gripper-dim`, `--fps` and
  `--action-bounds`: your state dims, scales, tracking pairs and event thresholds are lost, so
  flags differ from the report you reviewed.
- `--fps` is only a fallback when timestamps are missing (Zarr, HDF5, RLDS, RoboDM). If you
  leave it at 30 on a 10 Hz dataset, every duration and rate threshold is wrong.
- `--sample N` and `--quick` (= 50) take the **first** N episodes, not a random subset.
- Video flags never change the proprioception score; filter on them explicitly.
- `filter` and `dedup` write the source format, so convert first when the source is read-only.

Checks forge does not provide at commit 12a3167 (trimming, lag estimation, joint-limit
saturation, success detection, splits, and more): "Gaps" in `references/forge-workflows.md`.

## Conversion Pitfalls

forge converts through one canonical `Episode`/`Frame` model: one state vector, one action
vector, images, timestamps, language, and a few flags per frame. Anything else is dropped,
and the reader decides which source keys become state and action by name heuristics. forge
performs **no unit, rotation or frame-convention conversion**. Before converting, load
`references/forge-format-notes.md` (pitfalls table, per-format behaviour, a working
`conversion.yaml`, and the round-trip verification to run after every conversion).

## Triage Workflow (brief)

Run in this order; each step can invalidate the next.

1. **Pin provenance.** Work on a copy; never filter in place.
2. **Integrity.** `forge inspect --output json`, `forge lint`, `forge stats`. Stop and fix
   anything wrong here before scoring.
3. **Configure and score everything.** Then `--video --video-stride 4` on all cameras.
4. **Rank and look.** Watch the 10 worst, 5 median and 5 best episodes, plus every null-score
   episode.
5. **Custom checks** (`references/custom-checks.md`).
6. **Dataset level.**
7. **Duplicates.** `forge dedup` dry-run; review clusters.
8. **Decide.** For every episode: keep, trim (with indices), relabel, drop (with reason).
9. **Apply and verify.** Recompute stats, episode-level split by session, re-run steps 2-3.
10. **Report.**

Full step detail, the report template and how to state evidence:
`references/triage-and-report.md`.

## Anti-Patterns

1. **Filtering by score alone** (`--min-quality 7`): drops legitimate slow or waiting demos,
   keeps null-score (possibly corrupt) episodes.
2. **Splitting train/val by frame**: split by episode, or by session/scene, and dedup first.
3. **Trusting default units and FPS**: pass `--fps` and `--state-scales`.
4. **Treating flags as failures**: review before dropping.
5. **Computing normalisation stats before cleaning**: recompute after every filter.
6. **Converting without verifying**: run the round-trip check.
7. **Scoring a sample and deciding for the whole dataset**: use samples for calibration only.
8. **Mixing configurations in one comparison**.
9. **Blocking the agent on a viewer**: background it, or export videos and plots.
10. **Cleaning away the multimodality problem** (see `imitation-learning`).

BAD/GOOD examples and the reasoning for each: `references/triage-and-report.md`.

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

- `references/quality-checklist.md`: load for the full episode-level, visual (camera) and
  dataset-level check tables with forge flags, thresholds and the failure each prevents.
- `references/forge-workflows.md`: load when running any forge step in detail: install extras,
  path arguments, write support, `forge quality` configuration rules and examples (bimanual,
  tracking pairs), reading the export in Python, video checks, viewers, filter and dedup
  caveats, the catalog; also the list of checks forge does not provide yet ("Gaps").
- `references/forge-cli-reference.md`: load when you need any forge flag, default, flag
  trigger, JSON field, catalog table column, or Python API call not shown above.
- `references/forge-format-notes.md`: load before any `forge convert`, when a reader's output
  looks wrong (missing state, no language, black frames, wrong FPS), or to run the round-trip
  verification.
- `references/custom-checks.md`: load to run checks forge lacks: idle trim spans, length
  outliers, spikes, lag, joint-limit occupancy, gripper events, normalisation stats, coverage,
  task-string hygiene, episode-level splits.
- `references/triage-and-report.md`: load when running a full audit end to end, writing the
  data-quality report (template), or for the anti-pattern BAD/GOOD examples.

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
