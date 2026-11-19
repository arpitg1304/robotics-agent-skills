# Quality Checklist: Full Tables

Load this file when auditing visual (camera) quality, dataset-level coverage and splits, or
when you need an episode-level check not in the compact table in SKILL.md.

Thresholds are **starting points to calibrate on your robot, task and FPS**, not standards.
Print the distribution first, then choose the cut. Recipes for checks forge does not run are in
`references/custom-checks.md`.

## Episode-level

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

## Visual

| Check | forge flag (needs `--video`) | Failure it prevents |
|---|---|---|
| Blur | `blurry` (> 30% of frames below var-of-Laplacian 100 at 128 px) | Policy ignores a camera or learns motion-blur cues |
| Exposure | `over_exposed`, `under_exposed` (> 25% of frames) | Object invisible in some sessions; lighting overfit |
| Frozen / repeated frames | `frozen_frames`, `video_frame_drops`, `video_decode_failure`, `video_camera_missing` | Stale observations; also black-frame substitution from bad decoders |
| Shot cuts / camera bumps | `cut_detected`, `shaky` (`--video-level motion`) | Viewpoint shift mid-episode |
| Occlusion | not in forge: review wrist/scene views manually | Policy acts on occluded state |
| Camera moved between sessions | not in forge: compare mean first frame per session | Distribution shift by session; works Monday, fails Tuesday |
| Wrong camera mapping | not in forge: tile one frame per camera key per session | Wrist and scene swapped; policy trained on nonsense |

## Dataset-level

| Check | How | Failure it prevents |
|---|---|---|
| Initial-condition and object coverage | first-frame tiles; `initial_state_spread` recipe | Fails outside the demonstrated region |
| Operator consistency / multimodality | split metrics by operator or session; watch a few per operator | **Mode averaging**: two strategies averaged into one that fails both (see `imitation-learning`) |
| Task-string hygiene | `forge lint` (`task.missing`, `task.generic`, `task.too_short` < 25 chars); `task_hygiene` recipe | Language-conditioned policy ignores or confuses instructions |
| Task and class balance | `forge query` on a catalog, or count tasks | Rare task never learned |
| Train/val split by episode | `split_episodes` recipe, group by session | Frame-level leakage: val loss looks great, robot fails |
| Normalisation stats | `norm_stat_report` recipe; recompute after filtering | One spike squashes all real motion into a sliver of [-1, 1] |
| Units and conventions | `forge stats` per-dim ranges | Degrees vs radians, gripper 0-1 vs 0-100, mixed datasets |
