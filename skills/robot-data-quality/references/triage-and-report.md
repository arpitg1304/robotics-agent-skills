# Triage Workflow, Report Template and Anti-Patterns in Full

Load this file when running a full audit end to end, writing the data-quality report, or
when you need the worked BAD/GOOD examples behind the anti-patterns in SKILL.md.

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
