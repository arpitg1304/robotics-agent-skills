# Custom Checks forge Does Not Run (NumPy Recipes)

Load this file when a quality question needs a check that `forge quality`, `forge lint`,
`forge stats` and `forge dedup` do not perform: idle head/tail trimming, episode-length
outliers, action-to-state lag, joint-limit saturation on *state*, normalisation-stat
outliers, gripper event counts per episode, task-string hygiene beyond `forge lint`,
initial-condition coverage, and episode-level train/val splits.

Every threshold below is a **starting point to calibrate on your robot and task**, not a
standard. Print the distribution before you pick a cut-off.

All recipes read episodes through forge's reader API, so they work on any format forge can
read. Tested against forge commit `12a3167` with the Push-T Zarr sample in the forge repo.

## Loading episodes as arrays

```python
import numpy as np
from forge.formats.registry import FormatRegistry

def load_arrays(path, max_episodes=None, fps_fallback=None):
    """Yield (episode, actions[T,A], states[T,S], t[T]) as float64 arrays.

    path must be a local path. For hf:// or s3:// sources, run any forge CLI
    command once (it caches/downloads), or use forge.io.localize for s3/gs.
    """
    fmt = FormatRegistry.detect_format(path)
    reader = FormatRegistry.get_reader(fmt)
    for i, ep in enumerate(reader.read_episodes(path)):
        if max_episodes is not None and i >= max_episodes:
            break
        frames = ep.load_frames()
        acts = np.array([f.action for f in frames], dtype=np.float64) if frames[0].action is not None else None
        sts = np.array([f.state for f in frames], dtype=np.float64) if frames[0].state is not None else None
        if frames[0].timestamp is not None:
            t = np.array([f.timestamp for f in frames], dtype=np.float64)
        else:
            fps = ep.fps or fps_fallback
            if fps is None:
                raise ValueError(f"{ep.episode_id}: no timestamps and no fps; pass fps_fallback")
            t = np.arange(len(frames)) / fps
        yield ep, acts, sts, t
        ep.clear_cache()
```

`ep.load_frames()` materialises the proprio arrays; images stay lazy (`LazyImage`) until
you call `.load()` on one, so this is cheap even for video datasets. If a reader returns
`None` for some frames' action or state, `np.array` will produce an object array; check
`acts.dtype` before doing maths.

## Episode-length outliers

```python
def length_outliers(lengths, k=3.5):
    """Robust z-score on episode length. k=3.5 is a common starting point."""
    x = np.asarray(lengths, dtype=float)
    med = np.median(x)
    mad = np.median(np.abs(x - med)) or 1.0
    z = 0.6745 * (x - med) / mad
    return np.flatnonzero(np.abs(z) > k), med, mad
```

Short outliers are usually aborted or truncated takes (check the last frames: did the gripper
release? is the object at the goal?). Long outliers are usually retries, operator hesitation,
or a missed "stop recording". Both deserve a look in the viewer before dropping.

## Idle head and tail (trim candidates)

Policies trained on long idle starts learn "do nothing" for the initial observation. This
finds the first and last frame where the robot is actually moving.

```python
def active_span(states, t, rel_speed=0.02, min_hold_s=0.3):
    """Return (first_active, last_active) frame indices.

    Speed per dim is normalised by that dim's p95 speed over the episode, so the
    threshold is unit-free. rel_speed=0.02 (2% of p95) is a starting point.
    """
    dt = np.diff(t)
    speed = np.abs(np.diff(states, axis=0)) / dt[:, None]
    scale = np.percentile(speed, 95, axis=0)
    scale[scale < 1e-12] = np.inf            # ignore channels that never move
    moving = np.max(speed / scale, axis=1) > rel_speed
    # require motion to persist for min_hold_s so a single jitter sample does not count
    hold = max(1, int(round(min_hold_s / np.median(dt))))
    run = np.convolve(moving.astype(int), np.ones(hold, int), mode="valid") == hold
    idx = np.flatnonzero(run)
    if idx.size == 0:
        return None, None                    # never moves: drop or inspect
    return int(idx[0]), int(idx[-1] + hold)
```

Report idle head and tail in seconds. As a starting point, flag head idle above 1 s or above
10% of the episode. Trim (keep a few frames of the static start so the policy still sees the
initial state) rather than drop. forge has no trim command; trim in your own writer or in
the training data loader by start/end index.

## Action spikes and jerk

```python
def action_spikes(actions, k=8.0):
    """Frames where a per-dim step exceeds k x that dim's median absolute step."""
    step = np.abs(np.diff(actions, axis=0))
    base = np.median(step, axis=0)
    base[base < 1e-9] = np.inf
    return np.flatnonzero(np.any(step / base > k, axis=1)) + 1
```

Cross-check against forge's `jerky` flag (LDLJ below -25 on state positions) and the
`psd_high_band_chatter` flag. Isolated single-frame spikes are often teleop dropouts or IK
flips (sudden joint jumps with a small Cartesian change): inspect the joint and EE traces
together.

## Action-to-state lag

forge's tracking diagnostics take a lag as input (`--tracking-lag`) but do not estimate it.
For absolute-position actions with the same units as state:

```python
def estimate_lag(actions, states, t, dims, max_lag_s=0.5):
    """Lag (seconds) maximising correlation between commanded and measured velocity."""
    dt = np.median(np.diff(t))
    va = np.diff(actions[:, dims], axis=0)
    vs = np.diff(states[:, dims], axis=0)
    best, best_c = 0, -np.inf
    for k in range(0, int(max_lag_s / dt) + 1):
        a = va[: len(vs) - k]
        s = vs[k:]
        c = np.corrcoef(a.ravel(), s.ravel())[0, 1]
        if c > best_c:
            best, best_c = k, c
    return best * dt, best_c
```

Expect a stable lag of a few control periods. A lag that differs between recording sessions
means the teleop or logging pipeline changed; a near-zero correlation means the "action" is
not a position command for those dims (velocity, delta, or a different frame), and tracking
checks are not applicable. Feed the estimate back into
`forge quality --action-semantics position --tracking-pairs ... --tracking-lag <s>`.

## Joint-limit saturation on state

`--action-bounds` covers commands only. For the measured state, use the URDF limits:

```python
def limit_occupancy(states, lower, upper, margin=0.02):
    """Fraction of frames within margin (fraction of range) of a joint limit, per dim."""
    lo, hi = np.asarray(lower), np.asarray(upper)
    m = margin * (hi - lo)
    near = (states <= lo + m) | (states >= hi - m)
    return near.mean(axis=0)
```

As a starting point, more than 5% of an episode near a limit deserves review: the policy
will learn to drive into the limit, and on a real arm that becomes a controller fault.

## Gripper events per episode

```python
def gripper_events(g, hysteresis=0.1):
    """Count close/open events with hysteresis on a gripper channel in [min, max]."""
    lo, hi = np.min(g), np.max(g)
    if hi - lo < 1e-6:
        return 0, 0
    x = (g - lo) / (hi - lo)
    state, closes, opens = x[0] > 0.5, 0, 0
    for v in x[1:]:
        if state and v < 0.5 - hysteresis:
            state, opens = False, opens + 1     # sign convention: check yours
        elif not state and v > 0.5 + hysteresis:
            state, closes = True, closes + 1
    return closes, opens
```

Check the sign convention first (is a larger value open or closed?). A pick-and-place should
show about one close and one open; zero closes is a failed or aborted grasp; many is regrasping
or chatter. forge's `gripper_chatter` flag only fires on rate (> 2 transitions/s by default),
so slow regrasps pass it.

## Normalisation statistics

```python
def norm_stat_report(all_actions):
    """all_actions: (N_frames_total, A). Compares min/max with robust quantiles."""
    q01, q99 = np.percentile(all_actions, [1, 99], axis=0)
    mn, mx = all_actions.min(axis=0), all_actions.max(axis=0)
    spread = np.where(q99 - q01 > 1e-12, q99 - q01, np.nan)
    return {
        "min_overshoot": (q01 - mn) / spread,   # >> 1 means a few outliers set the min
        "max_overshoot": (mx - q99) / spread,
        "std": all_actions.std(axis=0),
    }
```

If min/max normalisation is used (common for diffusion and ACT-style policies), a single
spike that is 10x the normal range squashes all real motion into a small slice of [-1, 1].
Find the episodes that contain the extreme values and fix or drop them before computing stats,
or use quantile-based stats. Recompute stats after any filtering: stats computed on the
unfiltered set still contain the dropped outliers.

## Initial-condition coverage

```python
def initial_state_spread(first_states, dims):
    """Per-dim range and a crude occupancy count on a 2D grid of two chosen dims."""
    x = np.asarray(first_states)[:, dims]
    rng = x.max(axis=0) - x.min(axis=0)
    H, _, _ = np.histogram2d(x[:, 0], x[:, 1], bins=5)
    return rng, int((H > 0).sum()), H
```

Robot initial state is only half the story; object initial positions usually live in images.
Sample the first frame of each episode (for example with `forge export-video -e <i>` or by
loading `frame.images[cam]` of frame 0) and tile them to see placement coverage. Gaps in that
grid are the regions where the policy will fail first.

## Task-string hygiene beyond `forge lint`

```python
import re
from collections import Counter

def task_hygiene(tasks):
    canon = [re.sub(r"\s+", " ", t.strip().lower().rstrip(".")) for t in tasks]
    raw, can = Counter(tasks), Counter(canon)
    merged = {c: n for c, n in can.items() if sum(1 for r in raw if
              re.sub(r"\s+", " ", r.strip().lower().rstrip(".")) == c) > 1}
    return can.most_common(), merged      # merged: strings that differ only by case/space/period
```

Also look for: one string covering two different tasks, two strings for one task, typos,
and per-episode strings that leak the outcome ("failed pick", "retry") into the instruction.

## Episode-level train/val split

```python
import hashlib

def split_episodes(episode_ids, groups=None, val_frac=0.1, salt="v1"):
    """Deterministic split by episode (or by group, e.g. session/operator/scene).

    Never split by frame: adjacent frames are near-identical, so a frame-level
    split leaks training data into validation.
    """
    keys = groups if groups is not None else episode_ids
    def bucket(k):
        return int(hashlib.sha1(f"{salt}:{k}".encode()).hexdigest(), 16) % 10_000 / 10_000
    val = {k for k in set(keys) if bucket(k) < val_frac}
    return [e for e, k in zip(episode_ids, keys) if k not in val], \
           [e for e, k in zip(episode_ids, keys) if k in val]
```

Split by session or scene (pass `groups`) when you want validation loss to say something about
generalisation to new setups. Record the salt and the resulting id lists with the dataset
revision. HDF5 robomimic files ship `mask/train` and `mask/valid`; forge's HDF5 reader does not
carry those masks through conversion, so save them before converting if you need them.
