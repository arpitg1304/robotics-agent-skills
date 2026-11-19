# Deployment Runtime for Chunked Policies (reference)

Load this file when writing the code that runs a trained policy on a robot: the control
loop, chunk execution, temporal ensembling, latency compensation, warm-up. SKILL.md
explains when to use each mode; this file has tested reference code.

Both snippets were run with Python 3.14 (numpy for the ensembler, stdlib for the async
runner). They are sketches of the timing logic, not a robot driver: plug in your own
`get_obs`, `send_action` and safety layer.

## Execution modes at a glance

- *Synchronous:* infer, execute `n` actions of the chunk, repeat. Simplest; pauses
  during inference unless inference is fast relative to `n` steps.
- *Temporal ensembling (ACT):* infer every step, average overlapping predictions.
  Smooth, but one inference per step and less reactive. Do not average across modes.
- *Asynchronous:* infer the next chunk while executing the current one, drop the
  actions that went stale during inference. Needed for large VLAs at high rates.
  SmolVLA ships async inference; real-time chunking (arXiv 2506.07339) makes
  flow/diffusion chunks join smoothly.

## 1. Measure the latency budget first

Write down, in milliseconds, for your actual hardware:

| Term | How to measure |
|---|---|
| Camera latency (exposure to frame in RAM) | Film a millisecond clock or rolling QR timestamp with the camera, compare to host clock (the UMI paper does this) |
| Preprocess + inference (p50 and p99) | Time 200 calls after warm-up; p99 matters, not the mean |
| Command to motion (execution latency) | Command a step, log measured joint position, find the lag |
| Control period | `1 / control_hz` |

`latency_steps = ceil((camera + inference_p99 + execution) * control_hz)`. In async mode
the refill threshold must exceed `latency_steps` with margin, or the queue runs dry. If
`latency_steps` is larger than the chunk you execute, the policy is acting on stale
observations: shorten inference (smaller model, fewer denoising steps, TensorRT,
lower resolution) or lower the control rate the policy outputs at and interpolate.

## 2. Synchronous chunk execution (simplest, start here)

```
loop:
    obs = get_obs()
    chunk = policy(obs)                 # robot holds still or coasts during inference
    for a in chunk[:n_execute]:         # n_execute <= chunk length, e.g. 8 of 16
        send(a); sleep_until_next_tick()
```

Fine when inference is a small fraction of `n_execute * period`. The visible symptom when
it is not is a pause every `n_execute` steps (stop-and-go motion). That pause is also a
distribution shift: demonstrations never contained it.

## 3. Temporal ensembling (ACT)

Query every step, keep every chunk that still covers the current step, and average their
predictions for this step. Smooths motion; costs one inference per control step and
reduces reactivity. With `m = 0.01` (the ACT reference value) the weights are close to
uniform over a 100-step chunk, so it acts as a moving average over past plans.

```python
import numpy as np


class TemporalEnsembler:
    """ACT-style temporal ensembling: query every step, average overlapping chunk
    predictions for the current step with weights exp(-m * i), i = 0 for the OLDEST
    prediction (Zhao et al. 2023; reference code uses m = 0.01)."""

    def __init__(self, chunk_size: int, m: float = 0.01):
        self.chunk_size, self.m = chunk_size, m
        self.pending: list[tuple[int, np.ndarray]] = []  # (step predicted at, chunk)

    def reset(self) -> None:  # call at episode start
        self.pending.clear()

    def step(self, t: int, chunk: np.ndarray) -> np.ndarray:
        """chunk: (chunk_size, action_dim) predicted at step t. Returns the action for step t."""
        assert chunk.shape[0] == self.chunk_size
        self.pending.append((t, chunk))
        self.pending = [(t0, c) for t0, c in self.pending if t - t0 < self.chunk_size]
        preds = np.stack([c[t - t0] for t0, c in self.pending])  # oldest first
        w = np.exp(-self.m * np.arange(len(preds)))
        return (w[:, None] * preds).sum(0) / w.sum()


if __name__ == "__main__":
    te = TemporalEnsembler(chunk_size=4, m=0.01)
    print(te.step(0, np.full((4, 2), 1.0)))   # [1. 1.]
    print(te.step(1, np.full((4, 2), 3.0)))   # ~[1.995 1.995]: old and new plan averaged
```

Do not ensemble across a mode switch: averaging "go left" and "go right" plans produces
"go straight into the obstacle". If the policy is multimodal (diffusion/flow), prefer
executing one sampled chunk at a time, or the async scheme below.

## 4. Asynchronous chunk execution

The control loop never blocks on the model. A worker starts inference when the queue
drops to `refill_at` actions; when the new chunk arrives, actions whose time already
passed are dropped (same idea as UMI's latency handling and SmolVLA's async inference).

```python
import threading
import time
from collections import deque


class AsyncChunkRunner:
    def __init__(self, policy, get_obs, send_action, hz: float, refill_at: int, hold_action):
        self.policy, self.get_obs, self.send_action = policy, get_obs, send_action
        self.dt, self.refill_at, self.hold_action = 1.0 / hz, refill_at, hold_action
        self.queue: deque = deque()
        self.lock = threading.Lock()
        self.step = 0                 # control steps executed so far
        self.busy = False
        self.starved_steps = 0        # steps with nothing to send (should stay 0)

    def _infer(self, obs, obs_step: int) -> None:
        chunk = self.policy(obs)                      # chunk[0] is the action for obs_step
        with self.lock:
            stale = self.step - obs_step             # actions already in the past
            self.queue = deque(chunk[stale:])        # replace the old tail with the fresh plan
            self.busy = False

    def warm_up(self, n_calls: int = 3) -> None:
        """Run inference synchronously before the loop starts: the first calls are slow
        (CUDA init, compilation, cache allocation) and the queue must not start empty."""
        for _ in range(n_calls):
            self.busy = True
            self._infer(self.get_obs(), self.step)

    def run(self, n_steps: int) -> None:
        next_tick = time.monotonic()
        for _ in range(n_steps):
            with self.lock:
                if not self.busy and len(self.queue) <= self.refill_at:
                    self.busy = True
                    threading.Thread(target=self._infer, args=(self.get_obs(), self.step),
                                     daemon=True).start()
                if self.queue:
                    action = self.queue.popleft()
                else:
                    action, self.starved_steps = self.hold_action(), self.starved_steps + 1
                self.step += 1
            self.send_action(action)
            next_tick += self.dt
            time.sleep(max(0.0, next_tick - time.monotonic()))


if __name__ == "__main__":
    sent = []

    def fake_policy(obs):            # 60 ms latency, 20-step chunk of absolute targets
        time.sleep(0.06)
        return [obs + i for i in range(20)]

    runner = AsyncChunkRunner(
        policy=fake_policy,
        get_obs=lambda: len(sent),   # observation = index of the next step to execute
        send_action=sent.append,
        hz=50, refill_at=8, hold_action=lambda: sent[-1] if sent else 0,
    )
    runner.warm_up()
    runner.run(150)
    jumps = [b - a for a, b in zip(sent, sent[1:]) if b - a != 1]
    print("steps:", len(sent), "starved:", runner.starved_steps, "jumps:", jumps)
    # steps: 150 starved: 0 jumps: []   (without warm_up: 3 starved steps, then a jump of 3)
```

Notes:

- `get_obs` must return the latest cached observation quickly; it runs under the lock.
- `hold_action` must hold the **measured current pose**, never zeros or a default pose.
- Replacing the tail with a fresh plan can still jump if the new plan disagrees with the
  old one (a different mode, or drift). Fixes: blend the first few actions of the new
  plan with the remaining old ones; for diffusion/flow policies use real-time chunking
  (Black, Galliker, Levine 2025, arXiv 2506.07339), which freezes actions that will
  execute during inference and inpaints the rest so the new chunk continues the old one;
  or Bidirectional Decoding (arXiv 2408.17355) for sampling-based selection across chunks.
- The `starved_steps` counter belongs in your telemetry. Non-zero in steady state means
  the latency budget is broken.

## 5. Safety layer (always, between policy and hardware)

Clamp every action before it reaches the driver, independent of the policy:

- Joint position limits with margin; workspace box for the end effector.
- Per-step delta limit (max change per tick) derived from the fastest motion seen in the
  demonstrations, plus a margin. A learned policy has no business exceeding it.
- Gripper force/current limit.
- Watchdog: if no new action arrives within N periods, hold position, then stop.
- NaN/inf check on every action: hold and alert.
- A human-held e-stop for every real rollout during evaluation.

See `robotics-design-patterns` (safety systems, watchdogs) for the structural patterns.
