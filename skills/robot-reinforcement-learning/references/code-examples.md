# Runnable Examples

Moved from SKILL.md. Every example is standard-library only and runs with `python3`.

## Example: Environment Skeleton

Dependency-free so it runs anywhere (gymnasium is not required; the API matches
`gymnasium.Env`: `reset() -> (obs, info)`, `step() -> (obs, reward, terminated, truncated, info)`).
Subclass `gymnasium.Env` and add `observation_space` / `action_space` in real code.

```python
import math
import random


class JointReachEnv:
    """1-DoF joint reaching task with a gymnasium-style API, standard library only."""

    sim_dt = 0.005          # physics step [s]
    decimation = 4          # physics steps per policy step -> 50 Hz policy
    max_episode_steps = 250 # 5 s episode -> time limit is a TRUNCATION
    action_scale = 1.6      # rad of target offset from default pose per unit action
    q_default = 0.0
    kp, kd, torque_limit = 20.0, 1.0, 5.0
    q_limit = 2.5           # rad; leaving this is a failure TERMINATION
    weights = {"track": 1.0, "action_rate": -0.01, "torque": -1e-4, "success": 0.5}

    def __init__(self):
        self.rng = random.Random()

    def reset(self, seed=None):
        if seed is not None:
            self.rng.seed(seed)
        self.q, self.qd = self.rng.uniform(-0.5, 0.5), 0.0
        self.goal = self.rng.uniform(-1.5, 1.5)
        self.inertia = 0.1 * self.rng.uniform(0.8, 1.2)   # domain randomisation
        self.prev_action, self.t, self.ep_terms = 0.0, 0, {}
        return self._obs(), {}

    def _obs(self):
        # Goal error, joint pos rel. to default, velocity, previous action (Markov for this task).
        return [self.goal - self.q, self.q - self.q_default, self.qd, self.prev_action]

    def step(self, action):
        a = max(-1.0, min(1.0, float(action)))           # clip in the env, not only in the policy
        q_target = self.q_default + self.action_scale * a  # absolute joint target around default pose
        tau = 0.0
        for _ in range(self.decimation):                 # PD runs at sim rate, policy at 50 Hz
            tau = self.kp * (q_target - self.q) - self.kd * self.qd
            tau = max(-self.torque_limit, min(self.torque_limit, tau))
            self.qd += tau / self.inertia * self.sim_dt
            self.q += self.qd * self.sim_dt
        self.t += 1

        err = self.goal - self.q
        success = abs(err) < 0.02 and abs(self.qd) < 0.05
        failed = abs(self.q) > self.q_limit
        terms = {                                         # weighted, per-step, logged separately
            "track": self.weights["track"] * math.exp(-err * err / 0.25),
            "action_rate": self.weights["action_rate"] * (a - self.prev_action) ** 2,
            "torque": self.weights["torque"] * tau * tau,
            "success": self.weights["success"] * float(success),
        }
        reward = sum(terms.values())
        for k, v in terms.items():
            self.ep_terms[k] = self.ep_terms.get(k, 0.0) + v
        self.prev_action = a

        # Success is NOT terminal here: with positive per-step reward, ending on success
        # would teach the policy to hover just outside the tolerance and keep collecting.
        terminated = failed                              # true end of the MDP: do NOT bootstrap
        truncated = (not terminated) and self.t >= self.max_episode_steps  # time limit: DO bootstrap
        info = {"reward_terms": terms, "success": success, "failed": failed}
        if terminated or truncated:
            info["episode_reward_terms"] = dict(self.ep_terms)
        return self._obs(), reward, terminated, truncated, info


if __name__ == "__main__":
    env = JointReachEnv()
    obs, _ = env.reset(seed=0)
    done = False
    while not done:
        # Scripted check: command the goal as the joint target (goal = err + q).
        obs, r, terminated, truncated, info = env.step((obs[0] + obs[1]) / env.action_scale)
        done = terminated or truncated
    print(env.t, terminated, truncated, info["success"],
          {k: round(v, 3) for k, v in info["episode_reward_terms"].items()})
```

Running it prints `250 False True True {...}`: a scripted controller reaches the goal, the
episode ends by truncation, and each reward term is reported separately. **Always run a
scripted or random policy through a new env before training**; it catches sign errors,
unreachable goals and broken resets in seconds instead of hours.

## Example: GAE With Correct Terminated / Truncated Handling

```python
def compute_gae(rewards, values, next_values, terminated, truncated, gamma=0.99, lam=0.95):
    """GAE over one environment's rollout of length T.

    next_values[t] = V(s_{t+1}) for the TRUE next state: at an episode end, the final
    observation (SAME_STEP autoreset: info["final_obs"]), never the next episode's reset obs.
    terminated[t] -> no bootstrap.  truncated[t] -> bootstrap, but stop the recursion.
    """
    T = len(rewards)
    adv, last = [0.0] * T, 0.0
    for t in reversed(range(T)):
        not_terminal = 1.0 - float(terminated[t])
        episode_continues = 1.0 - float(terminated[t] or truncated[t])
        delta = rewards[t] + gamma * not_terminal * next_values[t] - values[t]
        last = delta + gamma * lam * episode_continues * last
        adv[t] = last
    returns = [a + v for a, v in zip(adv, values)]
    return adv, returns


if __name__ == "__main__":
    # Reward 1 forever -> true V = 1/(1-gamma) = 100. A 3-step time limit cuts the episode.
    V = 100.0
    _, ok = compute_gae([1.0] * 3, [V] * 3, [V] * 3, [0, 0, 0], [0, 0, 1], gamma=0.99, lam=1.0)
    _, bug = compute_gae([1.0] * 3, [V] * 3, [V] * 3, [0, 0, 1], [0, 0, 0], gamma=0.99, lam=1.0)
    print("truncation bootstrapped  :", [round(x, 2) for x in ok])    # [100.0, 100.0, 100.0]
    print("time-out treated terminal:", [round(x, 2) for x in bug])   # [2.97, 1.99, 1.0]
```

rsl_rl (v5.5.2 `ppo.py`) approximates the bootstrap by adding `gamma * V(s_t)` (the value of
the current state, not of the final observation) to the reward on time-out steps. That is a
reasonable approximation when consecutive states have similar value; the exact version needs
the final observation.

## Example: Running Observation Normaliser

```python
import math


class RunningMeanStd:
    """Per-dimension running mean/variance with a batched (Chan et al.) update."""

    def __init__(self, dim, eps=1e-4, clip=5.0):
        self.mean, self.var, self.count = [0.0] * dim, [1.0] * dim, eps
        self.clip = clip

    def update(self, batch):  # batch: list of observation vectors
        n = len(batch)
        if n == 0:
            return
        for i in range(len(self.mean)):
            col = [x[i] for x in batch]
            b_mean = sum(col) / n
            b_var = sum((c - b_mean) ** 2 for c in col) / n
            delta, tot = b_mean - self.mean[i], self.count + n
            m2 = self.var[i] * self.count + b_var * n + delta * delta * self.count * n / tot
            self.mean[i] += delta * n / tot
            self.var[i] = m2 / tot
        self.count += n

    def normalize(self, x):
        return [max(-self.clip, min(self.clip, (xi - m) / math.sqrt(v + 1e-8)))
                for xi, m, v in zip(x, self.mean, self.var)]

    def state_dict(self):  # ship this WITH the policy weights; deployment must use the same stats
        return {"mean": list(self.mean), "var": list(self.var), "count": self.count}
```

Use NumPy or torch in real code. The failure this prevents is not numerical, it is
operational: update statistics only during training, freeze them for evaluation, and export
them with the checkpoint (or bake them into the ONNX graph). A policy deployed without its
normaliser outputs confident garbage.

## Example: IQM, Bootstrap CI and Wilson Interval

For the evaluation protocol in SKILL.md section 8.

```python
import math
import random


def iqm(scores):
    """Interquartile mean: mean of the middle 50% (rliable's recommended aggregate)."""
    s = sorted(scores)
    lo = int(0.25 * len(s))  # same cut as scipy.stats.trim_mean(scores, 0.25)
    mid = s[lo:len(s) - lo]
    return sum(mid) / len(mid)


def bootstrap_ci(scores, stat=iqm, reps=2000, alpha=0.05, seed=0):
    """Percentile bootstrap CI over seeds. Use rliable for stratified multi-task versions."""
    rng = random.Random(seed)
    boots = sorted(stat([rng.choice(scores) for _ in scores]) for _ in range(reps))
    return boots[int(alpha / 2 * reps)], boots[int((1 - alpha / 2) * reps) - 1]


def wilson_ci(successes, trials, z=1.96):
    """95% Wilson interval for a real-robot success rate (better than +-1.96*sqrt(p(1-p)/n))."""
    p = successes / trials
    denom = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return centre - half, centre + half


if __name__ == "__main__":
    per_seed = [0.92, 0.88, 0.95, 0.31, 0.90, 0.93, 0.89, 0.94, 0.12, 0.91]
    print("mean", round(sum(per_seed) / len(per_seed), 3), "IQM", round(iqm(per_seed), 3),
          "95% CI", tuple(round(c, 3) for c in bootstrap_ci(per_seed)))
    print("18/20 real trials:", tuple(round(c, 3) for c in wilson_ci(18, 20)))
```

Two failed seeds pull the mean to 0.775 while the IQM is 0.905; report both, and investigate
the failures rather than dropping them.
