# Evaluation Statistics and Reporting (reference)

Load this file when computing confidence intervals for a success rate, comparing two
policies, deciding how many trials to run, or writing up results. SKILL.md (Decision 5)
has the protocol and headline numbers; this file has the tested code and the template.

## Statistics (tested, standard library only)

```python
"""Success-rate statistics for robot policy evaluation. Python 3.8+, standard library only."""
from math import ceil, comb, sqrt
from statistics import NormalDist


def wilson_interval(successes: int, trials: int, confidence: float = 0.95) -> tuple:
    """Wilson score interval for a success rate. Behaves at 0/n and n/n, unlike p +/- z*se."""
    if trials <= 0 or not 0 <= successes <= trials:
        raise ValueError("need trials > 0 and 0 <= successes <= trials")
    z = NormalDist().inv_cdf(0.5 + confidence / 2)
    p = successes / trials
    denom = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def fisher_exact_p(s_a: int, n_a: int, s_b: int, n_b: int) -> float:
    """Two-sided Fisher exact test for two independent sets of trials."""
    total_s, total_n = s_a + s_b, n_a + n_b

    def pmf(k: int) -> float:  # P(policy A gets k of the successes) if both are equal
        return comb(n_a, k) * comb(n_b, total_s - k) / comb(total_n, total_s)

    observed = pmf(s_a)
    ks = range(max(0, total_s - n_b), min(n_a, total_s) + 1)
    return min(1.0, sum(pmf(k) for k in ks if pmf(k) <= observed * (1 + 1e-9)))


def mcnemar_exact_p(a_only: int, b_only: int) -> float:
    """Paired comparison on the SAME initial conditions: a_only = conditions where only A
    succeeded, b_only = only B succeeded. Ties (both/neither) carry no information."""
    n = a_only + b_only
    if n == 0:
        return 1.0
    tail = sum(comb(n, k) for k in range(min(a_only, b_only) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def trials_per_policy(p_a: float, p_b: float, alpha: float = 0.05, power: float = 0.8) -> int:
    """Approximate trials per policy needed to tell p_a from p_b (two-sided, normal approx)."""
    z_a, z_b = NormalDist().inv_cdf(1 - alpha / 2), NormalDist().inv_cdf(power)
    p_bar = (p_a + p_b) / 2
    root = z_a * sqrt(2 * p_bar * (1 - p_bar)) + z_b * sqrt(p_a * (1 - p_a) + p_b * (1 - p_b))
    return ceil(root ** 2 / (p_a - p_b) ** 2)


if __name__ == "__main__":
    for s, n in [(8, 10), (10, 10), (40, 50)]:
        lo, hi = wilson_interval(s, n)
        print(f"{s}/{n} = {s / n:.0%}, 95% CI [{lo:.0%}, {hi:.0%}]")
    print(f"8/10 vs 5/10:   p = {fisher_exact_p(8, 10, 5, 10):.2f}")
    print(f"40/50 vs 25/50: p = {fisher_exact_p(40, 50, 25, 50):.4f}")
    print(f"paired, A-only 9 vs B-only 2: p = {mcnemar_exact_p(9, 2):.3f}")
    print("trials/policy, 50% vs 80%:", trials_per_policy(0.5, 0.8))
    print("trials/policy, 70% vs 80%:", trials_per_policy(0.7, 0.8))
```

Output (cross-checked against scipy and statsmodels):

```
8/10 = 80%, 95% CI [49%, 94%]
10/10 = 100%, 95% CI [72%, 100%]
40/50 = 80%, 95% CI [67%, 89%]
8/10 vs 5/10:   p = 0.35
40/50 vs 25/50: p = 0.0031
paired, A-only 9 vs B-only 2: p = 0.065
trials/policy, 50% vs 80%: 39
trials/policy, 70% vs 80%: 294
```

What this means in practice:
- **8/10 is consistent with anything from about 49% to 94%.** 10/10 still allows 72%.
- **8/10 vs 5/10 is not evidence that A is better** (p = 0.35). The same rates over 50
  trials each are.
- Telling 50% from 80% needs ~40 trials per policy; 70% from 80% needs ~300. Small
  improvements on a good policy are expensive to prove; say so instead of claiming them.
- When both policies ran on the same initial conditions, the paired McNemar test uses
  only the conditions where they disagree and is usually more sensitive than Fisher.
- Comparing many policies or checkpoints at once inflates false positives; correct for
  it (Bonferroni is simple and conservative) or confirm the winner on fresh trials.

## Reporting template

```
Policy: act_pick_cube @ step 80k (ckpt sha 3f2a...), dataset v4 (212 episodes)
Split            Trials  Success  Rate   95% Wilson CI   Mean stage reached
in-distribution  50      41       82%    [69%, 90%]      3.7 / 4
held-out poses   30      14       47%    [30%, 64%]      2.9 / 4
Failure categories (in-dist): missed grasp 5, dropped in transit 3, timeout 1
Conditions: 25 poses x 2 start configs, interleaved with baseline, evaluator blind
```

## Why validation loss is a weak proxy

- The loss measures one-step (or one-chunk) prediction on demonstrator states. Rollouts
  depend on behaviour in states the policy itself causes.
- Multimodal demos make the loss noisy and irreducible; a lower loss can mean more
  mode averaging.
- robomimic evaluated every checkpoint by rollout because the lowest-validation-loss
  checkpoint was often far from the best; SIMPLER (Li et al. 2024) found validation MSE
  correlates poorly with real-world success.

Use validation loss to catch bugs (divergence, a loss that never drops, a large
train/val gap suggesting leakage or memorisation), not to pick the deployed checkpoint.
