# Policy Landscape for Imitation Learning (reference)

Load this file when choosing between policy families or pretrained VLAs, when a user
asks about licences, GPU requirements, or published default hyperparameters, or before
quoting any number about a specific model. SKILL.md has the decision logic; this file
has the facts and where they came from.

**Model landscape checked 2026-11-18; verify versions before relying on them.** This
area moves monthly. Licences in particular change between releases and sometimes
disagree between a README, a model card and the LICENSE file in the same repository.
Before a commercial deployment, read the LICENSE file in the exact checkpoint repo you
download, not a blog post or this table.

## Contents

1. From-scratch policy families (ACT, Diffusion Policy, BC baselines), family notes,
   licences, VLA fine-tuning
2. Pretrained generalist policies and VLAs
3. Published defaults worth knowing
4. Results that change practice
5. Sources

---

## 1. From-scratch policy families

| Family | Output head | Typical size | Train compute (heuristic) | Use when | Avoid when |
|---|---|---|---|---|---|
| BC MLP/CNN (single step) | Regression (L1/L2) | 1-30M | Hours on one consumer GPU | Baseline; state-based tasks; checking the pipeline | Multimodal demos; long precise motions (per-step error compounds) |
| BC-RNN / GMM (robomimic) | GMM over actions | ~10M | Hours | State-based sim tasks; reproducing robomimic results | High-rate real visuomotor control |
| ACT | CVAE + transformer decoder, L1 regression on chunks of absolute joint targets | ~80M (paper) | Paper: ~5 h on one 11 GB RTX 2080 Ti for ~50 demos; ~10 ms inference | Fine bimanual or single-arm tasks, ~50-200 demos, joint-space teleop (leader-follower) | Strongly multimodal demos (CVAE helps only partly); language-conditioned multi-task |
| Diffusion Policy (CNN U-Net or transformer) | DDPM/DDIM denoising of action sequences | Larger than ACT; depends on backbone | Longer than ACT (paper trains some tasks for 1000 epochs); one GPU is enough | Multimodal demos, contact-rich tasks, position control at ~10 Hz with interpolation | Hard latency budgets without DDIM / fewer steps; tiny datasets with no augmentation |
| Flow-matching heads | Flow matching on action chunks | ~100M (SmolVLA) to ~300M (pi0) action experts inside VLAs | n/a (usually part of a VLA) | Same niche as diffusion, fewer integration steps at inference | n/a |

Sizes and compute in this table are order-of-magnitude heuristics from the papers'
reported setups, not guarantees.

### Family notes, licences, VLA fine-tuning

- **Behaviour cloning MLP/CNN**: single-step regression. Fine for state-based tasks and
  for proving the pipeline works. Collapses on multimodal demos (averages modes).
- **ACT**: CVAE plus transformer predicting chunks of absolute joint targets with L1 loss.
  ~80M parameters, ~5 h training on one 11 GB GPU in the paper. Strong default for
  leader-follower data.
- **Diffusion Policy**: denoises action sequences; models multimodal action
  distributions; slower inference (use DDIM with ~10 steps). Start with the CNN variant.
- **Flow-matching heads**: the same niche as diffusion with fewer integration steps;
  used inside pi0, SmolVLA and GR00T. Recent evidence suggests the benefit of
  generative heads comes mostly from iterative refinement rather than multimodality
  (arXiv 2512.01809), so a well-built regression chunk policy is a fair baseline.
- **VLAs**: pretrained vision-language backbones plus an action head. Worth it for
  language conditioning, multi-task, and generalisation to new objects and scenes. Not
  automatically better on a narrow single task, and inference is heavier.

Licences differ (MIT, Apache-2.0, Gemma terms, NVIDIA licences, some non-commercial) and
can disagree between a README and the checkpoint's LICENSE file. Check
the licence columns in section 2 and then the actual checkpoint repo before a product
decision.

Fine-tuning VLAs: start with the provider's recipe (openpi configs, GR00T's "few
thousand steps" guidance, LeRobot defaults). LoRA if memory-bound; full fine-tuning
when you have the GPU and enough data. Keep the action normalisation and camera keys
exactly as the fine-tuning config expects.

## 2. Pretrained generalist policies and VLAs

| Model | Released | Params | Action head | Code licence | Weights licence | GPU guidance (from source) |
|---|---|---|---|---|---|---|
| Octo (Small / Base) | May 2024 (v1.5 later) | 27M / 93M | Diffusion head | MIT | MIT | Paper: fine-tuning ~100 demos for 50k steps took ~5 h on one 24 GB A5000 |
| OpenVLA | Jun 2024 | 7B (card shows ~8B) | 256-bin discrete tokens, autoregressive | MIT | MIT (Llama 2 backbone terms also apply) | Paper: ~15 GB bf16 inference, ~6 Hz on RTX 4090; LoRA fine-tune 10-15 h on one A100. README: LoRA default ~72 GB, works down to ~27 GB with smaller batch |
| OpenVLA-OFT / OFT+ | Feb 2025 | 7B base | Parallel decoding, continuous actions, L1 regression, action chunks; OFT+ adds FiLM | MIT (repo) | Per checkpoint; check | Paper reports 25-50x faster action generation than OpenVLA; ALOHA at 25 Hz |
| pi0 | Oct 2024 paper, weights Feb 2025 (openpi) | 3.3B (PaliGemma 3B + ~300M action expert) | Flow matching, H=50 chunks | Apache-2.0 (openpi) | openpi ships `LICENSE_GEMMA.txt`; Gemma terms apply to the backbone-derived weights | openpi README: inference > 8 GB, LoRA fine-tune > 22.5 GB, full fine-tune > 70 GB |
| pi0-FAST | Jan 2025 | ~3B | Autoregressive over FAST (DCT-compressed) action tokens | Apache-2.0 (openpi) | As pi0 | As pi0; FAST paper reports up to 5x faster training than diffusion VLAs; autoregressive decoding adds inference latency per chunk |
| pi0.5 | Apr 2025 paper, weights Sep 2025 (openpi `pi05_base`) | ~3B class (PaliGemma-based like pi0; exact count not verified) | Flow matching head (openpi supports flow head only for pi0.5) | Apache-2.0 (openpi) | As pi0 | As pi0 |
| pi0.6 / pi*0.6 (Recap) | Nov 2025 paper (arXiv 2511.14759) | 5B VLM + action expert | Flow matching; advantage-conditioned | n/a | **No official open weights found as of the check date**; community reimplementations exist | n/a |
| SmolVLA | Jun 2025 | ~450M (~100M action expert) | Flow matching transformer, chunks of 50 | Apache-2.0 (LeRobot) | Apache-2.0 (`lerobot/smolvla_base`) | Paper: trainable on one GPU, deployable on consumer GPU or CPU; async inference built in |
| GR00T N1 | Mar 2025 | 2B | VLM (System 2) + diffusion transformer (System 1) | Apache-2.0 | NVIDIA licence (check card) | Humanoid-focused; superseded |
| GR00T N1.5 / N1.6 | May 2025 / Dec 2025 | 3B | Flow-matching DiT | Apache-2.0 | NVIDIA non-commercial licence (per NVIDIA developer page) | Branches `n1d5`, `n1d6` of Isaac-GR00T |
| GR00T N1.7 | 2026 (current `main`) | 3B | Flow-matching DiT, Cosmos-Reason2-2B (Qwen3-VL) backbone, relative EEF actions | Apache-2.0 | NVIDIA Open Model License (README says commercially usable; GitHub issue #803 reports the checkpoint LICENSE file still reads non-commercial - verify) | README: inference 16 GB+; fine-tune 40 GB+ (H100/L40 recommended); examples use a few thousand steps, global batch 32. Backbone is a gated HF model |
| GR00T 2 (N2) | Announced CoRL, Nov 2026 | 2B (whitepaper) | World-action model (predicts future observations and actions) | n/a | **Not released at check date**; reports say weights planned for Dec 2026, licence unannounced | n/a |

Other open VLAs exist (for example RDT, X-VLA, and many 2026 world-action models). Treat
anything not in this table as unverified until you have read its model card.

### Picking among them (heuristic)

- **Single task, one robot, 50-300 demos, no language:** train ACT or Diffusion Policy
  from scratch first. It is cheaper, faster at inference, and the baseline you need anyway.
- **Many tasks, language instructions, or you need generalisation to new objects/scenes:**
  fine-tune a VLA. SmolVLA if compute is tight; pi0/pi0.5 (openpi) or GR00T N1.7 with a
  24-80 GB GPU.
- **Low-cost arm (SO-101 class) with LeRobot:** ACT, Diffusion, SmolVLA, pi0/pi0.5 and
  GR00T are reachable through LeRobot; see the `lerobot` skill for commands.
- **Commercial product:** licence decides before accuracy does. MIT/Apache weights
  (Octo, OpenVLA, SmolVLA) are simplest; Gemma-derived and NVIDIA-licensed weights need
  a legal read.
- **Fine-tune vs from scratch:** a VLA fine-tune is not automatically better on a narrow
  task. Evaluate both on the same fixed initial-condition set (SKILL.md, Evaluation).

## 3. Published defaults worth knowing

These are what the original papers used. They are starting points, not tuned values for
your robot. Library defaults (LeRobot, openpi) may differ; check the config you run.

**ACT** (Zhao et al. 2023, Table III): L1 loss on absolute target joint positions (delta joint targets did worse), chunk size 100, KL weight beta 10, lr 1e-5,
batch 8, 4 encoder / 7 decoder layers, hidden 512, 8 heads, dropout 0.1. Data at 50 Hz,
50 demos per task (100 for one task), 8-14 s episodes. Chunk-size ablation: success
rises steeply from k=1 to k=100, dips slightly beyond. Temporal ensembling weights
`w_i = exp(-m * i)` with `w_0` the oldest prediction; reference code uses `m = 0.01`
and queries the policy every step when ensembling is on.

**Diffusion Policy** (Chi et al. 2023, Tables 7-8): observation horizon To=2, prediction
horizon Tp=16, execution horizon Ta=8 for most tasks; lr 1e-4; batch 64 for image
inputs; DDIM with 100 training and 10 inference steps gave ~0.1 s latency on an RTX 3080.
Position control beat velocity control. Real robot commands at 10 Hz linearly
interpolated to 125 Hz. Recommendation: start with the CNN backbone; the transformer is
more sensitive to hyperparameters.

**pi0** (Black et al. 2024): chunks H=50, 10 flow integration steps, 73 ms on-board
inference on an RTX 4090; post-training used 5 h of data for the simplest tasks and
100+ h for the hardest.

**Octo**: 2 frames of history, chunk 64 for ALOHA fine-tuning with 12 actions executed
per inference (receding horizon).

**SmolVLA**: chunk 50; ablations found chunk sizes 10-50 balance reactivity and
performance. Async inference sends a new observation when the remaining queue drops
below a fraction `g` of the chunk.

## 4. Results that change practice

- **Validation loss is a poor checkpoint selector.** robomimic (Mandlekar et al. 2021)
  evaluates every checkpoint by rollout and lists offline policy selection as a core
  lesson; the policy with lowest validation loss can be far worse than the best one.
  SIMPLER (Li et al. 2024) also reports validation MSE correlates poorly with real
  performance.
- **Diversity beats volume.** Lin et al. 2024 (Data Scaling Laws in IL): diversity of
  environments and objects matters far more than demos per environment; beyond a
  threshold per environment/object, more demos add little.
- **Generative heads may help through iteration, not multimodality.** Pan et al. 2025
  ("Much Ado About Noising", arXiv 2512.01809) report that a two-step supervised
  regression policy roughly matches flow policies on their benchmarks. Do not assume
  diffusion is required; do not assume it is useless either. Test both if latency matters.
- **Action representation matters for portable data.** UMI (Chi et al. 2024) found
  relative-trajectory actions (relative to the pose at chunk start) beat delta (error
  accumulates) and absolute (frame calibration errors) on their task; GR00T N1.7 also
  moved to relative EEF actions for cross-embodiment transfer.
- **L1 regression collapses modes.** The OpenVLA-OFT authors note L1 regression may
  learn only a median mode when demonstrations are multimodal.
- **Sim co-training helps even with a visual gap.** Maddukuri et al. 2025 report an
  average 38% real-world improvement from adding sim data, with notable sim-real
  differences.
- **Real-world evaluation at scale.** TRI's LBM study (2025) ran 50 real rollouts per
  task per policy per condition, blind and randomised, with matched initial conditions
  and sequential paired tests. That is the bar for claims of one policy beating another.

## 5. Sources

Papers (arXiv IDs verified 2026-11-18):

- ACT / ALOHA: Zhao et al., *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*, arXiv 2304.13705; code github.com/tonyzhaozh/act
- Mobile ALOHA: Fu et al., arXiv 2401.02117
- Diffusion Policy: Chi et al., arXiv 2303.04137
- robomimic: Mandlekar et al., *What Matters in Learning from Offline Human Demonstrations for Robot Manipulation*, arXiv 2108.03298
- Octo: arXiv 2405.12213; github.com/octo-models/octo
- OpenVLA: Kim et al., arXiv 2406.09246; huggingface.co/openvla/openvla-7b; github.com/openvla/openvla
- OpenVLA-OFT: Kim, Finn, Liang, arXiv 2502.19645; openvla-oft.github.io
- pi0: Black et al., arXiv 2410.24164; FAST: arXiv 2501.09747; pi0.5: arXiv 2504.16054; pi*0.6: arXiv 2511.14759
- openpi: github.com/Physical-Intelligence/openpi (README "Updates" section and hardware table)
- Real-time chunking: Black, Galliker, Levine, arXiv 2506.07339
- Bidirectional Decoding: Liu et al., arXiv 2408.17355
- SmolVLA: arXiv 2506.01844; huggingface.co/lerobot/smolvla_base
- GR00T N1: arXiv 2503.14734; github.com/NVIDIA/Isaac-GR00T (README, License section); huggingface.co/nvidia/GR00T-N1.7-3B; developer.nvidia.com/isaac/gr00t
- GR00T 2: research.nvidia.com/labs/gear/gr00t-2 (whitepaper); release status from news coverage, unconfirmed
- Data scaling: Lin et al., *Data Scaling Laws in Imitation Learning for Robotic Manipulation*, arXiv 2410.18647
- Generative vs regression: *Much Ado About Noising*, arXiv 2512.01809
- UMI: Chi et al., arXiv 2402.10329
- Sim-and-real co-training: Maddukuri et al., arXiv 2503.24361
- MimicGen: Mandlekar et al., arXiv 2310.17596
- TRI LBM evaluation: arXiv 2507.05331
- DAgger: Ross, Gordon, Bagnell, arXiv 1011.0686; HG-DAgger: Kelly et al., arXiv 1810.02890
- Causal confusion: de Haan, Jayaraman, Levine, arXiv 1905.11979; copycat problem: Wen et al., arXiv 2010.14876
