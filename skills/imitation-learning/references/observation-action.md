# Observation and Action Design (reference)

Load this file when choosing the action representation or setting image resolution
and cropping. SKILL.md (Decision 2) has the rules; this file has the comparison table
and image details.

## Action space

| Action | Pros | Cons | Use when |
|---|---|---|---|
| Absolute joint positions | Matches leader-follower teleop exactly; no IK at runtime; ACT reported delta joints did worse | Embodiment-specific; does not transfer across robots | Leader-follower arms, ACT, single robot |
| Delta joint positions | Small numeric range | Errors integrate; drift | Rarely the best choice |
| Absolute EEF pose | Robot-agnostic | Needs accurate calibration of base frame; IK at runtime | Calibrated cells, cross-robot datasets |
| Delta EEF (per step) | Natural for SpaceMouse; robust to base offset | Accumulates error over the chunk | Low-rate VLA control on SpaceMouse-style data (OpenVLA was trained on single-arm EEF control data) |
| Relative trajectory (each action relative to EEF pose at chunk start) | Robust to calibration and camera shift; no per-step accumulation | Needs good state estimation of the current pose | Handheld or cross-embodiment data; UMI found it beat delta and absolute; GR00T N1.7 uses relative EEF |

## Proprioception vs images

- If the policy ignores images and replays a trajectory from proprio, you get a policy
  that reaches for where the object usually is. Symptom: works only at demonstrated
  positions. Remedies: more position diversity, proprio dropout or noise during training.

## Images

- Resolution is modest in practice: Diffusion Policy's real tasks used 320x240 per
  camera; ACT streamed 480x640; VLAs fix the size in their processor (check the model
  card). Higher resolution mostly costs latency; crop before you upscale.
- Random crop at train time, centre crop at test time is the usual recipe. Diffusion
  Policy cropped 320x240 to 288x216; OpenVLA's LIBERO setup used 90%-area random crops
  and its README warns to centre crop at evaluation.
- Crop away regions that change but do not matter (people walking by, monitors).
