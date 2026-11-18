# Further Failure Modes (reference)

Load this file when a symptom is not in the Failure Modes table in SKILL.md. Same
columns as that table.

| Symptom | Likely cause | Fix |
|---|---|---|
| Fails in different lighting, background, or with people nearby | Overfitting to appearance | Vary conditions during collection; colour jitter; crop distractors; sim co-training |
| Success drops after someone bumps a camera | Camera pose shift | Rigid mounts; pre-session overlay check; shift augmentation; rely more on wrist camera; relative actions |
| Jerk at every chunk boundary | Successive chunks disagree; synchronous pauses | Temporal ensembling, blending, real-time chunking; overlap execution with inference |
| Sim success high, real success zero | Visual or dynamics gap; mismatched cameras or control rate | Match camera placement and rate; co-train with real data; domain randomisation (`isaac-sim`, `mujoco`) |

## Overfitting signs

- Success near demonstrated start states, failure a few centimetres away.
- The robot reaches to where the object was in most demos rather than where it is.
- Training loss near zero, validation loss rising early, rollouts unchanged or worse.
- Fixes: more start-state diversity, augmentation, smaller model or fewer epochs, proprio
  noise or dropout. More of the same demos rarely helps.
