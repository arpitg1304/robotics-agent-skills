# Data Collection for Imitation Learning (reference)

Load this file when planning a demonstration campaign, choosing a teleoperation
interface, or deciding how many demos to record. SKILL.md (Decision 1) has the
essential guidance; this file has the supporting tables and the per-session checklist.

## Demo budget

**What a demo budget buys (heuristic, single arm, one task, from-scratch policy):**

| Demos | Realistic outcome |
|---|---|
| 10-20 | Pipeline check. Expect success only near demonstrated start states. |
| 50 | ACT-style precise tasks in a fixed scene (the ACT paper used 50 per task, 10-20 min of data). Fragile to new positions, lighting, distractors. |
| 100-300 | Usable within a randomised start-state region; still a single scene. Octo's fine-tuning setups used ~100. |
| 1000+ or many hours across scenes | Needed for new environments and objects. pi0 post-training used 5 h for simple tasks, 100+ h for complex ones. |

## Teleoperation modality

| Modality | Good for | Weaknesses |
|---|---|---|
| Leader-follower (ALOHA, SO-101 pair) | Joint-space actions, bimanual, fast dexterous motion, cheap | Needs a kinematically matched leader; no force feedback (operator relies on vision) |
| VR / hand tracking | Mobile and bimanual EEF control, humanoids, natural motions | IK singularities, tracking jitter, latency; operator fatigue |
| SpaceMouse / gamepad | EEF delta control on single arms, simple pick and place | Slow; operators move one axis at a time, producing unnatural, hesitant data |
| Kinesthetic (hand-guiding the arm) | Arms with good gravity compensation; contact tasks | The human is in the camera view; the robot pose is driven by a hand, so recordings mix human and robot |
| Handheld grippers (UMI-style) | In-the-wild collection without a robot | Needs latency matching and relative actions; embodiment gap at deployment |

Pick the interface that makes demonstrations **fast and smooth**. Hesitation, pauses
and corrections in demos become hesitation in the policy.

## Cameras

- **Wrist camera plus at least one external camera** is the default for manipulation. The
  wrist view resolves fine alignment and grasp state; the external view gives global
  context and object location. ALOHA used two wrist cameras plus front and top views.
- Fix exposure and white balance. Auto-exposure changes the input distribution with
  every arm motion.
- Avoid views where the operator's body or the leader arm is visible; it will be absent
  at deployment.
- Record at the control rate you will deploy at, or above it and downsample consistently.

## Collection session checklist

- [ ] Task spec and success criterion written down; same for every operator
- [ ] Camera positions verified against reference images; exposure locked
- [ ] Initial-condition region marked; per-episode conditions logged
- [ ] Episode boundaries: start recording at first motion, stop at task completion
- [ ] Operator practised until demos are smooth; first warm-up demos discarded
- [ ] Recovery episodes planned (for example 10-20% of the budget)
- [ ] Dataset QA run after every session (`robot-data-quality`), not after the campaign
