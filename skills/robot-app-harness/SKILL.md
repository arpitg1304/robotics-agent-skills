---
name: robot-app-harness
description: >
  Closed-loop workflow for writing robot application software against a rig (a robot plus its
  sensors and actuators) using the `rah` harness in this repository: probe the devices, write
  drivers and task code, run it in simulation, read the evaluator report, and iterate until it
  passes held-out seeds. Use this skill when the user asks to make a robot do a task, to
  integrate a sensor or actuator into a robot application, to write pick-and-place or other
  manipulation code for a rig, or to "vibe code" robotics software and verify it in a simulator.
  Trigger on mentions of rah, rig manifests, robot task specs, the ARMATURE simulator, probing
  robot devices, device descriptors, running or verifying a robot app, held-out seeds, or
  diagnosing why a simulated robot task fails.
---

# Robot App Harness Skill

Write robot software the way you write any other software with a test suite: discover what
is there, write code, run it, read what failed, fix it. The harness supplies the parts
robotics normally lacks: a uniform view of the hardware, a resettable world, and an
evaluator you cannot argue with.

## When to Use This Skill

- The user names a robot and a task and wants working code, not advice.
- A sensor or actuator has to be integrated into an application (frames, units, rates, limits).
- A robot application fails in simulation and the cause has to be found.

It does not replace the domain skills. Load `robotics-software-principles` for structure and
`robotics-testing` when the code needs its own tests.

The harness drives scripted applications against a simulated rig today. Learned policies and
real hardware are separate skills: `lerobot` for the SO-101 and its datasets, `imitation-learning`
and `robot-reinforcement-learning` for training policies, `robot-data-quality` for auditing
demonstrations, and `mujoco` or `isaac-sim` for other simulators.

## Setup

The harness lives in `harness/` of the robotics-agent-skills repository. `rah` below means
`harness/bin/rah` (Python 3.10+, no packages to install).

```bash
export ARMATURE_HOME=/path/to/armature   # checkout with `npm ci` already run, Node.js 22+
rah doctor                               # confirms the simulator backend loads
```

## The Loop

### 1. Probe before writing anything

```bash
rah probe --rig harness/rigs/armature-panda-bench.json
```

The output is the ground truth about the machine: each device's kind, units, limits, rate,
frame and mounting transform, and the services it offers. Read all of it. Do not assume a
sensor reports in the world frame, at the control rate, or without noise; the descriptor says.

Read the task file too: `instruction` is the request, and at run time `rig.task["layout"]`
gives the goal relations, target positions and part dimensions.

### 2. Write the application

```bash
rah init my_app --rig <rig.json> --task <task.json>   # creates my_app/app.py and LESSONS.md
```

An app is a Python file defining `run(rig)`. Sibling modules are importable. The whole API:

| Call | Purpose |
| --- | --- |
| `rig.devices` | Device descriptors, same content as `rah probe` |
| `rig.task` | Instruction, public layout, step budget |
| `rig.safety` | Workspace envelope the backend enforces |
| `rig.read(device)` | Latest sample; check `stamp`, slow sensors return held samples |
| `rig.write(device, **command)` | Actuator target; holds until replaced |
| `rig.call(device, method, **params)` | Device service, such as the arm's `ik` |
| `rig.step(n=1)` | Advance n control ticks; raises `EpisodeOver` once the episode ends |
| `rig.done` | True once the episode has ended |
| `rig.mark(label)` | Name the phase being entered |
| `rig.claim(kind, ...)` | State a belief derived from sensors, to be checked |

Structure the code as drivers (one class per device, everything robot specific read from
the descriptor) and an application on top. Hard-coding a joint count, a gripper width or a
frame offset makes the app fail on the next rig.

Two calls cost nothing and make every later failure cheaper to diagnose:

- `rig.mark("descend object_0")` before each phase. Reports say which phase a relation broke in.
- `rig.claim("object_position", id=0, position=[x, y, z])` (world frame) whenever you compute
  an object pose from a sensor, and `rig.claim("holding", id=0)` after a grasp. The report
  scores claims against simulator ground truth. A large error with a failing task means your
  sensor integration is wrong (frame, stale sample, units), not your motion.

### 3. Run on dev seeds and read the report

```bash
rah run --rig <rig.json> --task <task.json> --app my_app/app.py --note "what this attempt changes"
```

Read the per-episode lines top to bottom:

| Signal | Meaning |
| --- | --- |
| `outcome` | `success`, `never_grasped`, `slipped`, `disturbed`, `not_released`, `off_target`, `fell`, `safety_violation`, `app_error` |
| `progress` | Share of stages reached (grasped, lifted, placed per part). Rising progress with no new successes is still progress |
| `goal not met` | Which goal relations fail at the end |
| relation timeline | When each relation became true or false, and during which mark |
| `belief check` | Your claims disagreed with ground truth |

`runs/<run>/report.json` has everything, and `episode-<seed>.trace.jsonl` has joint, tool
and object positions every third tick. Compute from the trace instead of guessing: for a
placement error, print the tool-to-object offset at each mark and see in which phase it grows.

To let the user watch an episode, `rah replay runs/<run>` (optionally `--seed N`) serves a
viewer that re-simulates it in the ARMATURE studio next to the event timeline. It blocks
until interrupted, so start it in the background and give the user the printed URL.

### 4. Change one thing, rerun, compare

Every run appends to `runs/ledger.jsonl` with a digest of the app source, your note, and
per-seed outcomes. Use it to tell whether a change helped. Runs are deterministic per seed,
so a different result means the code changed it.

- Fix the earliest failing stage first.
- Five seeds are few. Before keeping or discarding a change on a one-seed difference, rerun
  with more: `--seeds 1 2 3 4 5 6 7 8 9 10`.
- Revert changes that do not help. An unexplained fix is a future regression.

### 5. Verify on held-out seeds

```bash
rah verify --rig <rig.json> --task <task.json> --app my_app/app.py
```

This is the acceptance gate. Do not tune against it: if it fails, form a hypothesis, test it
on dev seeds, and verify again. Report the held-out number to the user as measured.

### 6. Record what the rig taught you

Add durable, rig-specific facts to `LESSONS.md` in the workspace (for example "parts creep in
the SO-101 jaw while held; keep carries short"). Read it before the next task on that rig.

## Rules

- **The app sees devices, the developer sees the evaluator.** Never copy ground truth from a
  report or trace into the app. An app that passes by reading the answer is worthless.
- **Do not edit the harness, task, rig or seeds to make a run pass.** If one looks wrong,
  say so and stop.
- **A safety violation is a bug, not noise.** The tool left the workspace; find out why
  before retrying.
- **Report fidelity honestly.** The ARMATURE backend uses velocity-limited kinematic servos
  with approximate collision. Passing here shows the logic and the integration are sound. It
  is not evidence the code will work on hardware, and the handover must say so.

## Common Failures

| Symptom | Likely cause |
| --- | --- |
| `never_grasped`, large belief error | Tracker reading used without applying its mount transform |
| `never_grasped`, small belief error | Grasp height, or the tool had not converged before closing |
| Placement off by a constant amount | Part sits off-centre in the fingers; measure it in hand and compensate |
| `disturbed` | The open gripper or a carried part hit a part that was already placed; reorder or approach higher |
| `slipped` | Lifted before the grip settled, or carried too long or too fast |
| Out of step budget | Fixed waits; replace them with convergence checks |
| `EpisodeOver` raised mid-phase | Budget, safety, or the task already succeeded; let it propagate |

## Checklist Before Handing Over

- [ ] `rah verify` result stated with the number of held-out seeds
- [ ] No ground-truth values in the app; belief errors within sensor noise
- [ ] Nothing robot specific hard-coded that a descriptor provides
- [ ] `LESSONS.md` updated
- [ ] Simulator fidelity limits stated
