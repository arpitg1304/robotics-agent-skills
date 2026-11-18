# Robotics Agent Harness (`rah`)

An experimental harness that lets a coding agent write robot application software the way
it writes any other software: discover the hardware, write code, run it, read what failed,
fix it.

You pick a **rig** (a robot plus its peripherals) and describe a **task**. The agent probes
the rig, writes drivers and task code against a small device bus, runs the code in a
simulator, and iterates on an evaluator report until the task passes on seeds it has not
tuned against.

The first backend is [ARMATURE](https://github.com/arpitg1304/armature), run headlessly.

## Quick start

Requires Python 3.10+ and Node.js 22+. Nothing to install on the Python side.

```bash
git clone https://github.com/arpitg1304/armature.git && (cd armature && npm ci)
export ARMATURE_HOME=$PWD/armature

cd robotics-agent-skills/harness
bin/rah doctor
bin/rah probe  --rig rigs/armature-panda-bench.json
bin/rah run    --rig rigs/armature-panda-bench.json --task tasks/transfer-blocks.json --app examples/pick_place/app.py
bin/rah verify --rig rigs/armature-panda-bench.json --task tasks/transfer-blocks.json --app examples/pick_place/app.py
```

Watch any episode from a run in the ARMATURE studio (needs `npm run build` in the ARMATURE
checkout):

```bash
bin/rah replay runs/<run-directory>            # first failed episode, or --seed N
```

To have an agent write the app instead, install the
[`robot-app-harness`](../skills/robot-app-harness/SKILL.md) skill and ask for the task:

```bash
bin/rah init my_app --rig rigs/armature-panda-bench.json --task tasks/stack-blocks.json
# then, in your agent: "Make my_app/app.py solve the task. Use the robot-app-harness skill."
```

## How it fits together

```text
 agent ── writes ──> app.py + drivers.py
   ^                     │ run(rig)
   │                     v
   │             ┌──────────────┐   JSON lines   ┌──────────────────────────┐
 report,         │ rah runner   │ <────────────> │ backend (ARMATURE bridge)│
 trace,          │  Rig bus     │  read / write  │  devices   ── app side   │
 ledger          │  ledger      │  call / step   │  evaluator ── privileged │
   │             └──────────────┘                └──────────────────────────┘
   └──────────────────────┘
```

| Piece | What it is |
| --- | --- |
| Rig manifest (`rigs/*.json`) | Robot, peripherals with mounting, rate and noise, and a safety envelope |
| Task (`tasks/*.json`) | Instruction, scene, optional goal relations, dev and held-out seeds |
| App | Python file defining `run(rig)`; all it can touch is the device bus |
| Backend | A process that owns the world, the devices and the evaluator |
| Report | Per-episode outcome, stage progress, goal relations, relation timeline, belief checks |
| Replay (`rah replay`) | The recorded episode re-simulated in the ARMATURE studio with the event timeline |
| Ledger (`runs/ledger.jsonl`) | Append-only record tying each result to a digest of the app source |

### What the app sees

`rig.devices` describes every device: units, limits, rates, frames, mounting transforms and
services. On the ARMATURE rigs those are an arm (joint-position interface plus controller
IK), a gripper with contact detection, and an object tracker. The tracker reports in its own
mounted frame, below the control rate, with noise, so using it is a real integration step.

`rig.task["layout"]` is the public description of the job: goal relations, target positions
and part dimensions, the kind of thing a cell drawing provides.

### What the app does not see

The evaluator runs in the backend on simulator ground truth and reports to the developer:

- **Goal relations.** Success is a set of relations such as `at(object_0,target_2)` or
  `on(object_2,object_1)` holding with the gripper open and the parts at rest. A task can
  define its own; otherwise ARMATURE's built-in criterion applies.
- **Stage progress.** Each part is grasped, lifted and placed. A failed run still tells you
  how far it got.
- **Relation timeline.** When each relation became true or false, and during which phase
  (`rig.mark`).
- **Belief checks.** The app states what it believes from its sensors (`rig.claim`); the
  report scores that against ground truth, which separates perception bugs from motion bugs.
- **Safety.** The tool leaving the rig's workspace ends the episode.

The app and the evaluator share a process boundary but not a sandbox: the separation is
enforced by the API, and by the agent's instructions, not by isolation.

### Replay

Every episode records the action sent on each control tick. `rah replay` serves a viewer
that loads the ARMATURE studio and steps it with those actions, alongside the phase marks
and the relation timeline; clicking an event jumps to one second before it. The simulator is
deterministic per seed, so this is a re-simulation, and the viewer checks it against the
recorded object positions and says so if they diverge. For tasks with their own goal the
studio may end the episode by its built-in rule before the recording does; the viewer
reports that too.

### Dev and held-out seeds

`rah run` uses a task's dev seeds and is what the agent iterates against. `rah verify` uses
the held-out seeds and is the acceptance gate. Nothing stops an agent passing held-out seeds
to `rah run`; the skill tells it not to, and the ledger records the split of every run.

## Reference results

`examples/pick_place` was written through the loop above. Measured on this checkout, ten
held-out seeds each:

| Rig | Task | Dev seeds | Held-out seeds |
| --- | --- | --- | --- |
| Panda | `transfer-blocks` | 5/5 | 10/10 |
| Panda | `transfer-crossed` | 5/5 | 9/10 |
| Panda | `stack-blocks` | 4/5 | 10/10 |
| SO-101 | `transfer-blocks` | 5/5 | 10/10 |
| SO-101 | `transfer-crossed` | 3/5 | 4/10 |
| SO-101 | `stack-blocks` | 0/5 | 0/10 |

The SO-101 failures are real and unsolved: on its compact bench the parts sit 40 mm apart,
so the open jaw knocks parts that are already placed, and a held part creeps in the jaw
while it is carried. They are left in as open problems for an agent to work on; see
[examples/pick_place/LESSONS.md](examples/pick_place/LESSONS.md).

These numbers describe this simulator. ARMATURE moves the arm with velocity-limited
kinematic servos and approximate collision proxies; it is not a validated dynamics model,
and nothing here is evidence that the code transfers to hardware.

## Backend protocol

A backend is any executable that reads one JSON request per line on stdin and writes one
JSON response per line on stdout: `{"ok": true, "result": {...}}` or
`{"ok": false, "error": "..."}`.

| `op` | Fields | Result |
| --- | --- | --- |
| `hello` | | `protocol`, backend name and version, fidelity statement |
| `configure` | `rig` | `devices`, `safety` |
| `reset` | `scene`, `goal`, `seed` | public episode context (`layout`, `step_budget`) |
| `read` | `device` | latest sample with `stamp` |
| `write` | `device`, `command` | |
| `call` | `device`, `method`, `params` | service result |
| `step` | `n` | `step`, `stamp`, `done` |
| `mark` | `label` | |
| `claim` | `kind`, ... | (scored, nothing returned) |
| `finish` | | `report`, `trace`, optional `replay` |

Point a rig at another implementation with `"backend": {"type": "command", "argv": [...]}`.
That is the route to other simulators, a ROS 2 graph, or hardware.

## Tests

```bash
python3 -m unittest discover -s tests     # backend tests skip if ARMATURE is not found
```

## Licensing

The harness is Apache-2.0 like the rest of this repository. ARMATURE is GPL-3.0-only. The
bridge loads it at run time from your own checkout; no ARMATURE code is included here.

## Not built yet

- **Cameras.** ARMATURE renders RGB and depth only in a browser. A Playwright-driven backend
  would add them; until then the tracker stands in for perception.
- **Robustness sweeps.** Friction, mass and sensor-noise variation as a second gate.
- **More goal relations and tasks.** Insertion, sorting, containment.
- **Parallel agents.** One simulator process per worktree, results merged through git.
- **Hardware backends.** A real rig additionally needs automatic reset and learned or
  scripted verification, which the simulator gives away for free.
