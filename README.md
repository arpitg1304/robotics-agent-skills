# 🤖 Robotics Agent Skills

Production-grade robotics knowledge for AI coding agents, and a harness that lets an
agent build robot software and prove it works.

Drop the `SKILL.md` files into Claude Code, Autohand Code, Cursor, Copilot-style
agents, or custom agent frameworks to make them generate better ROS1/ROS2 software:
safer nodes, correct QoS, lifecycle patterns, tests, launch files, Docker, bringup,
perception, and security. Then use the harness to have the agent write robot
application code, run it in simulation, and iterate until it passes held-out trials.

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
![Skills](https://img.shields.io/badge/skills-17-green)
![ROS](https://img.shields.io/badge/ROS1%20%2B%20ROS2-supported-orange)
![Agent Format](https://img.shields.io/badge/format-SKILL.md-purple)
[![Install with AIPM](https://www.aipm-registry.com/install-with-aipm.svg)](https://www.aipm-registry.com/skills/arpitg1304/ros2/1.0.0?utm_source=github&utm_medium=readme&utm_campaign=package_badge)

## Two Parts

- **Skills** (`skills/`): robotics engineering knowledge an agent loads as context. Stable.
- **Harness** (`harness/`): a closed loop in which an agent writes robot application code
  against a rig, runs it in a simulator and iterates on an evaluator report. Experimental.

## Agent Harness (Experimental)

Knowledge makes an agent write better robot code. It does not tell the agent whether the
code works. The harness closes that loop: pick a **rig** (a robot plus its sensors and
actuators) and a **task**, and the agent probes the devices, writes drivers and task code,
runs them in simulation, reads what failed, and iterates until the code passes seeds it has
not tuned against.

```text
probe the rig ──> write app.py ──> rah run (dev seeds) ──> read report ──┐
                        ^                                                 │
                        └───────────────── fix one thing ─────────────────┘
                                                   └──> rah verify (held-out seeds)
```

What the first version gives you:

- **A device bus, not a simulator API.** The app sees an arm with controller IK, a gripper
  with contact sensing, and an object tracker that reports in its own mounted frame, below
  the control rate, with noise. Integrating the sensors is part of the job.
- **An evaluator the app cannot see.** Success is a set of goal relations such as
  `at(object_0,target_2)` or `on(object_2,object_1)`. Reports give per-part stage progress,
  a timeline of which relation broke during which phase, a check of the app's beliefs
  against ground truth, and safety-envelope violations.
- **Honest iteration.** Dev and held-out seeds, plus an experiment ledger that ties every
  result to a hash of the code that produced it.
- **Replay.** `rah replay` re-simulates any episode in the ARMATURE studio next to its event
  timeline.
- **An agent procedure.** The [robot-app-harness](skills/robot-app-harness/SKILL.md) skill
  teaches an agent the loop.

### Try it

Requires Python 3.10+ and Node.js 22+. The first backend is the
[ARMATURE](https://github.com/arpitg1304/armature) simulator, loaded at run time from your
own checkout.

```bash
git clone https://github.com/arpitg1304/armature.git
(cd armature && npm ci && npm run build)     # the build is only needed for replay
export ARMATURE_HOME=$PWD/armature

cd robotics-agent-skills/harness
bin/rah doctor
bin/rah probe  --rig rigs/armature-panda-bench.json
bin/rah run    --rig rigs/armature-panda-bench.json --task tasks/stack-blocks.json --app examples/pick_place/app.py
bin/rah verify --rig rigs/armature-panda-bench.json --task tasks/stack-blocks.json --app examples/pick_place/app.py
bin/rah replay runs/<run-directory>
```

To have an agent write the app, start a workspace with `bin/rah init my_app --rig ... --task ...`
and ask your agent to solve it using the `robot-app-harness` skill.

### First results

A reference pick-and-place app, written through the loop, on held-out seeds:

| Task | Franka Panda | SO-101 |
| --- | --- | --- |
| Move three blocks to their targets | 10/10 | 10/10 |
| Same, in reverse order | 9/10 | 4/10 |
| Stack three blocks | 10/10 | 0/10 |

The SO-101 failures are open problems, described in the app's
[LESSONS.md](harness/examples/pick_place/LESSONS.md). ARMATURE moves the arm with kinematic
servos and approximate collision, so these numbers show that the logic and sensor
integration are sound, not that the code transfers to hardware.

Design, backend protocol and what is not built yet: [harness/README.md](harness/README.md).

## Why the Skills Exist

General-purpose coding agents know ROS syntax, but they often miss the robotics details
that matter in production: QoS compatibility, deterministic startup, bounded callbacks,
sensor health checks, safe shutdown, launch ergonomics, and testability.

This repo is the missing robotics engineering layer for AI coding agents.

## Evidence: Same Prompt, With and Without Skills

The `evals/` directory compares the same ROS2 package prompt run once with no skills and
once with the robotics skills loaded as context.

| Result | Without Skills | With Skills |
|--------|----------------|-------------|
| Node architecture | Plain `rclpy.Node`; setup in constructor | Lifecycle node with configure/activate/deactivate/cleanup/shutdown |
| Sensor QoS | Default depth 10 for everything | Sensor QoS for camera/joints, reliable QoS for status |
| Sensor handling | Latest-message assignment | Thread-safe bounded buffers, drop counts, watchdogs |
| Timestamp sync | None | Sync offset checks and dropped-frame accounting |
| Tests | 0 lines | 601 lines |
| Structure | Single node module | Separate writer module and cleaner responsibilities |
| Config surface | 4 parameters | 13 parameters |

This is a single-task, single-run comparison, self-assessed, and neither package was
built against a real ROS2 distribution — treat it as an illustration of the design
decisions the skills push toward, not as a benchmark. Caveats and the known build
defects in both artifacts are documented in the report.

Full report: [evals/EVAL_REPORT.md](evals/EVAL_REPORT.md)

## Skills Quick Start

Install selected skills into a project:

```bash
git clone https://github.com/arpitg1304/robotics-agent-skills.git
cd robotics-agent-skills
./install.sh --target /path/to/your/robot/.claude/skills --skills ros2 robotics-testing robot-bringup
```

For Autohand Code project skills:

```bash
./install.sh --target /path/to/your/robot/.autohand/skills --skills ros2 robotics-testing robot-bringup
```

Or copy manually:

```bash
cp -R skills/ros2 /path/to/your/robot/.claude/skills/
cp -R skills/robotics-testing /path/to/your/robot/.claude/skills/
```

## Recommended Bundles

| Bundle | Skills |
|--------|--------|
| ROS2 application | `ros2`, `robotics-software-principles`, `robotics-testing`, `robot-bringup` |
| Robot perception | `ros2`, `robot-perception`, `robotics-testing`, `docker-ros2-development` |
| Production robot | `ros2`, `robot-bringup`, `robotics-security`, `robotics-testing` |
| Web dashboard | `ros2`, `ros2-web-integration`, `robotics-security`, `robot-bringup` |
| ROS1 maintenance | `ros1`, `robotics-software-principles`, `robotics-testing` |
| Robot learning on LeRobot hardware | `lerobot`, `imitation-learning`, `robot-data-quality`, `robot-app-harness` |
| RL in simulation | `robot-reinforcement-learning`, `mujoco` or `isaac-sim`, `robotics-testing` |

## Skills

| Skill | Description | Key Topics |
|-------|-------------|------------|
| [robotics-software-principles](skills/robotics-software-principles/SKILL.md) | Design principles | SOLID for robotics, fail-safe defaults, rate separation, composability, graceful degradation |
| [ros1](skills/ros1/SKILL.md) | ROS1 development | catkin, rospy, roscpp, nodelets, tf, actionlib, launch XML, migration |
| [ros2](skills/ros2/SKILL.md) | ROS2 development | rclpy, rclcpp, DDS, QoS, lifecycle nodes, components, Python launch |
| [robotics-design-patterns](skills/robotics-design-patterns/SKILL.md) | Architecture patterns | Behavior trees, FSMs, HAL, safety systems, sensor fusion, sim-to-real |
| [robot-perception](skills/robot-perception/SKILL.md) | Perception systems | Cameras, LiDAR, depth, calibration, point clouds, detection, tracking, sensor fusion |
| [robotics-testing](skills/robotics-testing/SKILL.md) | Testing strategies | pytest + ROS, launch_testing, mock hardware, golden files, CI/CD |
| [docker-ros2-development](skills/docker-ros2-development/SKILL.md) | Docker + ROS2 | Multi-stage Dockerfiles, compose, DDS across containers, GPU passthrough, devcontainers |
| [ros2-web-integration](skills/ros2-web-integration/SKILL.md) | Web integration | rosbridge, FastAPI/Flask bridges, WebSocket streaming, REST APIs, MJPEG/WebRTC, security |
| [robot-bringup](skills/robot-bringup/SKILL.md) | System bringup | systemd services, launch composition, udev rules, watchdogs, log rotation, graceful shutdown |
| [lerobot](skills/lerobot/SKILL.md) | Hugging Face LeRobot | SO-101 bring-up, calibration, teleop, recording, LeRobotDataset, training and deploying policies |
| [robot-data-quality](skills/robot-data-quality/SKILL.md) | Dataset quality | Episode QA, sync and frame drops, idle frames, coverage, format conversion and inspection with forge |
| [imitation-learning](skills/imitation-learning/SKILL.md) | Imitation learning | Demo collection, action chunking, ACT, Diffusion Policy, VLAs, honest evaluation, deployment |
| [robot-reinforcement-learning](skills/robot-reinforcement-learning/SKILL.md) | Reinforcement learning | MDP and reward design, PPO/SAC, parallel sim training, sim-to-real, real-world RL, debugging |
| [mujoco](skills/mujoco/SKILL.md) | MuJoCo simulation | MJCF modelling, contacts and stability, rendering, MJX and Playground, Menagerie |
| [isaac-sim](skills/isaac-sim/SKILL.md) | NVIDIA Isaac Sim and Isaac Lab | USD assets, URDF import, Isaac Lab environments, parallel training, tiled cameras, Mimic |
| [robot-app-harness](skills/robot-app-harness/SKILL.md) | Closed-loop app development | Probing rigs, device drivers, running in simulation, reading evaluator reports, held-out verification |
| [robotics-security](skills/robotics-security/SKILL.md) | Security and hardening | SROS2, DDS encryption, network segmentation, secrets management, e-stop isolation, secure boot |

## How Agents Use These Skills

### Claude Code

Copy or symlink the skills you need into your project's `.claude/skills/` directory.
Claude Code auto-discovers `SKILL.md` files and triggers them based on the YAML
`description` field.

### Autohand Code

Copy or symlink the skills you need into your project's `.autohand/skills/`
directory. User-level skills live under `~/.autohand/skills/`; project-level
skills live under `<project>/.autohand/skills/`.

### Claude Projects

Place the skill directories in your project's `/mnt/skills/user/` directory.

### Custom Agent Frameworks

Load the relevant `SKILL.md` as system-prompt context when a task matches the skill:

```python
from pathlib import Path

SKILLS = {
    "ros2": "skills/ros2/SKILL.md",
    "testing": "skills/robotics-testing/SKILL.md",
    "perception": "skills/robot-perception/SKILL.md",
    "bringup": "skills/robot-bringup/SKILL.md",
    "security": "skills/robotics-security/SKILL.md",
}

def load_skill(task_description: str) -> str:
    text = task_description.lower()
    for trigger, path in SKILLS.items():
        if trigger in text:
            return Path(path).read_text()
    return ""
```

## What Good Skills Contain

- Specific trigger descriptions so agents know when to load them.
- Working examples, not just prose.
- Robotics failure modes: QoS mismatches, stale transforms, unsafe shutdown, blocking callbacks.
- Production defaults: bounded queues, lifecycle management, observability, tests.
- Anti-patterns that show what to avoid.

## Coverage Map

```text
Robot System Architecture
├── Design Principles ---- robotics-software-principles/ (SOLID, safety, composability)
├── Middleware ----------- ros1/, ros2/
├── Behaviors ----------- robotics-design-patterns/ (BT, FSM)
├── Perception ---------- robot-perception/ (cameras, LiDAR, depth, calibration, fusion)
├── Planning ------------ robotics-design-patterns/ (motion planning)
├── Control ------------- robotics-design-patterns/ (control loops)
├── Safety -------------- robotics-design-patterns/ (watchdogs, limits)
├── Testing ------------- robotics-testing/ (unit, integration, sim)
├── Containerization ---- docker-ros2-development/ (Dockerfiles, compose, DDS, GPU)
├── Web Interfaces ------ ros2-web-integration/ (REST, WebSocket, streaming, dashboards)
├── System Bringup ------ robot-bringup/ (systemd, udev, watchdogs, boot sequence)
├── Security ------------ robotics-security/ (SROS2, hardening, e-stop isolation)
├── Deployment ---------- ros2/ (production checklist, CI/CD)
├── Robot Learning ------ imitation-learning/, robot-reinforcement-learning/, lerobot/
├── Data ---------------- robot-data-quality/ (episode QA, formats, forge)
├── Simulation ---------- mujoco/, isaac-sim/
└── Closed-Loop Dev ----- robot-app-harness/ + harness/ (probe, write, simulate, verify)
```

## Roadmap

For the harness roadmap, see [harness/README.md](harness/README.md#not-built-yet).

High-impact skills to add next:

- `robot-navigation/` - Nav2, SLAM, localization, costmaps, planners, recovery behaviors.
- `robot-manipulation/` - MoveIt2, grasp planning, planning scenes, TF, pick-and-place.
- `ros2-control/` - hardware interfaces, controllers, update loops, real-time pitfalls.
- `gazebo/` - Gazebo with ROS 2, sim clock, resets, sensors (MuJoCo and Isaac Sim now have skills).
- `robot-description/` - URDF, Xacro, inertials, collision meshes, robot_state_publisher.
- `robot-debugging/` - ros2 doctor, rqt, tracing, rosbag replay, diagnostics.
- `robot-learning-deployment/` - on-robot inference, latency budgets, async execution, model serving.
- `multi-robot-systems/` - namespaces, fleets, DDS partitions, task allocation.

Contributions are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for the skill format
and review checklist.
