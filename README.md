# 🤖 Robotics Agent Skills

Production-grade robotics knowledge for AI coding agents.

Drop these `SKILL.md` files into Claude Code, Cursor, Copilot-style agents, or custom
agent frameworks to make them generate better ROS1/ROS2 software: safer nodes, correct
QoS, lifecycle patterns, tests, launch files, Docker, bringup, perception, and security.

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
![Skills](https://img.shields.io/badge/skills-10-green)
![ROS](https://img.shields.io/badge/ROS1%20%2B%20ROS2-supported-orange)
![Agent Format](https://img.shields.io/badge/format-SKILL.md-purple)

## Why This Exists

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

Full report: [evals/EVAL_REPORT.md](evals/EVAL_REPORT.md)

## Quick Start

Install selected skills into a project:

```bash
git clone https://github.com/arpitg1304/robotics-agent-skills.git
cd robotics-agent-skills
./install.sh --target /path/to/your/robot/.claude/skills --skills ros2 robotics-testing robot-bringup
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
| [robotics-security](skills/robotics-security/SKILL.md) | Security and hardening | SROS2, DDS encryption, network segmentation, secrets management, e-stop isolation, secure boot |

## How Agents Use These Skills

### Claude Code

Copy or symlink the skills you need into your project's `.claude/skills/` directory.
Claude Code auto-discovers `SKILL.md` files and triggers them based on the YAML
`description` field.

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
└── Deployment ---------- ros2/ (production checklist, CI/CD)
```

## Roadmap

High-impact skills to add next:

- `robot-navigation/` - Nav2, SLAM, localization, costmaps, planners, recovery behaviors.
- `robot-manipulation/` - MoveIt2, grasp planning, planning scenes, TF, pick-and-place.
- `ros2-control/` - hardware interfaces, controllers, update loops, real-time pitfalls.
- `robot-simulation/` - Gazebo, Isaac Sim, MuJoCo, sim clock, resets, domain randomization.
- `robot-description/` - URDF, Xacro, inertials, collision meshes, robot_state_publisher.
- `robot-debugging/` - ros2 doctor, rqt, tracing, rosbag replay, diagnostics.
- `robotics-data-pipelines/` - RLDS, LeRobot, Zarr, dataset curation, format conversion.
- `multi-robot-systems/` - namespaces, fleets, DDS partitions, task allocation.

Contributions are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for the skill format
and review checklist.
