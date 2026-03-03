# 🤖 Robotics Skill Store

A curated collection of SKILL.md files for AI agents working on robotics software development. Each skill follows the Anthropic skill format with YAML frontmatter, actionable patterns, real code examples, and documented anti-patterns.

## Skills

| Skill | Description | Key Topics |
|-------|-------------|------------|
| **[robotics-software-principles/](skills/robotics-software-principles/SKILL.md)** | Design Principles | SOLID for robotics, fail-safe defaults, rate separation, composability, graceful degradation |
| **[ros1/](skills/ros1/SKILL.md)** | ROS1 Development | catkin, rospy, roscpp, nodelets, tf, actionlib, launch XML, migration |
| **[ros2/](skills/ros2/SKILL.md)** | ROS2 Development | rclpy, rclcpp, DDS, QoS, lifecycle nodes, components, Python launch |
| **[robotics-design-patterns/](skills/robotics-design-patterns/SKILL.md)** | Architecture Patterns | Behavior trees, FSMs, HAL, safety systems, sensor fusion, sim-to-real |
| **[robot-perception/](skills/robot-perception/SKILL.md)** | Perception Systems | Cameras, LiDAR, depth, calibration, point clouds, detection, tracking, sensor fusion |
| **[robotics-testing/](skills/robotics-testing/SKILL.md)** | Testing Strategies | pytest + ROS, launch_testing, mock hardware, golden files, CI/CD |
| **[docker-ros2-development/](skills/docker-ros2-development/SKILL.md)** | Docker + ROS2 | Multi-stage Dockerfiles, docker-compose, DDS across containers, GPU passthrough, devcontainers |
| **[ros2-web-integration/](skills/ros2-web-integration/SKILL.md)** | Web Integration | rosbridge, FastAPI/Flask bridges, WebSocket streaming, REST APIs, MJPEG/WebRTC, security |
| **[robot-bringup/](skills/robot-bringup/SKILL.md)** | System Bringup | systemd services, launch composition, udev rules, watchdogs, log rotation, graceful shutdown |
| **[robotics-security/](skills/robotics-security/SKILL.md)** | Security & Hardening | SROS2, DDS encryption, network segmentation, secrets management, e-stop isolation, secure boot |

## How Agents Use These Skills

### In Claude Code
Copy or symlink the skills you need into your project's `.claude/skills/` directory:

```bash
# Copy specific skills into your robotics project
cp -r /path/to/robotics-agent-skills/skills/ros2 .claude/skills/
cp -r /path/to/robotics-agent-skills/skills/robot-bringup .claude/skills/

# Or symlink to avoid duplication
ln -s /path/to/robotics-agent-skills/skills/ros2 .claude/skills/ros2
```

Claude Code auto-discovers SKILL.md files in `.claude/skills/` and triggers them based on the YAML `description` field.

### In Claude Projects
Place the skill directories in your project's `/mnt/skills/user/` directory. The agent will auto-detect and reference them based on the YAML `description` field.

### In Custom Agent Frameworks
Load the relevant SKILL.md as system prompt context when the agent encounters a matching task:

```python
# Example: Agent skill loader
def load_skill(task_description: str) -> str:
    skills = {
        'ros1': 'skills/ros1/SKILL.md',
        'ros2': 'skills/ros2/SKILL.md',
        'design': 'skills/robotics-design-patterns/SKILL.md',
        'perception': 'skills/robot-perception/SKILL.md',
        'testing': 'skills/robotics-testing/SKILL.md',
        'docker': 'skills/docker-ros2-development/SKILL.md',
        'web': 'skills/ros2-web-integration/SKILL.md',
        'bringup': 'skills/robot-bringup/SKILL.md',
        'security': 'skills/robotics-security/SKILL.md',
    }
    # Match task to skill and inject into context
    for key, path in skills.items():
        if key in task_description.lower():
            return open(path).read()
```

### With LangChain / LlamaIndex
```python
from langchain.tools import Tool

ros2_skill = Tool(
    name="ROS2 Development Guide",
    description="Best practices for ROS2 development including QoS, lifecycle nodes, DDS configuration",
    func=lambda q: open("skills/ros2/SKILL.md").read()
)
```

## Design Principles

These skills follow several core principles:

1. **Actionable over theoretical** — Every pattern includes working code
2. **Anti-patterns documented** — Learn from common mistakes, not just successes
3. **Progressive complexity** — Start with basics, layer on advanced patterns
4. **Format-agnostic** — Skills work with any agent framework
5. **Real failure modes** — Document what actually breaks in production

## Adding New Skills

Follow the standard skill format:

```
my-new-skill/
├── SKILL.md           # Required: Main skill file with YAML frontmatter
├── references/        # Optional: Detailed reference docs
│   └── advanced.md
└── scripts/           # Optional: Executable helper scripts
    └── validate.py
```

YAML frontmatter must include:
- `name`: Skill identifier (kebab-case)
- `description`: When to trigger (be specific and "pushy" — list explicit trigger phrases)

## Coverage Map

```
Robot System Architecture
├── Design Principles ──── robotics-software-principles/ (SOLID, safety, composability)
├── Middleware ──────────── ros1/, ros2/
├── Behaviors ──────────── robotics-design-patterns/ (BT, FSM)
├── Perception ─────────── robot-perception/ (cameras, LiDAR, depth, calibration, fusion)
├── Planning ───────────── robotics-design-patterns/ (motion planning)
├── Control ────────────── robotics-design-patterns/ (control loops)
├── Safety ─────────────── robotics-design-patterns/ (watchdogs, limits)
├── Testing ────────────── robotics-testing/ (unit, integration, sim)
├── Containerization ───── docker-ros2-development/ (Dockerfiles, compose, DDS, GPU)
├── Web Interfaces ─────── ros2-web-integration/ (REST, WebSocket, streaming, dashboards)
├── System Bringup ─────── robot-bringup/ (systemd, udev, watchdogs, boot sequence)
├── Security ──────────── robotics-security/ (SROS2, hardening, e-stop isolation)
└── Deployment ─────────── ros2/ (production checklist, CI/CD)
```

## Roadmap

Future skills to consider:
- `robotics-data-pipelines/` — RLDS, LeRobot, Zarr, format conversion, asymmetric I/O, curation
- `robot-simulation/` — MuJoCo, Isaac Sim, Gazebo setup and best practices
- `robot-manipulation/` — MoveIt2, grasp planning, force/torque control, pick-and-place pipelines
- `robot-navigation/` — Nav2, SLAM, path planning, costmaps, localization
- `robot-learning/` — Imitation learning, RL, VLA model fine-tuning, policy deployment
- `ros2-control/` — ros2_control framework, hardware interfaces, controllers, PID tuning, real-time
- `robot-description/` — URDF, Xacro, joint types, inertials, collision meshes, SDF, robot_state_publisher
- `robot-hardware-interfaces/` — Serial (UART, RS485), CAN bus, EtherCAT, GPIO, I2C/SPI, custom drivers
- `tf2-transforms/` — Coordinate frames, static vs dynamic transforms, frame conventions, time travel, debugging
- `multi-robot-systems/` — Fleet management, namespacing, multi-robot coordination, task allocation, DDS partitions
- `robot-debugging/` — ros2 doctor, rqt, tracing, CPU/memory profiling, bag recording and replay, diagnostics
