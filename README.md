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

## How Agents Use These Skills

### In Claude Projects / Claude Code
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
└── Deployment ─────────── ros2/ (production checklist, CI/CD)
```

## Roadmap

Future skills to consider:
- `robotics-data-pipelines/` — RLDS, LeRobot, Zarr, format conversion, asymmetric I/O, curation
- `robot-simulation/` — MuJoCo, Isaac Sim, Gazebo setup and best practices
- `robot-manipulation/` — Grasping, motion planning, force control
- `robot-navigation/` — Nav2, SLAM, path planning, localization
- `robot-learning/` — Imitation learning, RL, VLA model fine-tuning
- `robot-deployment/` — Docker, Kubernetes, fleet management, OTA updates
