# Contributing

Thanks for helping make AI agents better robotics engineers.

This repo is most useful when each skill is practical, opinionated, and grounded in
real failure modes from robot software.

## Good First Contributions

- Add a missing anti-pattern to an existing skill.
- Add a production checklist to a skill.
- Add a small, realistic code example.
- Improve the eval prompts or add another before/after eval.
- Start one of the roadmap skills from the README.

## Skill Structure

Each skill lives in `skills/<skill-name>/SKILL.md` and starts with YAML frontmatter:

```yaml
---
name: robot-navigation
description: >
  Nav2, SLAM, localization, costmaps, planners, behavior trees, recovery behaviors,
  and ROS2 navigation deployment. Use this skill when the user mentions Nav2,
  AMCL, map_server, costmap tuning, BT Navigator, planner plugins, controller
  plugins, recoveries, localization, or mobile robot navigation.
---
```

Then include:

- When to use the skill.
- Core patterns.
- Working code or configuration examples.
- Common pitfalls and anti-patterns.
- Testing and debugging notes.
- Production checklist.

## Review Checklist

Before opening a PR:

- The skill has specific trigger language in `description`.
- Examples use realistic ROS names, QoS settings, parameters, and launch patterns.
- Advice distinguishes simulation, development, and production when it matters.
- Safety-critical behavior fails closed or degrades gracefully.
- Tests, diagnostics, and observability are covered where relevant.
- The README skill table or roadmap is updated if needed.

## Style

- Prefer concrete examples over generic advice.
- Mention the failure mode the pattern prevents.
- Keep code snippets small enough to copy, but complete enough to run.
- Avoid claiming universal best practices when ROS distribution, DDS vendor, hardware,
  or robot class changes the answer.
