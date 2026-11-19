# LeRobot Overview: Scope, Pipeline and Config System

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Load this file for the full scope
list, the end-to-end pipeline diagram, or how draccus turns config dataclasses into CLI flags.
The stage contracts table stays in SKILL.md.

## When to Use This Skill
- Installing LeRobot, or picking extras for a robot, policy or simulator
- Bringing up an SO-100/SO-101 (or Koch, OMX, LeKiwi, OpenArm, ...) arm: ports, motor IDs,
  calibration, teleoperation, cameras
- Recording teleoperated demonstrations with `lerobot-record`, resuming a recording, pushing it to the Hub
- Inspecting, loading, visualising, editing, merging or converting `LeRobotDataset` data
- Training ACT / Diffusion / SmolVLA / pi0.5 / other policies with `lerobot-train`, and resuming runs
- Deploying a policy on the real robot with `lerobot-rollout` (sync, RTC, remote inference)
- Evaluating in simulation with `lerobot-eval`
- Debugging calibration, camera, timestamp, normalisation and feature-name failures

## Mental Model

```
 HARDWARE                      DATA                           LEARNING                 DEPLOY
 so101_leader ─┐
               ├─ lerobot-record ─► LeRobotDataset v3 ─► lerobot-train ─► checkpoint ─► lerobot-rollout (robot)
 so101_follower┘   (teleop loop)    parquet + mp4 + meta/  (draccus CLI)   config.json     base|episodic|sentry|
 + cameras                          local cache <-> Hub                    model.safetensors highlight|dagger
 (calibration JSON per id)          ~/.cache/huggingface/lerobot/<repo>    pre/postprocessor ─► lerobot-eval (sim)
                                                                           train_config.json
```

## Config system

**Config system (verified):** every CLI is a [draccus](https://github.com/dlwh/draccus) dataclass.
Nested fields become dotted flags: `--robot.port`, `--dataset.repo_id`, `--policy.chunk_size`,
`--strategy.type`. `type` fields choose a registered subclass (`--robot.type=so101_follower`,
`--policy.type=act`). Dicts and lists are passed as YAML/JSON strings
(`--robot.cameras="{ top: {...} }"`, `--dataset.episodes='[0,1,2]'`). `lerobot-train` also takes
`--config_path=<file.json|yaml|Hub repo>`, and CLI flags override the file. **Every command
supports `--help`, and that is the ground truth for the installed version.**
