# Demo Recorder

> **Eval artifact — not build-verified.** This is verbatim agent output kept unmodified
> as evidence for [EVAL_REPORT.md](../../EVAL_REPORT.md). Its `CMakeLists.txt` never
> installs the Python package or the node, so the build instructions below do not
> produce a runnable package. Read it as a baseline, not as a package to copy.

A ROS2 Python package for recording robot demonstration episodes during teleoperation.

## Overview

This package subscribes to camera images and joint states, and records them as structured episodes that can be used for learning from demonstration.

## Topics

- Subscribes to: `/camera/color/image_raw` (sensor_msgs/Image), `/joint_states` (sensor_msgs/JointState)
- Publishes: `~/status` (demo_recorder/RecordingStatus)

## Services

- `~/start_recording` - Start recording an episode (provide task name and operator)
- `~/stop_recording` - Stop recording and save to disk

## Usage

```bash
ros2 launch demo_recorder demo_recorder.launch.py
```

Start recording:
```bash
ros2 service call /demo_recorder/start_recording demo_recorder/srv/StartRecording "{task_name: 'pick_and_place', operator_name: 'user1'}"
```

Stop recording:
```bash
ros2 service call /demo_recorder/stop_recording demo_recorder/srv/StopRecording "{success_label: true}"
```

## Configuration

Edit `config/demo_recorder.yaml` to change topics and save directory.
