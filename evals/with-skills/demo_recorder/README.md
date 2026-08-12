# demo_recorder

> **Eval artifact — not build-verified.** This is verbatim agent output kept unmodified
> as evidence for [EVAL_REPORT.md](../../EVAL_REPORT.md). It has a known build defect:
> `install(PROGRAMS ...)` in `CMakeLists.txt` lacks `RENAME demo_recorder_node`, so the
> installed executable is `demo_recorder_node.py` and the launch file cannot resolve it.
> Read it for the patterns, not as a package to copy.

A production-grade ROS2 package for recording robot demonstration episodes
during teleoperation.  Designed for a 7-DOF Franka Emika Panda arm with a
wrist-mounted Intel RealSense camera.

## Architecture

The package is built around a **lifecycle node** (`DemoRecorderNode`) that
transitions through deterministic states:

```
[Unconfigured] --configure--> [Inactive] --activate--> [Active] --deactivate--> [Inactive]
```

Key design patterns applied:

| Pattern | Where |
|---|---|
| Lifecycle node | Deterministic startup/shutdown, auto-save on deactivate |
| Single responsibility | `EpisodeWriter` handles persistence/quality; node handles capture |
| Threaded sensor capture | `SensorBuffer` -- bounded deque, never blocks callbacks |
| Software timestamp sync | Capture loop checks camera/joint offsets against configurable window |
| Episode recorder | `Episode` container with streams, metadata, quality metrics |
| QoS profiles | BEST_EFFORT depth-1 for sensors, RELIABLE depth-10 for services/status |
| Fail-safe defaults | Watchdog pauses recording on sensor loss |
| Configuration over code | All tunables in YAML, runtime-changeable via `ros2 param set` |
| Idempotent services | start-while-recording and stop-while-idle return descriptive failures |
| Observability | `RecordingStatus` msg carries alive flags, sync offset, capture rate, drops |
| Graceful degradation | Records joints-only if camera drops out; works without cv_bridge |

## Topics

| Topic | Type | QoS | Direction |
|---|---|---|---|
| `/camera/wrist/color/image_raw` | `sensor_msgs/Image` | BEST_EFFORT | Subscribe |
| `/franka/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT | Subscribe |
| `~/status` | `demo_recorder/RecordingStatus` | RELIABLE | Publish |

## Services

| Service | Type | Description |
|---|---|---|
| `~/start_recording` | `demo_recorder/StartRecording` | Begin a new episode (task name + operator) |
| `~/stop_recording` | `demo_recorder/StopRecording` | End the episode, save to disk, return path |

## Parameters

All parameters are declared with descriptors and ranges.  See
`config/demo_recorder.yaml` for defaults and documentation.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_topic` | string | `/camera/wrist/color/image_raw` | Camera image topic |
| `joint_states_topic` | string | `/franka/joint_states` | Joint state topic |
| `save_directory` | string | `/tmp/demo_recordings` | Episode output directory |
| `capture_rate_hz` | float | 30.0 | Timestep capture rate |
| `status_publish_rate_hz` | float | 2.0 | Status publish rate |
| `sensor_timeout_sec` | float | 1.0 | Sensor watchdog timeout |
| `buffer_size` | int | 2 | Bounded buffer depth per sensor |
| `max_sync_offset_ms` | float | 50.0 | Max camera/joint sync offset |
| `min_episode_timesteps` | int | 10 | Quality gate: min timesteps |
| `max_dropped_frame_ratio` | float | 0.05 | Quality gate: max drop ratio |
| `image_encoding` | string | `bgr8` | cv_bridge encoding |
| `save_images` | bool | true | Save full image arrays |
| `expected_num_joints` | int | 7 | Expected DOF for validation |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select demo_recorder
source install/setup.bash
```

## Launch

```bash
# Default (auto-configures and activates the lifecycle node):
ros2 launch demo_recorder demo_recorder.launch.py

# With overrides:
ros2 launch demo_recorder demo_recorder.launch.py \
    save_directory:=/data/demos \
    capture_rate_hz:=15.0 \
    log_level:=debug

# Manual lifecycle management (no autostart):
ros2 launch demo_recorder demo_recorder.launch.py autostart:=false
ros2 lifecycle set /demo_recorder configure
ros2 lifecycle set /demo_recorder activate
```

## Recording workflow

```bash
# Start an episode:
ros2 service call /demo_recorder/start_recording \
    demo_recorder/srv/StartRecording \
    "{task_name: 'pick_and_place', operator_name: 'alice'}"

# ... operator performs the demonstration ...

# Stop and save:
ros2 service call /demo_recorder/stop_recording \
    demo_recorder/srv/StopRecording \
    "{success_label: true, notes: 'clean execution'}"

# Monitor status:
ros2 topic echo /demo_recorder/status
```

## Episode format

Episodes are saved as JSON files under the configured `save_directory`:

```
episode_0001_20260302_143022.json
```

Structure:

```json
{
  "metadata": {
    "episode_id": 1,
    "task_name": "pick_and_place",
    "operator": "alice",
    "start_time": "2026-03-02T14:30:22.123456",
    "duration_sec": 12.345,
    "success": true,
    "notes": "clean execution",
    "num_timesteps": 370,
    "dropped_frames": 2,
    "avg_sync_offset_ms": 3.1
  },
  "timesteps": [
    {
      "timestamp_ns": 1709388622123456789,
      "observation": {
        "joint_names": ["panda_joint1", "..."],
        "joint_positions": [0.0, "..."],
        "joint_velocities": [0.0, "..."],
        "joint_efforts": [0.0, "..."],
        "image": "...",
        "image_shape": [480, 640, 3]
      },
      "action": {
        "joint_positions": [0.0, "..."]
      }
    }
  ]
}
```

## Testing

```bash
# Unit tests (no ROS2 required -- tests SensorBuffer, Episode, EpisodeWriter):
cd demo_recorder
pytest test/test_demo_recorder.py -v -k "TestSensorBuffer or TestEpisode or TestEpisodeWriter"

# Full integration tests (requires sourced ROS2 workspace):
colcon test --packages-select demo_recorder
colcon test-result --verbose
```

## License

Apache-2.0
