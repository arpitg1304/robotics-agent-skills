# Eval Report: Skills Impact on Robotics Code Generation

## Setup

**Prompt**: Build a ROS2 Python package called `demo_recorder` for recording robot
demonstration episodes during teleoperation. (See [PROMPT.md](PROMPT.md) for full prompt.)

**Model**: Claude Opus 4.6

| Run | Skills Loaded | Agent Prompt |
|-----|--------------|--------------|
| `without-skills/` | None — base model knowledge only | [PROMPT.md](PROMPT.md) |
| `with-skills/` | All 5 skills as raw context, no extra instructions | [PROMPT.md](PROMPT.md) + [AGENT_PROMPT.md](AGENT_PROMPT.md) |

Both runs received the **same task prompt**. The only difference is whether the 5 skill
files (ros2, robot-perception, robotics-design-patterns,
robotics-software-principles, robotics-testing) were loaded as context. The agent was
given no explicit design instructions — it had to independently decide which patterns
from the skills to apply.

---

## Summary Metrics

| Metric | Without Skills | With Skills | Delta |
|--------|---------------|-------------|-------|
| Total lines of code | 336 | 2,107 | **6.3x** |
| Main node (`.py`) | 184 | 761 | **4.1x** |
| Files | 13 | 15 | +2 (test + episode_writer) |
| Config parameters | 4 | 13 | **3.3x** |
| Status msg fields | 6 | 12 | **2x** |
| Launch arguments | 0 | 8 | — |
| Tests | 0 | 601 lines | — |
| Separate modules | 1 (node only) | 2 (node + episode_writer) | +1 |

---

## Side-by-Side Comparison

### 1. Node Architecture

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
class DemoRecorderNode(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        # Everything in __init__
        # No lifecycle management
```

Plain `rclpy.Node`. All setup in constructor.
No deterministic startup/shutdown.

</td>
<td>

```python
class DemoRecorderNode(LifecycleNode):
    def on_configure(self, state):
        # Allocate resources, create subs/pubs
    def on_activate(self, state):
        # Start timers
    def on_deactivate(self, state):
        # Auto-save, stop timers
    def on_cleanup(self, state):
        # Release resources
    def on_shutdown(self, state):
        # Auto-save on shutdown
```

Lifecycle node with 5 transition callbacks.
Auto-saves in-progress episodes on deactivate/shutdown.
Separate `EpisodeWriter` module (single responsibility).

</td>
</tr>
</table>

### 2. QoS Profiles

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
# Default QoS (depth=10) for everything
self.create_subscription(
    Image, topic, self.image_callback, 10)
self.create_subscription(
    JointState, topic, self.joint_callback, 10)
self.create_publisher(
    RecordingStatus, '~/status', 10)
```

Same `depth=10` RELIABLE for sensors and status.
Potential silent failure if sensor publishes BEST_EFFORT.

</td>
<td>

```python
SENSOR_QOS = QoSProfile(
    reliability=BEST_EFFORT, depth=1,
    durability=VOLATILE)

RELIABLE_QOS = QoSProfile(
    reliability=RELIABLE, depth=10,
    durability=VOLATILE)

# Sensors use SENSOR_QOS
self.create_subscription(
    Image, topic, cb, SENSOR_QOS)
# Status uses RELIABLE_QOS
self.create_publisher(
    RecordingStatus, '~/status', RELIABLE_QOS)
```

Proper QoS matching: BEST_EFFORT for sensors
(won't silently fail), RELIABLE for services/status.

</td>
</tr>
</table>

### 3. Sensor Handling

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
def image_callback(self, msg):
    self.latest_image = msg

def joint_callback(self, msg):
    self.latest_joints = msg
```

Direct assignment. No thread safety.
No buffer overflow protection.
No health monitoring.

</td>
<td>

```python
class SensorBuffer:
    """Thread-safe bounded FIFO buffer."""
    def __init__(self, maxlen=2):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._drops = 0
        self._last_recv_time = None

    def push(self, msg):
        with self._lock:
            if len(self._buf) == self._buf.maxlen:
                self._drops += 1
            self._buf.append(msg)
            self._last_recv_time = time.monotonic()

# Callbacks just push — never block
def _on_image(self, msg):
    self._camera_buf.push(msg)

# Watchdog detects sensor dropout
def _sensor_alive(self, buf):
    t = buf.last_recv_time
    if t is None:
        return False
    return (time.monotonic() - t) < self._sensor_timeout
```

Thread-safe bounded buffers with drop counting.
Watchdog timeout for sensor health.
Callbacks never block the executor.

</td>
</tr>
</table>

### 4. Timestamp Synchronization

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

No synchronization.
Uses whatever `latest_image` and `latest_joints` happen to be,
regardless of timestamp offset.

</td>
<td>

```python
# Check sync offset between sensors
joint_stamp_ns = (joint_msg.header.stamp.sec * 1e9
                  + joint_msg.header.stamp.nanosec)
image_stamp_ns = (image_msg.header.stamp.sec * 1e9
                  + image_msg.header.stamp.nanosec)
sync_offset_ms = abs(joint_stamp_ns - image_stamp_ns) / 1e6

if sync_offset_ms > self._max_sync_offset_ms:
    # Skip this timestep — data too far apart
    self._episode.dropped_frames += 1
    return
```

Computes per-timestep sync offset.
Skips timesteps that exceed configurable threshold.
Records average sync offset in episode metadata.

</td>
</tr>
</table>

### 5. Episode Data Structure

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
# Flat list of dicts
self.current_episode = []
self.current_episode.append(timestep)

# Save with basic metadata
episode_data = {
    'metadata': {
        'task_name': ...,
        'operator': ...,
        'timestamp': ...,
        'success': ...,
        'num_timesteps': ...,
        'episode_id': ...,
    },
    'timesteps': self.current_episode
}
```

6 metadata fields. No quality metrics.
Save logic embedded in the node.

</td>
<td>

```python
# episode_writer.py (separate module)
class Episode:
    """Structured container with quality metrics."""
    def __init__(self, episode_id, task_name, operator_name):
        self.timesteps = []
        self.dropped_frames = 0
        self.sync_offsets_ms = []

class EpisodeWriter:
    """Persistence + quality validation."""
    def save(self, episode, success_label, notes=""):
        # Quality gate: warn on low timesteps
        # Quality gate: warn on high drop ratio
        # Serialize and write to disk
```

Dedicated Episode class + EpisodeWriter module.
10 metadata fields including dropped frames, sync
offset, duration, operator notes. Quality validation
on save. Single responsibility separation.

</td>
</tr>
</table>

### 6. Graceful Degradation

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
def record_timestep(self):
    if self.latest_image is None or \
       self.latest_joints is None:
        return  # Silently skip forever
```

If either sensor never publishes, recording never starts.
No logging, no feedback, no partial recording.

</td>
<td>

```python
# Camera dropout → record joints only
if not camera_alive:
    self.get_logger().debug(
        "Camera offline -- recording joints only.",
        throttle_duration_sec=5.0)

# Joint dropout → skip (mandatory sensor)
if not joints_alive:
    self.get_logger().warn(
        "Joint state sensor timeout -- skipping.",
        throttle_duration_sec=2.0)
    self._episode.dropped_frames += 1
    return

# cv_bridge unavailable → metadata only
try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # graceful in CI/headless
```

Camera loss doesn't halt recording.
Joint loss is logged with throttling.
cv_bridge failure falls back to metadata-only images.

</td>
</tr>
</table>

### 7. Configuration

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```yaml
demo_recorder:
  ros__parameters:
    save_directory: '/tmp/demo_recordings'
    camera_topic: '/camera/color/image_raw'
    joint_states_topic: '/joint_states'
    status_publish_rate: 1.0
```

4 parameters. No descriptors. No runtime change callback.

</td>
<td>

```yaml
demo_recorder:
  ros__parameters:
    camera_topic: "/camera/wrist/color/image_raw"
    joint_states_topic: "/franka/joint_states"
    save_directory: "/tmp/demo_recordings"
    capture_rate_hz: 30.0
    status_publish_rate_hz: 2.0
    sensor_timeout_sec: 1.0
    buffer_size: 2
    max_sync_offset_ms: 50.0
    min_episode_timesteps: 10
    max_dropped_frame_ratio: 0.05
    image_encoding: "bgr8"
    save_images: true
    expected_num_joints: 7
```

13 parameters with `ParameterDescriptor`, `FloatingPointRange`,
`IntegerRange`. Runtime change callback for hot-reload.
Every tunable is externalized.

</td>
</tr>
</table>

### 8. Message/Service Definitions

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

**RecordingStatus.msg** (6 fields):
```
builtin_interfaces/Time stamp
bool is_recording
string task_name
string operator_name
uint32 episode_count
uint32 current_timestep
```

**StopRecording.srv**:
```
bool success_label
---
bool success
string message
string file_path
```

</td>
<td>

**RecordingStatus.msg** (12 fields):
```
builtin_interfaces/Time stamp
bool is_recording
string task_name
string operator_name
uint32 episode_count
uint32 current_timestep
bool camera_alive            # NEW
bool joint_states_alive      # NEW
float64 sync_offset_ms       # NEW
float64 avg_capture_rate_hz  # NEW
uint32 dropped_frames        # NEW
```

**StopRecording.srv**:
```
bool success_label
string notes                 # NEW
---
bool success
string message
string file_path
uint32 num_timesteps         # NEW
float64 duration_sec         # NEW
uint32 dropped_frames        # NEW
```

</td>
</tr>
</table>

### 9. Launch File

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('demo_recorder'),
        'config', 'demo_recorder.yaml')
    return LaunchDescription([
        Node(
            package='demo_recorder',
            executable='demo_recorder_node',
            name='demo_recorder',
            parameters=[config],
            output='screen'),
    ])
```

Plain Node. No arguments. No lifecycle orchestration.

</td>
<td>

```python
def generate_launch_description():
    # 8 launch arguments:
    # config_file, node_name, camera_topic,
    # joint_states_topic, save_directory,
    # capture_rate_hz, autostart, log_level

    recorder_node = LifecycleNode(...)

    # Auto-lifecycle: configure → activate
    configure_event = EmitEvent(
        event=ChangeState(
            transition_id=TRANSITION_CONFIGURE),
        condition=IfCondition(
            LaunchConfiguration("autostart")))

    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            goal_state="inactive",
            entities=[EmitEvent(
                event=ChangeState(
                    transition_id=TRANSITION_ACTIVATE))]))
```

LifecycleNode with auto-configure/activate.
8 launch arguments for operator overrides.
Log-level passthrough.

</td>
</tr>
</table>

### 10. Tests

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

**No tests.**

</td>
<td>

**601 lines of tests covering:**

- `TestSensorBuffer` (6 tests) — push/latest, overflow drops, thread safety, clear
- `TestEpisode` (5 tests) — timesteps, serialization, disk persistence, dropped frames
- `TestEpisodeWriter` (5 tests) — file creation, notes, quality warnings, directory creation
- `TestLifecycleTransitions` (4 tests) — configure, activate, deactivate, cleanup
- `TestRecordingServices` (4 tests) — start, stop, idempotent start, idempotent stop, full record cycle
- `TestSensorHealth` (3 tests) — camera alive, joint alive, timeout detection
- `TestDeactivateAutoSave` (1 test) — auto-save on deactivate

**Includes mock hardware:**
- `MockCamera` — publishes synthetic 640x480 BGR8 images at 30Hz
- `MockJointStatePublisher` — publishes synthetic 7-DOF sinusoidal joint states at 100Hz

**Graceful test env:**
- Pure-Python tests run without ROS2 (SensorBuffer, Episode, EpisodeWriter)
- ROS2 integration tests skip with `@pytest.mark.skipif(not ROS2_AVAILABLE)`

</td>
</tr>
</table>

### 11. Entry Point

<table>
<tr><th>Without Skills</th><th>With Skills</th></tr>
<tr>
<td>

```python
def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Single-threaded executor. No KeyboardInterrupt handling.

</td>
<td>

```python
def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

Multi-threaded executor (4 threads) for concurrent
service handling + sensor callbacks. Proper cleanup.

</td>
</tr>
</table>

---

## Patterns Independently Applied by the Agent

The agent was given raw skill files as context with no explicit instructions on which
patterns to use. It independently identified and applied every relevant pattern:

| Pattern Applied | Source Skill |
|----------------|-------------|
| Lifecycle node (on_configure/activate/deactivate/cleanup/shutdown) | ros2 |
| QoS profiles (BEST_EFFORT for sensors, RELIABLE for status) | ros2 |
| ParameterDescriptor with FloatingPointRange/IntegerRange | ros2 |
| Runtime parameter change callback | ros2 |
| ament_cmake + rosidl for custom msgs | ros2 |
| Launch arguments + auto-lifecycle orchestration | ros2 |
| MultiThreadedExecutor with callback groups | ros2 |
| Threaded bounded sensor buffers (SensorBuffer class) | robot-perception |
| Software timestamp synchronization with configurable threshold | robot-perception |
| Sensor health watchdog with timeout detection | robot-perception |
| Camera dropout → joints-only recording | robot-perception |
| Episode container with quality metrics (drops, sync, duration) | robotics-design-patterns |
| Auto-save on deactivate/shutdown | robotics-design-patterns |
| Single responsibility: separate EpisodeWriter module | robotics-software-principles |
| Fail-safe defaults (sensor timeout, bounded buffers) | robotics-software-principles |
| Configuration over code (13 externalized params) | robotics-software-principles |
| Idempotent services (no side-effects on duplicate calls) | robotics-software-principles |
| Graceful degradation (cv_bridge fallback, partial sensor data) | robotics-software-principles |
| Observability (12-field status message with health + metrics) | robotics-software-principles |
| Separation of rates (capture timer vs status timer) | robotics-software-principles |
| pytest fixtures with mock hardware | robotics-testing |
| Unit + integration test separation | robotics-testing |
| ROS2-optional test gating (`@pytest.mark.skipif`) | robotics-testing |

---

## Known Defects in the Generated Artifacts

Both packages are **verbatim agent output**, kept unmodified so the comparison stays
honest. Neither has been built against a real ROS2 distribution, and a review found
build defects in both. They are recorded here rather than patched, because editing the
artifacts would destroy their value as evidence.

| Defect | Run | Effect |
|--------|-----|--------|
| `CMakeLists.txt` generates interfaces but never calls `ament_python_install_package` or installs the node | without-skills | Nothing is installed; `ros2 run demo_recorder demo_recorder_node` and the launch file both fail |
| `install(PROGRAMS demo_recorder/demo_recorder_node.py DESTINATION lib/${PROJECT_NAME})` has no `RENAME`, so the executable is named `demo_recorder_node.py` | with-skills | The launch file requests `executable="demo_recorder_node"` and fails to resolve it |
| `setup.py` ships alongside `<build_type>ament_cmake</build_type>` | both | Dead file — never invoked by the ament_cmake build; the install rules in `CMakeLists.txt` are what count |

The with-skills defect is a one-line fix (`RENAME demo_recorder_node`); the
without-skills package needs install rules written from scratch. That difference is
itself a data point, but neither package should be copied as-is.

**What this means for the metrics:** the table above measures the *design decisions* in
the generated code — lifecycle transitions, QoS selection, buffering, test coverage. It
does not measure whether the code runs, because neither run was build-verified.

---

## Methodology Caveats

Read the numbers with these limits in mind:

- **n=1.** One prompt, one task, one model, one run per arm. No repeated sampling, so
  run-to-run variance is unmeasured and the deltas below are not significance-tested.
- **Self-assessed.** The comparison was written by the same system that produced the
  artifacts. No independent robotics reviewer graded either package.
- **Line counts are a proxy, not a result.** "6.3x total lines" measures volume, and
  volume is the axis a language model inflates for free. 2,107 lines for an episode
  recorder is arguably over-engineered for some deployments. The defensible claims are
  the categorical ones: lifecycle transitions present vs. absent, sensor QoS chosen
  deliberately vs. left at default, 601 lines of tests vs. zero.
- **Not build-verified.** See the defects above.

Adding a build-verified, multi-run eval across several task prompts is the highest-value
improvement to this directory. See [CONTRIBUTING.md](../CONTRIBUTING.md).

---

## Conclusion

The **without-skills** version is a naive implementation. Beyond its missing install
rules, it would fail in production due to QoS mismatches, no sensor health monitoring,
no graceful degradation, and no tests.

The **with-skills** version applies production robotics patterns throughout. The agent was
not told which patterns to use — it read the skills, recognized which ones were relevant
to the task, and applied them independently. The result is a design a senior robotics
engineer would recognize as production-shaped — though, as noted above, it still needs a
build fix before it runs.

**The skills didn't make the agent more capable — they made it more experienced.**
