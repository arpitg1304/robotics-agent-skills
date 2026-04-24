---
name: ros2-web-integration
description: "Integrates ROS2 systems with web technologies including REST APIs, WebSocket bridges, and browser-based robot interfaces. Use this skill when building web dashboards for robots, streaming camera feeds to browsers, exposing ROS2 services as REST endpoints, or implementing bidirectional WebSocket communication between web UIs and ROS2 nodes. Trigger whenever the user mentions rosbridge, roslibjs, FastAPI with ROS2, Flask with rclpy, WebSocket for robot telemetry, MJPEG streaming, WebRTC for robots, REST API wrapping ROS2 services, web-based robot control, browser robot interface, robot dashboard, CORS configuration for robots, or any web-to-ROS2 bridge pattern. Also trigger for authentication on robot web interfaces, rate limiting sensor streams, video streaming from robot cameras, or running async web frameworks alongside the ROS2 executor. Covers rosbridge_suite, FastAPI, Flask, WebSocket, and WebRTC."
---

# ROS2 Web Integration Skill

## When to Use This Skill
- Building a web dashboard to monitor or control a robot running ROS2
- Streaming camera feeds (MJPEG, WebRTC, compressed WebSocket) from a robot to a browser
- Exposing ROS2 services and actions as REST API endpoints
- Implementing bidirectional WebSocket communication between a web UI and ROS2 nodes
- Setting up rosbridge_suite for quick prototyping or foxglove integration
- Writing a custom FastAPI or Flask bridge to ROS2 for production deployments
- Adding authentication, rate limiting, or CORS to robot web interfaces
- Running an async web server (uvicorn) alongside the rclpy executor without deadlocks
- Publishing teleop commands from a browser joystick to cmd_vel
- Serving ROS2 parameter configuration pages or diagnostic dashboards over HTTP

## Architecture Comparison

| Feature | rosbridge_suite | Custom FastAPI Bridge | Custom Flask Bridge |
|---|---|---|---|
| Latency | ~5-15ms (WebSocket) | ~2-5ms (WebSocket), ~10-30ms (REST) | ~10-50ms (REST only without extensions) |
| Throughput | Medium (JSON serialization overhead) | High (binary WebSocket, async) | Low-Medium (sync, GIL-bound) |
| Auth | Basic (rosauth, limited) | Full (JWT, OAuth2, API keys) | Full (Flask-Login, JWT) |
| Complexity | Low (launch and connect) | Medium (must manage two event loops) | Medium (must manage threading) |
| Video Streaming | Requires separate web_video_server | Native (MJPEG, WebSocket binary) | MJPEG via generator responses |
| Production Ready | No (exposes full topic graph) | Yes | Yes (with gunicorn) |
| When to Use | Prototyping, foxglove, quick demos | Production APIs, high-perf streaming | Simple internal tools, legacy systems |

## Decision Guide

Use **rosbridge_suite** when you need a working bridge in under 10 minutes, the client is foxglove/webviz, security is not a concern, and you do not need custom business logic.

Use a **custom bridge** (FastAPI/Flask) when you need authentication, want to expose only specific topics, need data transformation, REST endpoints, video streaming control, or are deploying to production.

## Pattern 1: rosbridge_suite

Drop-in WebSocket bridge for ROS2. Install, launch, and connect with roslibjs from the browser. Best for prototyping and foxglove integration.

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```javascript
const ros = new ROSLIB.Ros({ url: 'ws://robot-host:9090' });

// Subscribe with throttling to avoid flooding the browser
const imageTopic = new ROSLIB.Topic({
  ros, name: '/camera/image/compressed',
  messageType: 'sensor_msgs/msg/CompressedImage',
  throttle_rate: 100, queue_size: 1
});
imageTopic.subscribe((msg) => {
  document.getElementById('camera-feed').src = 'data:image/jpeg;base64,' + msg.data;
});
```

**Key limitations:** JSON serialization overhead (~30% inflation for binary data), no topic filtering by default (exposes entire ROS2 graph), single-threaded Tornado event loop, minimal auth (rosauth MAC tokens only).

For complete implementation including SSL launch, service calls, and joystick teleop, see [bridge implementations reference](references/bridge-implementations.md).

## Pattern 2: Custom FastAPI Bridge

Production-grade async bridge. Runs uvicorn in the main thread, rclpy `MultiThreadedExecutor` in a background thread, with thread-safe shared state between them.

```python
# Core pattern: ROS2 node exposes data via lock-protected shared state
class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        self._lock = threading.Lock()
        self._latest_image: Optional[bytes] = None
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            CompressedImage, '/camera/image/compressed',
            self._image_cb, sensor_qos)

    def _image_cb(self, msg):
        with self._lock:
            self._latest_image = bytes(msg.data)

    def get_latest_image(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_image
```

**Critical integration:** Uvicorn and rclpy must run on separate threads with coordinated shutdown via signal handlers. Never call `rclpy.spin_once()` from a request handler.

For the full FastAPI app, WebSocket streaming endpoints, REST service wrappers, main.py entry point, and project structure, see [bridge implementations reference](references/bridge-implementations.md).

## Pattern 3: Flask Bridge

Synchronous bridge using a background thread for the ROS2 executor. Simpler than FastAPI but limited to REST (no native WebSocket).

```python
# rclpy spins in background thread, Flask serves in main thread
executor = MultiThreadedExecutor()
executor.add_node(ros_node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
app.run(host='0.0.0.0', port=8080, threaded=True)
```

Use Flask when you only need simple REST endpoints, have few concurrent users, or your team is not ready for async. Use FastAPI when you need WebSocket streaming, high concurrency, or auto-generated OpenAPI docs.

For the complete Flask node implementation with BAD/GOOD threading examples, see [bridge implementations reference](references/bridge-implementations.md).

## Video Streaming

Three approaches ranked by complexity and quality:

| Method | Latency | Client Complexity | Best For |
|---|---|---|---|
| MJPEG over HTTP | Medium (~100ms) | Zero (native `<img>` tag) | Simple dashboards |
| Binary WebSocket | Low (~50ms) | Minimal JS | Multi-client streaming |
| WebRTC (webrtc_ros) | Very low (~30ms) | Moderate (WebRTC API) | High-quality, low-latency |

**MJPEG** requires no client-side JavaScript -- just `<img src="http://robot:8080/video/mjpeg" />`. Use binary WebSocket for ~30% bandwidth savings over base64 JSON. Use WebRTC when latency is critical.

For streaming endpoint implementations and adaptive quality reduction, see [bridge implementations reference](references/bridge-implementations.md).

## Bidirectional Communication

Key patterns for web-to-robot and robot-to-web communication:

- **Teleop with safety watchdog:** Accept joystick commands over WebSocket; publish zero velocity if no command received within 500ms to prevent runaway on disconnect.
- **Status broadcast:** Push robot state to all connected WebSocket clients; track client set and prune dead connections.
- **Command acknowledgment:** Use correlation IDs for reliable command execution (`{"id": "cmd-001", "action": "navigate_to"}`), with status progression: accepted -> in_progress -> completed.

For full implementations of TeleopHandler, StatusBroadcaster, and command acknowledgment patterns, see [bridge implementations reference](references/bridge-implementations.md).

## Critical Anti-Patterns

The three most common mistakes that cause production incidents:

1. **Blocking the executor from HTTP handlers** -- Never call `rclpy.spin_once()` inside a route handler. Spin the executor in a dedicated background thread; handlers read lock-protected shared state only.

2. **Streaming raw images over WebSocket** -- Subscribe to `CompressedImage` topics, not raw `Image`. Use `send_bytes()` for binary frames. Raw BGR8 at 640x480 is 921KB/frame; compressed JPEG is 30-80KB.

3. **No rate limiting on sensor streams** -- Always apply per-client rate limiting (token bucket or time-based throttle). A 20Hz LiDAR with 100K points generates ~8MB/s per client without throttling.

For all eight anti-patterns with BAD/GOOD code examples (including rosbridge auth, sync service calls, GIL contention, connection lifecycle, and topic allowlists), see [anti-patterns reference](references/anti-patterns.md).

## Security Essentials

- **TLS termination:** Use nginx/caddy as a reverse proxy; bridge listens on localhost only. Set `proxy_buffering off` for streaming endpoints.
- **JWT auth on mutation endpoints:** POST/PUT/teleop WebSocket require valid tokens. WebSocket auth uses query parameter (`?token=...`) since headers are not supported during upgrade.
- **CORS:** Always specify explicit `allow_origins` with dashboard URLs. Never use wildcard `*` in production.
- **Topic allowlist:** Expose only explicitly listed topics from configuration. Reject requests for topics not on the list.
- **Network segmentation:** Bridge has two interfaces (user network + robot VLAN). ROS2 DDS discovery is confined to the robot VLAN via `ROS_DOMAIN_ID`.

For full nginx config, JWT middleware implementation, and CORS examples, see [bridge implementations reference](references/bridge-implementations.md).

## Production Checklist

1. **Thread separation** -- rclpy executor in background thread, web server in main thread, never sharing an event loop
2. **Lock-protected shared state** -- Every cross-thread data access guarded by `threading.Lock()`
3. **Graceful shutdown** -- Signal handlers set shutdown event, executor stopped before `rclpy.shutdown()`, spin thread joined with timeout
4. **Topic allowlist** -- Only explicitly listed topics exposed, loaded from config
5. **Per-client rate limiting** -- Token bucket or time-based throttle on every WebSocket stream
6. **Command timeout watchdog** -- Zero velocity published after 500ms silence on teleop endpoints
7. **Compressed video** -- `CompressedImage` topics or server-side JPEG encoding; never forward raw `Image`
8. **TLS via reverse proxy** -- Bridge on localhost only; nginx handles TLS and WebSocket upgrade
9. **Auth on mutations** -- JWT or API key on POST/PUT/DELETE and teleop WebSocket; read-only may be unauthenticated on private networks
10. **CORS restricts origins** -- Specific dashboard URLs only, never wildcard
11. **Connection lifecycle** -- WebSocket clients tracked in a set, removed on disconnect, dead references pruned
12. **Binary WebSocket for high-bandwidth data** -- `send_bytes()` for images and point clouds, not base64 JSON
