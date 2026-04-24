# Web Integration Anti-Patterns

Common mistakes when integrating ROS2 with web technologies, with BAD/GOOD examples for each.

## 1. Blocking the ROS2 Executor from HTTP Handlers

**Problem:** Calling `rclpy.spin_once()` or `rclpy.spin_until_future_complete()` inside an HTTP handler blocks the web server thread and can deadlock if the ROS2 executor is already spinning in another thread.

**Fix:** Spin the ROS2 executor in a dedicated background thread. Access data via thread-safe shared state (lock-protected attributes). Never call spin functions from request handlers.

```python
# BAD: Spinning inside a Flask route
@app.route('/api/scan')
def get_scan():
    rclpy.spin_once(node, timeout_sec=1.0)  # Blocks the web server thread
    return jsonify(node.latest_scan)

# GOOD: Executor spins in background, handler reads shared state
@app.route('/api/scan')
def get_scan():
    return jsonify(node.get_latest_scan())  # Lock-protected read, non-blocking
```

## 2. Streaming Raw Image Messages Over WebSocket

**Problem:** Sending raw `sensor_msgs/Image` data (uncompressed BGR8, 640x480 = 921,600 bytes per frame) over WebSocket wastes bandwidth and CPU on the client. Base64 encoding inflates it to 1.2MB per frame.

**Fix:** Subscribe to `CompressedImage` topics (JPEG/PNG) or compress on the server side before sending. Use binary WebSocket frames instead of base64 JSON.

```python
# BAD: Subscribing to raw image and base64-encoding it
self.create_subscription(Image, '/camera/image_raw', self._raw_cb, 10)
# Each frame: 921,600 bytes raw -> 1,228,800 bytes base64 -> JSON overhead

# GOOD: Subscribe to compressed topic, send as binary WebSocket frame
self.create_subscription(
    CompressedImage, '/camera/image/compressed', self._compressed_cb,
    QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1))
# Each frame: ~30,000-80,000 bytes JPEG, sent as binary
await websocket.send_bytes(compressed_image_bytes)
```

## 3. No Rate Limiting on Sensor Subscriptions

**Problem:** A LiDAR publishing at 20Hz with 100K points per scan generates ~8MB/s of data. Forwarding every message to every WebSocket client saturates the network and browser.

**Fix:** Apply server-side rate limiting per client. Use a token bucket or simple time-based throttle. Let clients request their desired rate.

```python
# BAD: Forward every message to every client
def _scan_cb(self, msg):
    for client in self.clients:
        client.send(serialize(msg))  # 20 msgs/s * N clients

# GOOD: Rate-limit per client
def _scan_cb(self, msg):
    with self._lock:
        self._latest_scan = msg  # Just store latest

async def stream_to_client(self, ws, max_hz=5):
    interval = 1.0 / max_hz
    while True:
        scan = self.get_latest_scan()
        if scan:
            await ws.send_json(scan)
        await asyncio.sleep(interval)
```

## 4. Running rosbridge in Production Without Auth

**Problem:** rosbridge_suite with default settings exposes every topic, service, and parameter to any WebSocket client. Any browser on the network can publish to `/cmd_vel` or call `/emergency_stop`.

**Fix:** For production, use a custom bridge with authentication. If you must use rosbridge, enable rosauth, restrict topics via a filter, and run behind an authenticated reverse proxy.

```yaml
# BAD: Default rosbridge launch — full access to everything
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# GOOD: At minimum, enable authentication and use a reverse proxy
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    authenticate:=true \
    topics_glob:="['/camera/image/compressed', '/odom', '/cmd_vel']"
```

## 5. Synchronous Service Calls in Async Handlers

**Problem:** Calling `service_client.call(request)` (synchronous) inside an `async def` FastAPI handler blocks the event loop, freezing all other requests until the service responds.

**Fix:** Use `call_async()` and await the future via `asyncio.get_event_loop().run_in_executor()`, or use a dedicated thread pool.

```python
# BAD: Synchronous service call blocks the async event loop
@app.post("/api/navigate")
async def navigate(goal: NavGoal):
    response = nav_client.call(goal_request)  # Blocks entire event loop
    return {"result": response.result}

# GOOD: Async service call with executor bridge
@app.post("/api/navigate")
async def navigate(goal: NavGoal):
    future = nav_client.call_async(goal_request)
    response = await asyncio.get_event_loop().run_in_executor(
        None, lambda: future.result(timeout=30.0)
    )
    return {"result": response.result}
```

## 6. GIL Contention Between Web Server and ROS2 Spinner

**Problem:** Running uvicorn with multiple worker threads and rclpy.spin in another thread causes GIL contention. Under high load, both the web server and ROS2 callbacks stall each other, leading to dropped messages and increased latency.

**Fix:** Use `MultiThreadedExecutor` with a small thread count (2-4). For high-throughput systems, run the ROS2 node in a separate process and communicate via shared memory, Redis, or a Unix socket.

```python
# BAD: SingleThreadedExecutor competing with uvicorn for the GIL
executor = SingleThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin).start()
uvicorn.run(app, workers=4)  # 4 workers + 1 spinner = GIL contention

# GOOD: MultiThreadedExecutor with limited threads, single uvicorn worker
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()
uvicorn.run(app, workers=1, host="0.0.0.0", port=8080)
```

## 7. No Connection Lifecycle Management

**Problem:** WebSocket clients disconnect without sending a close frame (browser tab closed, network drop). The server keeps sending data to dead connections, wasting CPU and memory. Over time, the dead client set grows unbounded.

**Fix:** Track connected clients in a set, remove on disconnect, and periodically prune stale connections with a heartbeat check.

```python
# BAD: No tracking of client lifecycle
clients = []

@app.websocket("/ws/data")
async def data_ws(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    while True:
        await ws.send_json(get_data())  # Throws on dead client, never removed

# GOOD: Proper lifecycle management with cleanup
clients: set[WebSocket] = set()

@app.websocket("/ws/data")
async def data_ws(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            # Wait for client messages (ping/pong keepalive)
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)

async def broadcast(data: dict):
    dead = set()
    for client in clients.copy():
        try:
            await client.send_json(data)
        except Exception:
            dead.add(client)
    clients -= dead
```

## 8. Exposing All Topics Unconditionally

**Problem:** The web bridge subscribes to every topic on the ROS2 graph and makes all of them available to web clients. This leaks internal system details (diagnostics, debug topics), wastes bandwidth, and creates a security risk.

**Fix:** Maintain an explicit allowlist of topics that should be exposed. Load it from configuration. Reject requests for topics not on the list.

```python
# BAD: Dynamically subscribe to whatever the client requests
@app.websocket("/ws/topic/{topic_name}")
async def any_topic(ws: WebSocket, topic_name: str):
    # Client can request /rosout, /parameter_events, /diagnostics, etc.
    sub = node.create_subscription(String, topic_name, callback, 10)

# GOOD: Allowlist of exposed topics from configuration
ALLOWED_TOPICS = {
    "/camera/image/compressed": CompressedImage,
    "/odom": Odometry,
    "/battery_state": BatteryState,
    "/cmd_vel": Twist,
}

@app.websocket("/ws/topic/{topic_name:path}")
async def allowed_topic(ws: WebSocket, topic_name: str):
    topic_path = "/" + topic_name
    if topic_path not in ALLOWED_TOPICS:
        await ws.close(code=4004, reason=f"Topic '{topic_path}' not in allowlist")
        return
    msg_type = ALLOWED_TOPICS[topic_path]
    # Proceed with subscription
```
