---
name: robot-perception
description: "Comprehensive best practices for robot perception systems covering cameras, LiDARs, depth sensors, IMUs, and multi-sensor setups. Use this skill when working with RGB image processing, depth maps, point clouds, sensor calibration (intrinsic, extrinsic, hand-eye), object detection, semantic segmentation, 3D reconstruction, visual servoing, or perception pipeline optimization. Trigger whenever the user mentions OpenCV, Open3D, PCL, RealSense, ZED, OAK-D, camera calibration, AprilTags, ArUco markers, stereo vision, RGBD, point cloud filtering, ICP registration, coordinate transforms, camera intrinsics, distortion correction, image undistortion, sensor streaming, frame synchronization, or any computer vision task in a robotics context. Also covers multi-camera rigs, time synchronization across sensors, perception latency budgets, and production deployment of perception pipelines."
---

# Robot Perception Skill

## When to Use This Skill
- Setting up and configuring camera, LiDAR, or depth sensors
- Building RGB, depth, or point cloud processing pipelines
- Calibrating cameras (intrinsic, extrinsic, hand-eye)
- Implementing object detection, segmentation, or tracking for robots
- Fusing data from multiple sensor modalities
- Streaming sensor data with proper threading and buffering
- Synchronizing multi-sensor rigs
- Deploying perception models on robot hardware (GPU, edge)
- Debugging perception failures (latency, dropped frames, misalignment)

## Sensor Landscape

Choose sensors based on range, environment, and task. Structured light (RealSense) and stereo (ZED/OAK-D) suit indoor manipulation under 3m. Spinning LiDAR covers outdoor navigation at 10-200m. Event cameras handle high-speed tracking and HDR scenes where conventional cameras blur.

For complete sensor comparison tables and hardware SDK mappings, see [sensor landscape reference](references/sensor-landscape.md).

## Camera Calibration

Three calibration types matter for robotics: intrinsic (lens parameters), extrinsic (sensor-to-sensor transforms), and hand-eye (camera-to-robot). Always calibrate with sub-pixel corner refinement and verify reprojection error against target thresholds.

**Critical pattern -- intrinsic calibration with coverage tracking:**

```python
class IntrinsicCalibrator:
    def collect_calibration_images(self, camera, num_images=30, min_coverage=0.6):
        """IMPORTANT: Move board to cover all image regions including
        corners and edges. Tilt at various angles. Bad coverage = bad
        calibration, especially at image edges."""
        coverage_map = np.zeros((4, 4), dtype=int)  # Track board positions
        # ... collect images, track which grid cells are covered ...
        coverage = (coverage_map > 0).sum() / coverage_map.size
        if coverage < min_coverage:
            print(f"WARNING: Only {coverage:.0%} coverage.")
```

**Quality thresholds:**
- Intrinsic RMS < 0.5 px (good), < 0.3 px (excellent); need 20+ images
- Hand-eye: 15+ poses across 3+ rotation axes; verification error < 5mm for manipulation
- Recalibrate after any physical bump, focus change, or significant temperature shift

For complete calibration code (intrinsic, extrinsic, hand-eye, stereo, camera-to-LiDAR), see [calibration and pipelines reference](references/calibration-and-pipelines.md#intrinsic-calibration).

## Sensor Streaming

Decouple capture from processing: run sensor capture in a dedicated thread with a bounded buffer (`deque(maxlen=2)`), timestamp at capture time (not processing time), and always consume the latest frame. This prevents processing bottlenecks from blocking capture and avoids unbounded memory growth.

**Critical pattern -- threaded capture with bounded buffer:**

```python
class CameraStream:
    """Capture in dedicated thread; processing never blocks capture;
    always use LATEST frame; timestamp at capture, not processing."""

    def __init__(self, camera, buffer_size=2, name="camera"):
        self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()

    def get_latest(self) -> Optional[StampedFrame]:
        with self._lock:
            return self._buffer[-1] if self._buffer else None
```

**Key streaming rules:**
- Never capture and process in the same thread -- you get the rate of the slowest operation
- Use `deque(maxlen=N)` not an unbounded list -- unbounded buffers grow until OOM
- Use `time.monotonic()` at capture, not `time.time()` at processing -- scheduling delays corrupt timestamps
- Use event-driven waits (`wait_for_frame`) not `time.sleep(1/fps)` -- sleep drift accumulates

**Multi-sensor sync:** Use nearest-neighbor timestamp matching within a tolerance (default 33ms for 30Hz sensors). Prefer hardware trigger (GPIO, PTP) over software sync for stereo depth, multi-camera reconstruction, or fast-moving scenes.

For complete streaming, synchronization, and hardware sync code, see [calibration and pipelines reference](references/calibration-and-pipelines.md#camera-streaming-architecture).

## RGB Processing Pipelines

Always undistort before any geometric computation. Pre-compute undistortion maps once (`cv2.initUndistortRectifyMap`) and reuse per frame via `cv2.remap` -- never recompute maps each frame.

For robotics-specific detection, wrap models with workspace filtering (ignore detections outside robot reach), stability tracking (reject flickering detections), and depth-based 3D backprojection. Use median depth over a small patch rather than a single pixel to handle sensor noise and holes.

**Critical pattern -- robust depth sampling for backprojection:**

```python
def _backproject(self, pixel, depth_image):
    u, v = int(pixel[0]), int(pixel[1])
    # Sample depth with a small window (more robust than single pixel)
    patch = depth_image[v-2:v+3, u-2:u+3]
    valid = patch[patch > 0]
    if len(valid) == 0:
        return None
    Z = np.median(valid)
    if Z > 100:  # Likely millimeters
        Z = Z / 1000.0
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z])
```

For complete code (undistortion, object detection, fiducial/ArUco detection, semantic segmentation for manipulation), see [calibration and pipelines reference](references/calibration-and-pipelines.md#image-undistortion).

## Depth and Point Cloud Processing

Raw depth is noisy: flying pixels at depth discontinuities, holes, and range noise. Apply a cleanup pipeline in order: range filter, edge/flying-pixel removal, small-hole inpainting, then bilateral smoothing (preserves edges). For point clouds, process in order: crop to workspace, remove statistical outliers, voxel downsample, then estimate normals oriented toward the camera.

**Critical pattern -- point cloud preprocessing order:**

```python
class PointCloudProcessor:
    def preprocess(self, points, workspace_bounds=None, voxel_size=0.005):
        # 1. Crop to workspace (reduces data volume first)
        # 2. Statistical outlier removal (nb_neighbors=20, std_ratio=2.0)
        # 3. Voxel downsample
        # 4. Estimate normals, orient toward camera
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 4, max_nn=30))
        pcd.orient_normals_towards_camera_location(camera_location=np.array([0, 0, 0]))
        return pcd
```

Use RANSAC plane segmentation to separate table/floor from objects, then DBSCAN clustering to isolate individual objects. For registration, point-to-plane ICP converges faster than point-to-point but requires normals; always provide a good initial transform -- ICP is local optimization.

For complete code (depth filtering, depth-to-pointcloud, RGBD alignment, plane segmentation, clustering, ICP, global registration), see [calibration and pipelines reference](references/calibration-and-pipelines.md#depth-map-filtering-and-cleanup).

## Multi-Sensor Fusion

The typical multi-sensor architecture flows sensors through per-modality processing (detection on RGB, filtering on depth, preprocessing on point clouds), then into a fusion layer that associates and tracks objects across modalities, producing a unified world model of tracked objects.

Two fusion strategies: early fusion (project LiDAR into camera frame, combine at data level) and late fusion (detect in each modality, associate results). Early fusion is simpler when sensors are well-calibrated.

**Critical pattern -- enriching 2D detections with LiDAR 3D positions:**

```python
def enrich_detections(self, detections_2d, lidar_points):
    pixels, depths, valid = self.project_lidar_to_image(lidar_points)
    for det in detections_2d:
        x1, y1, x2, y2 = det.bbox
        in_box = valid & (pixels[:, 0] >= x1) & (pixels[:, 0] <= x2) & \
                 (pixels[:, 1] >= y1) & (pixels[:, 1] <= y2)
        if in_box.sum() > 3:
            det.position_3d = np.median(lidar_points[in_box][:, :3], axis=0)
```

For tracking, use Hungarian algorithm assignment with 3D Euclidean distance (preferred) or 2D IoU. Require `min_hits` confirmations before trusting a track, and age out stale tracks after `max_age` frames without matches.

For complete fusion and tracking code, see [calibration and pipelines reference](references/calibration-and-pipelines.md#camera--lidar-fusion).

## Critical Anti-Patterns

**Processing every frame when you don't need to:**
```python
# WRONG: Running heavy detection at full sensor framerate
def callback(self, msg):
    detections = self.heavy_model(msg)  # 100ms on every 30Hz frame

# RIGHT: Decimate or skip frames
def callback(self, msg):
    self.frame_count += 1
    if self.frame_count % 3 != 0:  # Process every 3rd frame
        return
    detections = self.heavy_model(msg)
```

**Ignoring sensor warmup:**
```python
# WRONG: Using first frames from sensor
camera.start()
frame = camera.capture()  # Often overexposed, auto-exposure not settled

# RIGHT: Discard initial frames
camera.start()
for _ in range(30):  # Let auto-exposure settle (~1 second at 30Hz)
    camera.capture()
frame = camera.capture()  # Now usable
```

**Not handling coordinate frame transforms:**
```python
# WRONG: Assuming everything is in the same frame
object_position = detection.position  # Camera frame? Robot frame? World frame?
robot.move_to(object_position)

# RIGHT: Explicit frame tracking
object_in_camera = detection.position_3d
object_in_base = T_base_camera @ np.append(object_in_camera, 1.0)
robot.move_to(object_in_base[:3])
```

**Not validating depth values:**
```python
# WRONG: Using raw depth blindly -- may be 0 (hole) or 65535 (invalid)
z = depth_image[v, u]
point_3d = backproject(u, v, z)

# RIGHT: Always validate depth
z = depth_image[v, u]
if z <= 0 or z > max_range:
    return None
z_meters = z * depth_scale
```

## Perception Latency Budget

Target < 100ms total pipeline for 10Hz perception. Key rule: perception latency + planning latency must be less than the control period. If control runs at 100Hz (10ms), pipeline perception so frame N is processed while control acts on frame N-1.

For the full per-component latency breakdown, see [calibration and pipelines reference](references/calibration-and-pipelines.md#perception-latency-budget).

## Production Checklist

1. **Calibrate** intrinsics, extrinsics, and hand-eye before deploying
2. **Validate calibration** with independent measurements
3. **Set sensor exposure** -- auto-exposure causes detection flicker
4. **Undistort** before any geometric computation
5. **Filter depth** -- remove flying pixels, fill small holes
6. **Timestamp at capture** -- not at processing time
7. **Track objects** across frames -- do not rely on single-frame detections
8. **Handle sensor failures** -- missing frames, degraded depth, overexposure
9. **Log perception output** -- bounding boxes, confidence scores, 3D positions
10. **Benchmark latency** -- measure each pipeline stage, find bottlenecks
11. **Test edge cases** -- empty scenes, cluttered scenes, reflective surfaces, direct sunlight
12. **Version your models** -- pin detection/segmentation model versions in deployment
