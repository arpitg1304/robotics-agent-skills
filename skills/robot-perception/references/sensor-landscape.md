# Sensor Landscape Reference

Detailed sensor comparison tables and hardware details for the robot-perception skill.

## Sensor Types and Characteristics

```
Sensor Type        Output              Range       Rate     Best For
─────────────────────────────────────────────────────────────────────────
RGB Camera         (H,W,3) uint8       ∞           30-120Hz Object detection, tracking, visual servoing
Stereo Camera      (H,W,3)+(H,W,3)    0.3-20m     30-90Hz  Dense depth from passive stereo
Structured Light   (H,W) float + RGB   0.2-10m     30Hz     Indoor manipulation, short range
ToF Depth          (H,W) float + RGB   0.1-10m     30Hz     Indoor, medium range
LiDAR (spinning)   (N,3) or (N,4)     0.5-200m    10-20Hz  Outdoor navigation, mapping
LiDAR (solid-st.)  (N,3)              0.5-200m    10-30Hz  Automotive, outdoor
IMU                (6,) or (9,)        N/A         200-1kHz Orientation, motion estimation
Force/Torque       (6,) float          N/A         1kHz+    Contact detection, force control
Tactile            (H,W) or (N,3)      Contact     30-100Hz Grasp quality, texture
Event Camera       Events (x,y,t,p)    ∞           μs       High-speed tracking, HDR scenes
```

## Common Sensor Hardware

```
Device             Type               SDK/Driver           ROS2 Package
──────────────────────────────────────────────────────────────────────────
Intel RealSense    Structured Light   pyrealsense2         realsense2_camera
Stereolabs ZED     Stereo + IMU       pyzed                zed_wrapper
Luxonis OAK-D      Stereo + Neural    depthai              depthai_ros
FLIR/Basler        Industrial RGB     PySpin/pypylon       spinnaker_camera_driver
Velodyne           Spinning LiDAR     velodyne_driver      velodyne
Ouster             Spinning LiDAR     ouster-sdk           ros2_ouster
Livox              Solid-state LiDAR  livox_sdk            livox_ros2_driver
USB Webcam         RGB                OpenCV VideoCapture  usb_cam / v4l2_camera
```

## Sensor Selection Guidelines

- **Indoor manipulation (< 3m):** Structured light (RealSense D4xx) or stereo (ZED/OAK-D) for depth + RGB
- **Outdoor navigation:** Spinning LiDAR (Velodyne/Ouster) for 360-degree range + RGB camera for detection
- **High-speed tracking:** Event cameras or high-FPS global shutter industrial cameras (FLIR/Basler)
- **Contact-rich manipulation:** Tactile sensors (GelSight, DIGIT) for grasp quality feedback
- **Multi-sensor rigs:** Combine modalities (e.g., LiDAR + camera) with hardware sync for accurate fusion
