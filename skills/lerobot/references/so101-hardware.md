# SO-101 Hardware Details

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Extended notes for the SO-101
bring-up steps in SKILL.md: motor gearing, calibration file locations and reuse, and camera
configuration. For other robots, see [robots.md](robots.md).

## Motor setup (step 2)

On a Waveshare board, set both jumpers to channel `B` (USB). The SO-101 leader uses mixed gear
ratios: 1/191 for pan and elbow, 1/345 for shoulder lift, 1/147 for the wrist and gripper. The
follower uses 6 × STS3215 at 1/345. A wrong-geared motor on the leader feels stiff or sags.

## Calibration files (step 3)

The directory comes from the class `name` (`so_follower`/`so_leader`), not from the `--robot.type`
string. Override the root with `HF_LEROBOT_CALIBRATION` or `HF_LEROBOT_HOME`, or set
`--robot.calibration_dir`.

If a file for that id exists, connect/calibrate asks: *"Press ENTER to use provided calibration file
... or type 'c' ... to run calibration"*. ENTER writes the saved file to the motors. When the motor
registers do not match the file, `connect()` starts calibration on its own.

## Cameras (step 5)

- OpenCV config fields (verified): `index_or_path` (int or path), `fps`, `width`, `height`,
  `color_mode` (rgb), `rotation` (0/90/180/-90), `warmup_s`, `fourcc`, `backend`. If the camera
  rejects the requested `fps`, `width` or `height`, connect raises `RuntimeError: ... failed to set
  fps=...` (or `capture_width`). A rejected `fourcc` only logs a warning ("Continuing with
  default format"), so check the log when you rely on MJPG for bandwidth. Use the exact
  fps/resolution profile that `lerobot-find-cameras` reports as supported.
- On Linux, `find_cameras` scans `/dev/video*`. **Advice:** a UVC camera often registers two
  nodes, so a second camera is usually `/dev/video2`, not `/dev/video1`. Use
  `/dev/v4l/by-id/...` or `/dev/v4l/by-path/...` so names survive a reboot. On macOS, indices
  move when an iPhone Continuity Camera or a new webcam shows up. Run `lerobot-find-cameras`
  again after any USB change.
- **Advice (USB bandwidth):** two uncompressed YUYV 640×480@30 streams on one hub can saturate it.
  Use `fourcc: MJPG`, put the cameras on separate root ports, and keep 640×480. The docs also
  call 640×480 at 30 fps enough for most tasks.
- RealSense: `{type: intelrealsense, serial_number_or_name: "233522074606", width: 640, height: 480, fps: 30, use_depth: true}`
  adds a `<name>_depth` feature.
