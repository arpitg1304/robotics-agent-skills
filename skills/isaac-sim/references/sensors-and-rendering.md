# Sensors and Rendering

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.
Field-level sensor tables are in `config-reference.md` (Sensors).

## Sensor types

| Sensor (`isaaclab.sensors`) | Measures | Cost and gotchas |
|-----------------------------|----------|------------------|
| `CameraCfg` | `rgb`, `depth`, `normals`, segmentation, ... as `(N, H, W, C)` `ProxyArray` in `data.output` | Rendering usually dominates step time. Tiling is built into `Camera`. `TiledCameraCfg` is a deprecated alias |
| `ContactSensorCfg` | `net_forces_w`, `net_normal_forces_w`, `force_matrix_w` (with `filter_prim_paths_expr`), air time | Needs `activate_contact_sensors=True` on the spawner. Filtered matrices cost more |
| `RayCasterCfg` + `patterns.GridPatternCfg` / `LidarPatternCfg` | Height scans, lidar-like hits against `mesh_prim_paths` | Cheap. Hits only the listed meshes; use `MultiMeshRayCasterCfg` for moving targets |
| `ImuCfg` | `ang_vel_b`, `lin_acc_b` (includes gravity, like a real IMU) | The 2.x full-state "IMU" is now `PvaCfg` |
| `FrameTransformerCfg` | Poses of target frames relative to a source frame | Use it for end-effector or fingertip frames instead of hand-rolled transforms |
| `JointWrenchSensorCfg` | Joint reaction wrenches | Replaces `data.body_incoming_joint_wrench_b` |

```python
from isaaclab.sensors import CameraCfg
from isaaclab_newton.renderers import NewtonWarpRendererCfg

wrist_cam = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/hand/wrist_cam",
    update_period=1 / 30,                 # render at the policy/camera rate, not the physics rate
    height=96, width=96, data_types=["rgb", "depth"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, horizontal_aperture=20.955, clipping_range=(0.05, 5.0)),
    offset=CameraCfg.OffsetCfg(pos=(0.05, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0), convention="ros"),  # XYZW
    renderer_cfg=NewtonWarpRendererCfg(),  # or select with renderer= presets
)
# rgb = env.scene["wrist_cam"].data.output["rgb"].torch   -> (num_envs, 96, 96, 3)
```

## Renderers

Renderers: **Newton Warp** (`renderer=newton_renderer`) is fastest and lightest, with direct
lighting and no motion vectors. **OVRTX** (`renderer=ovrtx`) is the recommended kit-less RTX
path. **Isaac RTX** (`renderer=isaacsim_rtx`) is legacy, for runs that must stay in Isaac Sim.

Performance and correctness:

- Image memory scales with envs × H × W × channels × outputs, plus your vision encoder. Use
  small images (64-128 px), request only the buffers the policy consumes, and set
  `update_period` to the control rate.
- **Large RTX camera atlases can return black or corrupted images with no error** (about a 4 GiB
  Vulkan buffer limit, Isaac Lab issue #8412). Reduce resolution, camera count or SPP, or use
  `newton_renderer`. Save and inspect a frame from the **last** env before training
  (`env.write_image_to_file=true` on camera tasks that support it).
- Camera `offset.convention`: `world` is +X forward/+Z up, `ros` is +Z forward/-Y up, `opengl`
  is -Z forward/+Y up. A wrong convention points the camera at the floor.
- Physics-only sensors (contact, ray caster, IMU) are cheap. Give them `update_period=sim.dt`
  or the control period on purpose.
