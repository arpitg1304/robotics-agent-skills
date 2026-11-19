# LeRobot Robots and Teleoperators

Verified against LeRobot 0.7.1 (commit 7e57397), 2026-11-18. Type strings come from the
`register_subclass` calls in `src/lerobot/robots` and `src/lerobot/teleoperators`. Extras come
from `pyproject.toml`. Each robot has a page at `docs/source/<robot>.mdx`
(https://huggingface.co/docs/lerobot/<robot>). Read it for assembly and wiring before you touch
the hardware.

To list the registered types for the installed version, import the CLI module so every robot
module gets imported:
```bash
python -c "import lerobot.scripts.lerobot_record; from lerobot.robots import RobotConfig; from lerobot.teleoperators import TeleoperatorConfig; print(sorted(RobotConfig.get_known_choices())); print(sorted(TeleoperatorConfig.get_known_choices()))"
```

## Robot types at this commit

| `--robot.type` | Hardware | Extra | Notes |
|---|---|---|---|
| `so101_follower`, `so100_follower` | SO-ARM101 / SO-ARM100, 6× Feetech STS3215 | `feetech` | Same class (`SOFollower`, calibration dir `so_follower`). `use_degrees=true` default |
| `bi_so_follower` | Two SO followers | `feetech` | `--robot.left_arm_config.port`, `--robot.right_arm_config.port`. Sub-arm ids are `<id>_left` / `<id>_right`. Per-arm keys are prefixed `left_` / `right_` |
| `koch_follower` | Koch v1.1, Dynamixel | `dynamixel` | `use_degrees=false` default |
| `omx_follower` | ROBOTIS OpenManipulator-X, Dynamixel | `dynamixel` | `use_degrees=false` default |
| `lekiwi`, `lekiwi_client` | LeKiwi mobile base + SO arm | `lekiwi` | `lekiwi` runs on the Pi (host). `lekiwi_client` runs on the laptop (`remote_ip`) |
| `hope_jr_arm`, `hope_jr_hand` | HopeJR humanoid arm and hand | `hopejr` | Teleop with `homunculus_arm` / `homunculus_glove` |
| `openarm_follower`, `bi_openarm_follower` | OpenArm, Damiao motors on CAN | `damiao` (docs) / `openarms` | `--robot.port=can0`. Set up CAN first |
| `reachy2` | Pollen Reachy 2 | `reachy2` | Uses `reachy2_sdk`. Own camera type |
| `unitree_g1` | Unitree G1 humanoid | `unitree_g1` + manual `unitree_sdk2` | See `docs/source/unitree_g1.mdx` |
| `earthrover_mini_plus` | FrodoBots EarthRover | `hardware` + vendor SDK | Teleop `keyboard_rover` |
| `yam_follower`, `bi_yam_follower` | I2RT YAM arm | `yam` | Degrees and 0–100 gripper by default. `use_degrees=false` gives radians and 0–1 (MolmoAct2/I2RT data) |
| `rebot_b601_follower`, `bi_rebot_b601_follower` | Seeed reBot B601 (CAN) | `rebot` | Leader `rebot_102_leader` |
| `sim` | A gym env exposed as a robot | env extra | `lerobot-rollout --robot.type=sim --robot.env.type=libero ...` |

## Teleoperator types

`so101_leader`, `so100_leader`, `bi_so_leader`, `koch_leader`, `omx_leader`, `openarm_leader`,
`bi_openarm_leader`, `openarm_mini`, `bi_openarm_mini`, `rebot_102_leader`, `bi_rebot_102_leader`,
`homunculus_arm`, `homunculus_glove`, `reachy2_teleoperator`, `unitree_g1`, `keyboard`,
`keyboard_ee`, `keyboard_rover`, `gamepad` (`gamepad` extra), `phone` (`phone` extra).

Keyboard teleop needs a global key backend: X11, a Windows desktop, or macOS with Accessibility
and Input Monitoring granted to the terminal. It does not work on Wayland or over headless SSH.
The record control keys (→ ← ESC / n r q) do fall back to the focused terminal.

## SO-100 vs SO-101

| | SO-100 | SO-101 |
|---|---|---|
| Follower | 6× STS3215 1/345 | 6× STS3215 1/345 |
| Leader | STS3215 with the **gears removed** (encoder only) | Mixed ratios: pan 1/191, shoulder lift 1/345, elbow 1/191, wrist flex, wrist roll and gripper 1/147 |
| Types | `so100_follower` / `so100_leader` | `so101_follower` / `so101_leader` |
| Bring-up | identical commands (`lerobot-setup-motors`, `lerobot-calibrate`) | identical |

Both use the same Python class, so `so100_*` and `so101_*` share calibration directories
(`calibration/robots/so_follower/`, `calibration/teleoperators/so_leader/`). Give each physical
arm its own `id`.

SO-specific behaviour (verified in `so_follower.py`):
- Motors: `shoulder_pan`(1), `shoulder_lift`(2), `elbow_flex`(3), `wrist_flex`(4),
  `wrist_roll`(5), `gripper`(6). Keys are `<motor>.pos`.
- Body joints are in degrees (`use_degrees=true`) or −100..100. The gripper is 0..100.
- At connect: position mode, P/I/D = 16/0/32. **Gripper only:** `Max_Torque_Limit=500`,
  `Protection_Current=250`, `Overload_Torque=25`, which keeps the gripper motor from burning out.
- `wrist_roll` is calibrated as a full-turn motor (the whole encoder range, no sweep).
- `max_relative_target` clips every command relative to the present position. It reads the bus
  every step, which costs some loop rate.

**Advice (not in the repo):** STS3215 comes in voltage variants. Use the power supply your kit's
BOM specifies (see https://github.com/TheRobotStudio/SO-ARM100). A weak or wrong supply shows
up as sagging under load, `Incorrect status packet` errors, or resets when several joints move
together.

### Calibration files and migration
- Path: `$HF_LEROBOT_CALIBRATION/<robots|teleoperators>/<class name>/<id>.json`. The default
  root is `~/.cache/huggingface/lerobot/calibration`.
- Each motor stores `id`, `drive_mode`, `homing_offset`, `range_min` and `range_max`.
  `range_min == range_max` is invalid and raises when values are normalised.
- Datasets recorded before PR #777 (old calibration: arm extended = zero, ±180° wrap logic)
  need conversion. `examples/backward_compatibility/replay.py` shows the per-joint fix
  (`shoulder_lift.pos = -(x - 90)`, `elbow_flex.pos -= 90`, `main_` prefix dropped, `.pos` suffix
  added).

## Bimanual SO example
```bash
lerobot-teleoperate \
  --robot.type=bi_so_follower --robot.id=bimanual \
  --robot.left_arm_config.port=/dev/ttyACM0 --robot.right_arm_config.port=/dev/ttyACM1 \
  --teleop.type=bi_so_leader --teleop.id=bimanual_leader \
  --teleop.left_arm_config.port=/dev/ttyACM2 --teleop.right_arm_config.port=/dev/ttyACM3
```
Check the exact field names with `lerobot-teleoperate --robot.type=bi_so_follower --help`.
Cameras can go at the top level (`--robot.cameras`) or per arm (prefixed keys).

## LeKiwi (host / client)
```bash
# On the Raspberry Pi (wireless version):
python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_kiwi --host.connection_time_s=600
# On the laptop: edit remote_ip/port in examples/lekiwi/teleoperate.py or record.py, then
python examples/lekiwi/teleoperate.py
```
The host exits after `--host.connection_time_s` (30 s by default), so raise it for recording.
ZMQ ports are 5555 (cmd) and 5556 (obs). The host has a 500 ms watchdog. Base keys: w/a/s/d
translate, z/x rotate, r/f change speed. The wired version runs everything on the laptop.

## OpenArm (CAN)
```bash
lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=test  --interfaces=can0,can1
lerobot-teleoperate --robot.type=openarm_follower --robot.port=can0 ... --teleop.type=openarm_leader ...
```

## Integrating a new robot
`docs/source/integrate_hardware.mdx` describes the contract: a `RobotConfig` subclass
registered with `@RobotConfig.register_subclass("name")`, plus a `Robot` subclass implementing
`connect`, `calibrate`, `configure`, `get_observation`, `send_action`, `disconnect`,
`observation_features` and `action_features`. Third-party packages named `lerobot_robot_<name>`
/ `lerobot_teleoperator_<name>` are discovered as plugins (`register_third_party_plugins`).
Read that page for your version before writing code. To support RTC or remote waiting, a robot
must declare position-hold support (`supports_position_hold`).
