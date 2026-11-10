# UniX AI Panther driver (read-only)

Phanthy Motus driver bundle for the UniX AI **Panther** dual-arm wheeled robot (ROS 2 Humble,
robot name `mh3`). This first batch is **monitoring only**: it subscribes to the robot and
shows its state on the canvas. It publishes nothing to the robot, calls none of its services
and switches no controller, so no card here can move the robot.

## Cards

| Card | Format | Rate | Source on the robot |
|---|---|---|---|
| `model` | resource | - | `resource/panther.urdf` (see `resource/SOURCE.md`) |
| `joints` | `sensor/skeleton` | 10 Hz | `/joint_states` (18 joints: 16 arm + 2 head) |
| `battery` | `data/json` | 1 Hz | `/battery_state_broadcaster/battery_state` |
| `imu` | `data/json` | 10 Hz | `/imu_sensor_broadcaster/imu` |
| `odom` | `state/odom` (`motus.odom/1`) | 10 Hz | `/odom` (mean of the window; `vx`, `vy`, `wz` only; no pose) |
| `gripper_lift_state` | `data/json` | 10 Hz | `/left_gripper/joint_states`, `/right_gripper/joint_states`, `/lifting_controller/joint_states` |
| `health` | `data/json` | 2 Hz | `/diagnostics` (CAN, serial, wheel/steering motors, arm motors, e-stop) |
| `estop` | `data/json` | 1 Hz | `/diagnostics`, the emergency-stop entry, verbatim, with its age |
| `laser_scan_front` / `laser_scan_rear` | `data/json` + `image/jpeg` | 5 Hz / 1 Hz | `/lidar_front/scan`, `/lidar_rear/scan` |
| `camera_color` | `image/jpeg` | 10 Hz | `/camera/color/image_raw` |
| `camera_depth` | `image/depth-zlib` | 5 Hz | `/camera/depth/image_raw` (resampled to 640x480 uint16 mm) |

Topic names are in `config.yaml` and were taken from `ros2 topic list -t` on a Panther. `/imu`
and `/scan` exist there but have no publisher; the broadcaster topics above are the live ones.

### Things a user of these cards needs to know

- **`estop` is not a safety device.** It repeats the vendor's diagnostic text. The only value
  seen on a robot so far is the "normal" one, so no other string is interpreted. When
  `/diagnostics` stops, the card keeps the last text but reports `stale: true`; a stale reading
  must never be read as "not pressed".
- **The cameras need the robot's Orbbec driver running**, which the robot's own bring-up does not
  start (`ros2 launch orbbec_camera gemini2.launch.py`). Without it the camera cards still start,
  and say in `info` that no frames have arrived. Camera optics are declared `unknown`.
- **Camera cards and the lidar top-down view do no work until the card is started.** The lidar
  distance JSON, like every other card, publishes continuously.
- **The lift is not drawn in the skeleton.** The vendor URDF declares `body_joint` as `fixed`
  (and limits it to 0.6 m, while the robot reports 0.61), so `gripper_lift_state` carries the lift
  as a number and `joints` does not move it. Gripper values are passed through unconverted: which
  end of the range is "open", and the lift's unit, are the vendor's to confirm.
- **`joints` leaves out anything not in the URDF** and any joint whose position is not a finite
  number, rather than sending `null`.
- Neither the vendor SDK document nor the URDF agree on the head axes (the SDK says
  `joint_head0` is pan, the URDF makes it pitch). The skeleton follows the URDF; confirm before
  anyone writes a head control card.

## How it connects

```
robot, domain 0, CycloneDDS, bound to 127.0.0.1      this container (network_mode: host)
  /joint_states, /odom, /diagnostics, ...   ------>  main process: rclpy subscriptions (read-only)
                                                      cards: throttle, clean, encode
                                                          |  bounded queues, newest sample wins
                                                          v
                                                     child processes: FastDDS, domain 42,
                                                     /opt/phanthy-motus/dds-local.xml
                                                          |
                                                          v
                                                     Agent Core
```

The robot's own `~/config/cyclonedds.xml` pins CycloneDDS to loopback. `config.yaml`
(`ros.cyclonedds_uri`) repeats that configuration for this container. **If it is wrong the
driver looks healthy**: the topic list is visible and no sample ever arrives. The driver says so
in its log after 30 s and in every card's `info` (`robot_link.state: never_received`).

`panther_bridge.py` is the same arrangement `deep_robotics/s10_pro` uses: FastDDS reads its
profile once per process and the RMW is chosen at `rclpy.init`, so one process cannot be on
both buses.

## Files

| File | Role |
|---|---|
| `main.py`, `device.py` | entry point; `Hub` (all logic, no ROS), `PantherNodes` (rclpy), `PantherPlugin` (MCP) |
| `panther_payloads.py` | message -> JSON-safe payloads, image encoders (NaN -> `null`, `byte` levels, nearest-return lidar pooling) |
| `panther_bridge.py` | domain-42 publisher child processes |
| `config.yaml`, `driver.yaml`, `deploy/service.yml` | settings, marketplace listing, compose fragment |
| `resource/panther.urdf`, `tools/strip_urdf.py` | skeleton model and how it was derived |

Tests: `python3 -m pytest tests/test_unixai_panther.py` (no ROS or robot needed).

## Acceptance on the robot

Done by a person at the robot. Nothing in this driver moves it; steps 5 and 7 need the operator to
move the robot with the vendor's own tools.

1. **Deploy** the image the review bot prints. On this robot the script is
   `/opt/phanthy-motus/run-pr-image.sh`. Expect the container `embodied-unixai-panther` to be
   `Started`.
2. **Log**: `docker logs --tail 80 embodied-unixai-panther` shows `subscribed (read-only) to:`,
   then `[dds-bridge] lane=state child pid=` and `first sample published` lines. A line containing
   `no data from the robot's DDS` means the Cyclone configuration is not matching.
3. **Isolation**: `sudo ss -lunp | grep -i python` - the container's UDP sockets bind `127.0.0.1`
   only, never `192.168.100.2` or the Wi-Fi address.
4. **The robot's graph is unchanged**: `ros2 topic list | wc -l` and `ros2 node list` before and
   after deployment differ only by the `/panther_driver` node.
5. **Canvas**: drag out each card and open its data view. Check `joints` follows the arms when the
   robot is moved by its normal operator tooling (the VR teleoperation app or the vendor's own
   tools, run by the person who normally does so - this driver never commands anything), `battery`
   shows a percentage (not `1`), `health` lists five groups and `estop` says `正常` with a small
   `age_ms`.
6. **Cameras**: before starting the Orbbec driver, `camera_color` must start and report that no
   frames arrived. Start the driver on the robot, restart the card, and check the picture. Record
   the encoding and size shown in the card's `info` (`image`), which this document does not know.
7. **Odometry**: at rest `vx`, `vy`, `wz` are 0.0 (not `null`). While the operator drives the base
   slowly with the vendor's own teleoperation (clear floor, emergency stop in reach), they must
   change. A card that stays 0.0 while the base moves is wrong. Do not push the 100 kg base by hand.
8. **Rear lidar**: expect `laser_scan_rear` to report no data unless the robot publishes
   `/lidar_rear/scan`. Record which.
9. **Not part of acceptance**: pressing the emergency stop cuts motor power and lets the arms drop.
   Only do it with the arms clear and the supervisor's agreement, and record `estop`'s text while
   pressed; it is the one value this driver has never seen.

Stop at any time with `sudo bash /opt/phanthy-motus/run-pr-image.sh --down`. The container only
listens, so stopping it changes nothing on the robot.

Open questions for the vendor (UniX AI): the gripper range and the lift unit; the head axis
naming; what `/diagnostics` shows while the emergency stop is pressed; whether `/lidar_rear/scan`
has a publisher; the Gemini 2 camera's profile and intrinsics.
