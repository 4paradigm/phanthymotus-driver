# UniX AI Panther driver

Phanthy Motus driver bundle for the UniX AI **Panther** dual-arm wheeled robot (ROS 2 Humble,
robot name `mh3`): sensor cards that show the robot's state, and five motion cards that can move
it. **Motion is off by default** (`safety.commissioning_motion_enabled: false` in `config.yaml`):
the motion cards are listed and answer `info`, and every command is refused until an operator
turns the switch on at the robot.

The sensor cards were written first and tried on a robot's topic list; the motion cards and the
lazy camera start come from the Panther driver proposed in PR #520 and were extended where that
review found gaps (a `stop` for every actuator, completion reporting for the long ones).

## Cards

### Sensors and resource

| Card | Format | Rate | Source on the robot |
|---|---|---|---|
| `model` | resource | - | `resource/panther.urdf` (see `resource/SOURCE.md`) |
| `joints` | `sensor/skeleton` | 10 Hz | `/joint_states` (18 joints: 16 arm + 2 head) |
| `imu` | `data/json` | 10 Hz | `/imu_sensor_broadcaster/imu` |
| `odom` | `state/odom` (`motus.odom/1`) | 10 Hz | `/odom` (mean of the window; `vx`, `vy`, `wz` only; no pose) |
| `robot_state` | `data/json` | 2 Hz, and at once on a diagnostics change | battery, `/diagnostics`, grippers, lift, `list_controllers` (below) |
| `laser_scan` | `data/json` + `image/jpeg` | 5 Hz / 1 Hz | `/lidar_front/scan` + `/lidar_rear/scan`, merged round `base_link` |
| `camera_color` | `image/jpeg` | 10 Hz | `/camera/color/image_raw` |
| `camera_depth` | `image/depth-zlib` | 5 Hz | `/camera/depth/image_raw` (resampled to 640x480 uint16 mm) |

`robot_state` is one low-rate card in blocks, each with its own `age_ms` and `fresh`:
`battery`, `diagnostics` (sorted by name: the robot's order changes per message), `estop`,
`grippers.left/right`, `lift`, `controllers` (polled once a second from
`/controller_manager/list_controllers`, read-only), plus `ready_for_motion` and why not.

`laser_scan` puts both lidars in one scan of 360 bins round `base_link`, nearest return per bin.
The vendor URDF has no frames for the individual lidars, so their mounting poses are read from the
robot's latched `/tf_static`. A lidar that is stale, has never reported, or has no known transform
is left out and the reason is in `sources`; if *no* lidar has a transform yet, the first one is
shown in its own frame and `transformed` is `false`. Ranges are measured from `base_link`'s origin,
not from either lidar. The card does not use `/scan` (the vendor's merged scan): it has no
publisher on this robot, and PR #520 reports otherwise, so confirm with `ros2 topic hz /scan`.

### Actuator cards

| Card | Actions | Does | Completion |
|---|---|---|---|
| `base_drive` | `start` `stop` `info` `move` | `/cmd_vel_teleop` at 20 Hz for `duration_s` (max 3 s, 0.2 m/s, 0.3 rad/s), then zero velocity five times | `action_id` now, completion posted when it ends |
| `arm_control` | `start` `stop` `info` `move` | point-to-point `FollowJointTrajectory`, one or both arms | `action_id` now, completion posted when it ends |
| `gripper` | `start` `stop` `info` `set` `check` | `set` sends one `unix_msgs/GripperCommand` with `position` 0..1. `check` reads actual feedback. | immediate snapshot for `check` |
| `lifting` | `start` `stop` `info` `set_position` | one `JointState` on `/lifting_controller/commands` | none |
| `head` | `start` `stop` `info` `look` | one `Float64MultiArray` `[head0, head1]` | none |

`gripper` accepts `{"action":"check","side":"left"}` (or `"right"`) even while motion
is disabled. It reads the latest `/<side>_gripper/joint_states` sample and returns
`position`, `fresh`, `age_ms`, and `opening`. Set `plugins.gripper.feedback_endpoints.<side>.closed`
and `.open` to that side's measured closed and open positions in feedback units. The 5% of
travel around each endpoint counts as `closed` or `open`, with `partial` in between.
`opening` is `unknown` with a reason when feedback is missing, stale, invalid, uncalibrated,
or outside the calibrated range and tolerance. `check` reports the sampled position at the
time of the call.

Every motion command needs all of these, checked in `panther_motion.py` before anything is sent:

- `safety.commissioning_motion_enabled: true` in `config.yaml`;
- `confirm_motion: true` in the call;
- fresh `/joint_states` (under 1 s) and a fresh, **normal** emergency-stop diagnostic (the
  diagnostic's standard `level` is 0 and it is under 3 s old; the vendor's text is never
  interpreted, and unrelated errors such as an absent camera do not block motion);
- the matching ros2_control controller `active` (checked live, not from the 1 Hz cache);
- values finite and inside the caps in `config.yaml`: arm and head targets inside the URDF limits
  (live `/robot_description` if received, otherwise the bundled copy) and at most 0.35 rad from
  where the joint is now; the lift at most 0.10 from its reading and below 0.65.

What `stop` does, honestly:

- `base_drive`: bumps a generation counter so the worker stops publishing, and publishes zero
  velocity five times.
- `arm_control`: cancels the trajectory goal and switches the controllers back, then returns. The
  call reports `stopping` instead of `idle` if that takes over 10 s.
- `gripper`, `lifting`, `head` are one position command each. Once it is published the robot's
  controller owns the motion, and this driver has no halt to call (the vendor's halt/disable
  services are deliberately not used). `stop` there promises only that **nothing more is sent**.

`arm_control` switches controllers, and only as far as the move needs: it takes a snapshot of what
is active, activates the trajectory controller of **each arm being moved** (and only if it is not
already active), and releases `dual_arm_forward_position_controller` if it was active (it owns the
joints of both arms, so it has to give way; the other arm's forward control pauses for the
duration). Afterwards it applies the exact inverse, so every controller ends as it began: a
trajectory controller someone else already had active stays active, and the other arm's trajectory
controller is never touched. If nothing needs changing no switch request is sent. If putting it
back fails, the completion status is `error`, it says `controllers_restored: false`, and the log
says to check `ros2 control list_controllers`. A SIGTERM mid-move cancels and restores first
(`stop_grace_period: 30s`).

`start` on a motion card always answers `state: "ready"` (the card is up), whatever the
commissioning switch or the robot's readiness; those are reported next to it as
`commissioning_motion_enabled`, `ready_for_motion`, `not_ready_reasons` and `accepts_motion`.
`stop` makes the card `idle` until the next `start` or command.

### Things a user of these cards needs to know

- **`estop` is not a safety device.** It repeats the vendor's diagnostic text. The only value
  seen on a robot so far is the "normal" one, so no other string is interpreted. When
  `/diagnostics` stops, the block keeps the last text but reports `stale: true`; a stale reading
  must never be read as "not pressed". The motion gate treats stale and non-zero levels as "not
  ready", which is conservative: it can refuse a move it need not have.
- **Starting a camera card starts the robot's Orbbec driver** if nothing else is publishing the
  camera topics: a transient host unit `panther_orbbec.service` running the vendor's
  `gemini2.launch.py` (`host_camera.py`). Both camera cards share it and stopping a card never
  stops it, because other programs on the robot may use the camera. A watchdog restarts it at most
  twice if frames stall. If the topics already have a publisher, the driver leaves it alone. This
  is why the container is `privileged` with `pid: host`; nothing else uses either. If the camera
  cannot be started, the card still starts and says why in `camera_runtime.start_error`.
- **Camera cards and the lidar top-down view do no work until the card is started.** The lidar
  distance JSON, like every other sensor card, publishes continuously.
- **The lift is not drawn in the skeleton.** The vendor URDF declares `body_joint` as `fixed`
  (and limits it to 0.6 m, while the robot reports 0.61), so `robot_state` carries the lift as a
  number and `joints` does not move it. Gripper and lift values are unconverted: which end of the
  gripper range is "open", and the lift's unit, are the vendor's to confirm. That is why the lift
  command is step-limited from the live reading rather than trusted as an absolute position.
- **`joints` leaves out anything not in the URDF** and any joint whose position is not a finite
  number, rather than sending `null`.
- **The head axes disagree between the vendor SDK document and the URDF** (the SDK says
  `joint_head0` is pan, the URDF makes it pitch about y, and `joint_head1` yaw about z). `head`
  therefore takes `head0_rad` and `head1_rad` by joint name rather than pan/tilt. It publishes
  `[head0, head1]`, assuming that is `head_forward_position_controller`'s joint order; the step
  limit from the live joint state would show a swapped order as a refused or small move, but
  confirm it with `ros2 param get /head_forward_position_controller joints` before enabling motion.

## How it connects

```
robot, domain 0, CycloneDDS, bound to 127.0.0.1      this container (network_mode: host)
  /joint_states, /odom, /diagnostics, ...   ------>  main process: rclpy subscriptions
  /cmd_vel_teleop, controllers, actions     <------  Motion (only if enabled and confirmed)
                                                      cards: throttle, clean, encode
                                                           |  bounded queues, newest sample wins
                                                           v
                                                      child processes: FastDDS, domain 42,
                                                      /opt/phanthy-motus/dds-local.xml
                                                           |
                                                           v
                                                      Agent Core
```

Robot command topics never cross to Agent Core's domain; only finished card payloads do.

The robot's own `~/config/cyclonedds.xml` pins CycloneDDS to loopback. `config.yaml`
(`ros.cyclonedds_uri`) repeats that configuration for this container. **If it is wrong the
driver looks healthy**: the topic list is visible and no sample ever arrives. The driver says so
in its log after 30 s and in every sensor card's `info` (`robot_link.state: never_received`).

`panther_bridge.py` is the same arrangement `deep_robotics/s10_pro` uses: FastDDS reads its
profile once per process and the RMW is chosen at `rclpy.init`, so one process cannot be on
both buses.

## Files

| File | Role |
|---|---|
| `main.py`, `device.py` | entry point; `Hub` (sensor logic, no ROS), `RosMotionIO` and `PantherNodes` (rclpy), `PantherPlugin` (MCP) |
| `panther_motion.py`, `contracts.py` | `Motion` (the gate, stop, completion; no ROS) and the validation rules it uses |
| `host_camera.py` | lazy start and watchdog of the Orbbec driver on the robot (no ROS) |
| `panther_payloads.py` | message -> JSON-safe payloads, image encoders, `/tf_static` and lidar merging |
| `panther_bridge.py` | domain-42 publisher child processes |
| `config.yaml`, `driver.yaml`, `deploy/service.yml` | settings, marketplace listing, compose fragment |
| `vendor/unix_msgs/` | the one message type the gripper topics use (built in the Dockerfile) |
| `resource/panther.urdf`, `tools/strip_urdf.py` | skeleton model and how it was derived |

Image size: the runtime image adds the ROS interface packages for control (`control_msgs`,
`controller_manager_msgs`, `trajectory_msgs`, `geometry_msgs`, `tf2_msgs`) to what the base image
lacks; `unix_msgs` is compiled in a throwaway build stage and only its `install/` is copied. No
`systemd` is installed: `host_camera.py` uses the host's through `nsenter`.

Tests: `python3 -m pytest tests/test_unixai_panther.py` (no ROS or robot needed).

## Acceptance on the robot

Done by a person at the robot, with the Master present. Do **sensors first, with motion off**;
turn motion on only for step 11, and only after everything before it is right.

1. **Deploy** the image the review bot prints. On this robot the script is
   `/opt/phanthy-motus/run-pr-image.sh`. Expect the container `embodied-unixai-panther` to be
   `Started`.
2. **Log**: `docker logs --tail 80 embodied-unixai-panther` shows `subscribed to:` and
   `motion cards enabled (commands are still refused ...)`, then `[dds-bridge] lane=state child
   pid=` and `first sample published` lines. A line containing `no data from the robot's DDS`
   means the Cyclone configuration is not matching.
3. **Isolation**: `sudo ss -lunp | grep -i python` - the container's UDP sockets bind `127.0.0.1`
   only, never `192.168.100.2` or the Wi-Fi address.
4. **The robot's graph**: before and after deployment `ros2 node list` differs only by
   `/panther_driver`. With motion cards enabled that node also has the command publishers and
   controller clients; nothing is published on them while the gate is closed
   (`ros2 topic echo /cmd_vel_teleop` stays silent).
5. **Canvas, sensors**: drag out `joints`, `imu`, `odom`, `robot_state`, `laser_scan`. `joints`
   follows the arms when the operator moves the robot with the vendor's tools (this driver
   commands nothing); `robot_state` shows a battery percentage (not `1`), `estop` `正常` with a
   small `age_ms`, grippers, lift, and `controllers` filled; `ready_for_motion` is true.
6. **Lidar**: `ros2 topic echo /tf_static --once | grep -B2 -A8 -i radar` names the two lidar
   frames; `laser_scan` should say `transformed: true` and use both in `sources`. Record whether
   `/lidar_rear/scan` publishes and whether `/scan` has a publisher (`ros2 topic hz /scan`).
7. **Cameras**: start `camera_color`. `docker logs` and `camera_runtime` show whether it started
   `panther_orbbec.service` (`systemctl status panther_orbbec`) or found a publisher already.
   Check the picture, then `camera_depth`. Stop both cards: `systemctl is-active
   panther_orbbec` must still be `active`. Record the encoding and size in each card's `info`.
8. **Odometry**: at rest `vx`, `vy`, `wz` are 0.0 (not `null`). While the operator drives the base
   slowly with the vendor's own teleoperation (clear floor, emergency stop in reach), they must
   change. Do not push the 100 kg base by hand.
9. **Motion refused while off**: with the switch still `false`, call `base_drive` `move` with
   `confirm_motion: true`; it must come back `error` naming `commissioning_motion_enabled`, and
   `ros2 topic echo /cmd_vel_teleop` must show nothing.
10. **Confirm the facts the head and lift rely on**:
    `ros2 param get /head_forward_position_controller joints`; the lift's unit and range with the
    vendor; `ros2 control list_controllers` against the names in `panther_motion.py`.
11. **Motion**, only with the operator beside the robot, the area clear and the emergency stop in
    reach: set `commissioning_motion_enabled: true`, restart, then try in this order and stop at
    the first surprise: `base_drive` 0.05 m/s for 1 s; `base_drive` `stop` during a 3 s move;
    `head` 0.1 rad; `lifting` 0.02 from its reading; `gripper` small step; `arm_control` 0.05 rad
    on one joint. After each arm move check `ros2 control list_controllers` shows the original
    controller active again, and that Agent Core received the completion.
12. **Not part of acceptance**: pressing the emergency stop cuts motor power and lets the arms
    drop. Only do it with the arms clear and the supervisor's agreement, and record `estop`'s text
    while pressed; it is the one value this driver has never seen.

Stop at any time with `sudo bash /opt/phanthy-motus/run-pr-image.sh --down`. That sends the stop
described above first. It does not stop `panther_orbbec.service`; do that with
`sudo systemctl stop panther_orbbec` if you started it only for this test.

Open questions for the vendor (UniX AI): the gripper range and the lift unit; the head axis
naming and the head controller's joint order; what `/diagnostics` shows while the emergency stop is
pressed; whether `/lidar_rear/scan` and `/scan` have a publisher; the lidars' frame names; the
Gemini 2 camera's profile and intrinsics.
