# UniX AI Panther driver

Phanthy Motus driver bundle for the UniX AI **Panther** dual-arm wheeled robot (ROS 2 Humble,
robot name `mh3`): sensor cards that show the robot's state, and five motion cards that move it.
**There is no confirmation field and no motion switch**: with intelligent control on, a call to a
motion card moves the robot (the operator asked for this, 2026-11-11). What still stands between a
call and the robot is listed under "What every command needs".

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
| `laser_scan` | `data/json` + `image/jpeg` | 5 Hz / 1 Hz | `/lidar_front/scan` + `/lidar_rear/scan` (silent here: no rear lidar), round `base_link` |
| `camera_color` | `image/jpeg` | 10 Hz | `/camera/color/image_raw` |
| `camera_depth` | `image/depth-zlib` | 5 Hz | `/camera/depth/image_raw` (resampled to 640x480 uint16 mm) |

`robot_state` is one low-rate card in blocks. Each block except `diagnostics` shows how old it is
(`age_ms`, `fresh`); the card is a reading view, trimmed of what is constant, always empty or
duplicated on this robot (the motion gate reads the full data, not this view):

| Block | Shows |
|---|---|
| `battery` | `percent`, `voltage_v`, `current_a`, `temperature_c`, `status` |
| `diagnostics` | `count`, `worst_level`, `worst_level_name`, `problems`, `age_ms`, `fresh` |
| `estop` | `found`, `level`, `message`, `stop_button_status`, `age_ms`, `stale` |
| `grippers.left/right`, `lift` | `position`, `effort` |
| `controllers` | every controller's state, polled once a second from `/controller_manager/list_controllers` (read-only) |

plus `ready_for_motion` and why not. `diagnostics.problems` lists, sorted by name, every entry
whose level is not 0 as `"name: message"`, and every entry that reads ok but has not been updated
for 5 s (`"name: 超过 5 秒没有更新"`): entries are kept by name, so a publisher that died would
otherwise leave its last "ok" on the card for ever. On the test robot `worst_level` is `error`
because of `lifecycle_manager_navigation: Nav2 Health: Nav2 is inactive`: the robot has the Nav2
nodes but the navigation lifecycle is not active. That is the robot's own report; it has nothing
to do with the sensors or the motion gate, and the card stays `error` for as long as Nav2 is off.
`diagnostics` is `null` until the first message arrives.

`laser_scan` is one scan of 360 bins round `base_link`, nearest return per bin. The driver can
merge a front and a rear lidar, but **this robot has one lidar, at the front**: the rear half of
the scan is blocked by the central column, so the rear topic never carries data and that is
normal (see below). The vendor URDF has no frame for the lidar, so its mounting pose is read from
the robot's latched `/tf_static`. Measured on the robot: `link_front_radar` is at (0.226, 0, 0.266)
from `base_link` with no rotation (the names `lidar_front_link` / `lidar_rear_link` in the vendor
SDK document are not the robot's). A lidar that is stale, has never reported, or has no known
transform is left out and the reason is in `sources`; if *no* lidar has a transform yet, the first
one is shown in its own frame and `transformed` is `false`. Ranges are measured from
`base_link`'s origin, not from the lidar. The card does not use `/scan` (the vendor's merged
scan): it needs both lidars, so it is not published on a robot with one.

Returns closer than `laser.self_filter_radius_m` (0.35 m) to `base_link` are the robot's own body
and are dropped; how many is in `self_filtered_beams`, and `nearest_m` is measured after that.
Why 0.35: the unfiltered `nearest_m` was 0.073, the lidar looking back at the central column,
which a photo of the robot shows standing near the middle of the base. A lidar 0.226 m ahead of
`base_link` seeing it 0.30 m away puts it 0.074 m from `base_link`'s origin, which matches. The
filter is a circle, so it also hides a real obstacle nearer than 0.35 m at the corners and the
sides (the base is not round), and only a lidar with a known transform is filtered. The radius is
an estimate from one photo and one number: **tune it against the top-down view**, and set it to 0
to turn the filter off.

**Why there is no rear lidar data.** The vendor software starts a `bluesea_node` for a rear lidar
(`rear_lidar.yaml`, port `/dev/ttyUSB_RAIDAR_REAR`) on every robot, but this one has no rear lidar, so
that port does not exist, the process never opens a serial device and the topic stays silent. Both
nodes are called `bluesea_node`, which makes ROS warn about duplicate node names. An earlier
version of this README blamed a missing udev rule for the rear lidar; that was wrong, from reading
the vendor's config on a robot that does not have the hardware. **Do not create the missing device
link** (for example by pointing it at `ttyACM1`): that is not a lidar, and the rear node would send
commands to whatever it is.

### Motion (all `actuator`)

| Card | Choices (`action`) | Boxes | Does | Completion |
|---|---|---|---|---|
| `base_drive` | `move` (default), `stop` | `linear_x`, `linear_y`, `angular_z`, `duration_s` | `/cmd_vel_teleop` at 20 Hz for `duration_s`, then zero velocity five times | `action_id` now, completion posted when it ends |
| `arm_control` | `move` (default), `home`, `stop` | `la0`-`la7`, `ra0`-`ra7`, `duration_s`; `side` for `home` | `FollowJointTrajectory`, one or both arms | `action_id` now, completion posted when it ends |
| `gripper` | `set` (default), `open`, `close` | `side` (`both` default, `left`, `right`), `mode`, `position` | one `unix_msgs/GripperCommand` per hand | none: no feedback defines it |
| `lifting` | **no `action` box**: its one motion | `position` | one `JointState` on `/lifting_controller/commands` | none |
| `head` | `look` (default), `home`, `stop` | `head0_rad`, `head1_rad` | `look`: one `Float64MultiArray` `[head0, head1]`; `home`: small steps to 0 | `home`: `action_id`, completion |

`start`, `stop` and `info` are always answered (the canvas lifecycle); they are not offered as
menu items except where `stop` has something to stop. A call that names no `action` runs the card's
default (`lifting` is always called that way).

**Every number box shows the range it accepts**, in its title (`左臂 la0（rad，-1.046 ~ 0.697）`)
and in its description, and the same numbers are the schema's `minimum`/`maximum` and what the
driver enforces: the tests try every box on its shown boundary (accepted) and just past it
(refused). A refusal quotes the range, and for a step limit, what would be allowed from where the
joint is now (`allowed now: -0.35 ~ 0.35`). The ranges:

| Box | Range | Plus |
|---|---|---|
| `linear_x`, `linear_y` | -0.2 ~ 0.2 m/s | empty = 0 |
| `angular_z` | -0.3 ~ 0.3 rad/s | empty = 0 |
| `duration_s` (base) | 0.1 ~ 3 s | empty = 1 s |
| `la0`-`la7`, `ra0`-`ra7` | each joint's URDF limit (e.g. `la0` -1.046 ~ 0.697, `ra7` -1.57 ~ 1.57) | at most 0.35 rad from where the joint is now; **empty = that joint stays where it is** |
| `duration_s` (arm) | 1 ~ 20 s | empty = 6 s |
| `position` (gripper) | 0 ~ 1 | **0 = open, 1 = closed** (operator-confirmed; the vendor did not say) |
| `position` (lift) | 0 ~ 0.65 (unit not confirmed by the vendor; reads about 0.61) | at most 0.10 from the current reading |
| `head0_rad` | -0.348 ~ 1.5 rad (pitch, about y) | at most 0.35 rad from now; empty = stays |
| `head1_rad` | -1.57 ~ 1.57 rad (turn, about z) | at most 0.35 rad from now; empty = stays |

The arm and head ranges shown come from the driver's bundled URDF; commands are checked against
the robot's live `/robot_description` once it arrives. If the two differ, the driver logs a
`WARNING: the robot's joint limits differ ...` at start-up naming the joints, and the boxes would
then show slightly wrong ranges: the robot's limits still win.

**Presets**:

- `arm_control` `home`: every joint of the chosen arms (`side`, default both) to 0 rad, the pose
  the operator confirmed is safe on this robot (2026-11-11). `config.yaml`'s `home_pose` can change
  any joint's value. It is **not** held to the 0.35 rad step; instead its duration is stretched so
  the furthest joint moves at no more than 0.35 rad/s on average (at least 2 s, refused if over 20 s).
  The trajectory controller interpolates, so the peak speed is somewhat higher than the average.
  Nothing checks for self-collision: the URDF limits are the only bound.
- `head` `home`: both joints to 0 rad in steps of at most 0.35/20 rad every 1/20 s (the head's
  forward controller would otherwise jump at its own top speed). `stop` halts it where it is, and
  a `look` supersedes it.
- `gripper` `open` / `close`: `position` 0 / 1; a leftover number in the `position` box is ignored.

### What every command needs

Checked in `panther_motion.py` before anything is sent; a refusal says which one failed:

- fresh `/joint_states` (under 1 s) and a fresh, **normal** emergency-stop diagnostic (the
  diagnostic's standard `level` is 0 and it is under 3 s old; the vendor's text is never
  interpreted, and unrelated errors such as Nav2 being inactive do not block motion);
- the matching ros2_control controller `active` (checked live, not from the 1 Hz cache); for
  `gripper` `both`, both hands' controllers are checked before either hand is sent anything;
- every value inside the range its box shows (above).

These are conditions, not switches: they clear by themselves when the robot is fine. **Without a
confirmation step, a value that is inside its range but simply wrong is executed.** The step and
speed limits bound how far one wrong command can move the robot; nothing else does.

**Interrupts**: `base_drive`, `arm_control` and `head` declare `x-hooks` `on_interrupt_motion` and
`on_interrupt_all` -> `stop`, so Agent Core's emergency and full interrupts stop the base, the arms
and a head `home` directly. `gripper` and `lifting` have nothing in flight to stop.

What `stop` does, honestly:

- `base_drive`: bumps a generation counter so the worker stops publishing, and publishes zero
  velocity five times.
- `arm_control`: cancels the trajectory goal and switches the controllers back, then returns. The
  call reports `stopping` instead of `idle` if that takes over 10 s. A stop is honoured at every
  stage of a move, not only while it is running: the worker looks for it before switching
  controllers and before each trajectory goal, so a stop that arrives before the worker thread has
  even started means nothing is switched or sent; and a stop that lands while the move is still
  being checked makes `move` answer `error` ("stopped while this move was being checked", nothing
  sent) instead of being answered `idle` and then followed by the whole move. The price is that
  such a `move` has to be sent again. A stop with nothing running changes nothing and does not
  affect the next move. This is tested on real threads as well as step by step: after `stop`
  returns `idle`, nothing new is issued and the controllers are as they were.
- `head`: a `home` in progress stops where it is (no further step is sent). A `look` is one
  position command, which the robot's controller finishes.
- `gripper`, `lifting` are one position command each. Once it is published the robot's controller
  owns the motion, and this driver has no halt to call (the vendor's halt/disable services are
  deliberately not used). `stop` there promises only that **nothing more is sent**.

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

`start` on a motion card always answers `state: "ready"` (the card is up), whatever the robot's
readiness; that is reported next to it as `ready_for_motion`, `not_ready_reasons` and
`accepts_motion`.
`stop` makes the card `idle` until the next `start` or command.

### Things a user of these cards needs to know

- **`estop` is not a safety device.** It repeats the vendor's diagnostic text. The only value
  seen on a robot so far is the "normal" one, so no other string is interpreted. When
  `/diagnostics` stops, the block keeps the last text but reports `stale: true`; a stale reading
  must never be read as "not pressed". The motion gate treats stale and non-zero levels as "not
  ready", which is conservative: it can refuse a move it need not have.
- **Starting a camera card starts the robot's Orbbec driver** if nothing else is publishing the
  camera topics: a transient host unit `panther_orbbec.service` running the vendor's
  `gemini2.launch.py` (`host_camera.py`). Both camera cards share it. Stopping a *card* never stops
  it, because other programs on the robot may use the camera; **stopping the container does**
  (once, `systemctl stop --no-block`, so it does not eat the stop grace period), but only a unit
  this driver started or found under its own name. A camera something else was already publishing
  is never touched, and a container that is killed rather than stopped leaves the unit running
  for the next start to reuse. A watchdog restarts it at most twice if frames stall, and cannot
  restart it once the container has begun to stop. If the topics already have a publisher, the
  driver leaves it alone.
  
  **What the container is granted for this, and what is not yet proven.** Entering PID 1's
  namespaces needs `pid: host` and the capability `SYS_ADMIN` (`setns(2)`). `deploy/service.yml`
  grants those two and not `privileged: true`, which would also give every other capability, every
  host device and no seccomp/AppArmor confinement; nothing else in the driver uses either.
  **The camera start was verified on the robot (unit created, colour about 3 fps, depth about 4
  fps, top-down view about 4 fps) only while the container was still `privileged`. It has not been
  run with `SYS_ADMIN` alone yet, and the manual pages do not settle it**: whether the host's
  seccomp/AppArmor profile lets `setns` through, and whether `nsenter` may read `/proc/1/ns/*`,
  can only be found out on the robot. Acceptance step 7 is therefore the check that matters for
  this change. If it fails, the card says so in `camera_runtime.start_error`, quoting nsenter's own
  message and what to check, rather than looking like a unit that does not exist.
  If the camera cannot be started, the card still starts and says why in
  `camera_runtime.start_error`.
- **The camera cards and the lidar top-down view show nothing until the project is running and the
  card has been started** (the canvas's own start, not just the container being up). The first
  test on the robot looked like a broken driver for exactly this reason: `info` said `state: idle`.
  Everything else publishes from the moment the container is up, which is why those cards had
  data and these three did not.
- **The lift is not drawn in the skeleton.** The vendor URDF declares `body_joint` as `fixed`
  (and limits it to 0.6 m, while the robot reports 0.61), so `robot_state` carries the lift as a
  number and `joints` does not move it. The lift's unit is the vendor's to confirm, which is why
  the lift command is step-limited from the live reading rather than trusted as an absolute
  position. Which end of the gripper range is open (0) was confirmed by the operator, not the vendor.
- **`joints` leaves out anything not in the URDF** and any joint whose position is not a finite
  number, rather than sending `null`.
- **The head axes disagree between the vendor SDK document and the URDF** (the SDK says
  `joint_head0` is pan, the URDF makes it pitch about y, and `joint_head1` yaw about z). `head`
  therefore takes `head0_rad` and `head1_rad` by joint name rather than pan/tilt. It publishes
  `[head0, head1]`, assuming that is `head_forward_position_controller`'s joint order; the step
  limit from the live joint state would show a swapped order as a refused or small move, but
  confirm it with `ros2 param get /head_forward_position_controller joints` before using `head`.

## How it connects

```
robot, domain 0, CycloneDDS, bound to 127.0.0.1      this container (network_mode: host)
  /joint_states, /odom, /diagnostics, ...   ------>  main process: rclpy subscriptions
  /cmd_vel_teleop, controllers, actions     <------  Motion (when the robot's state allows it)
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
5. **Canvas, sensors**: start the project, then drag out `joints`, `imu`, `odom`, `robot_state`,
   `laser_scan`. `joints`
   follows the arms when the operator moves the robot with the vendor's tools (this driver
   commands nothing); `robot_state` shows a battery percentage (not `1`), `estop` `正常` with a
   small `age_ms`, grippers, lift, and `controllers` filled; `ready_for_motion` is true.
   `diagnostics.problems` names whatever is wrong on the robot (here the Nav2 lifecycle).
6. **Lidar**: `ros2 run tf2_ros tf2_echo base_link link_front_radar` should print (0.226, 0, 0.266),
   the pose the card reads from `/tf_static`; `laser_scan` should say `transformed: true` and show
   the front lidar `used` in `sources`. The rear lidar is expected to say `no data received`:
   this robot has no rear lidar (see above). Start the card and check the top-down view: `nearest_m` should be a real obstacle, not the robot's body, and
   `self_filtered_beams` should be greater than 0. If part of the body is still drawn, raise
   `laser.self_filter_radius_m`; if a real obstacle at the corner disappears, lower it.
7. **Cameras, with the reduced privilege** (the check this container's `SYS_ADMIN`-only setup
   depends on; the camera start was only ever tried under `privileged`):
   1. `docker inspect embodied-unixai-panther --format '{{.HostConfig.Privileged}} {{.HostConfig.CapAdd}} {{.HostConfig.PidMode}}'`
      must print `false [SYS_ADMIN] host`.
   2. Before starting any camera card, `systemctl is-active panther_orbbec` should be `inactive`
      (or `could not be found`), and nothing should be publishing
      `/camera/color/image_raw`.
   3. Start `camera_color`. It must show `camera_runtime.host_service_managed: true` and no
      `start_error`, and the picture appears. **If it shows a `start_error` beginning "could not
      enter the host's namespaces", the reduced privilege is not enough on this robot**: read
      nsenter's own message in it, find which capability or seccomp/AppArmor rule is missing, and
      add that one (do not return to `privileged`); record what it was here and in the PR.
   4. Then `camera_depth`. Stop both cards: `systemctl is-active panther_orbbec` must still be
      `active`. Record the encoding and size in each card's `info`.
   5. Stop the container (`... --down`): `systemctl is-active panther_orbbec` must become
      `inactive` within a few seconds, and the log says `asked systemd to stop`.
   6. Repeat with the camera started by hand first (`ros2 launch orbbec_camera gemini2.launch.py`):
      the card must reuse it, `camera_runtime.host_service_managed` must be `false`, and stopping
      the container must leave it running.
8. **Odometry**: at rest `vx`, `vy`, `wz` are 0.0 (not `null`). While the operator drives the base
   slowly with the vendor's own teleoperation (clear floor, emergency stop in reach), they must
   change. Do not push the 100 kg base by hand.
9. **The cards show their ranges**: open each motion card. Every number box's title shows a range
   (`左臂 la0（rad，-1.046 ~ 0.697）` and so on); `lifting` has no `操作` choice, only `position`. If
   the canvas shows no execute button for `lifting`, report it: the fallback is one line (an
   `action` with a single choice).
10. **Confirm the facts the head and lift rely on**:
    `ros2 param get /head_forward_position_controller joints`; the lift's unit and range with the
    vendor; `ros2 control list_controllers` against the names in `panther_motion.py`.
11. **Motion**, only with the operator beside the robot, the area clear and the emergency stop in
    reach. There is no switch to turn on: with intelligent control on, each call below moves the
    robot. Try in this order and stop at the first surprise: `base_drive` 0.05 m/s for 1 s;
    `base_drive` `stop` during a 3 s move; `head` `look` 0.1 rad on one box; `head` `home`, then
    `stop` half way; `lifting` 0.02 from its reading; `gripper` `open`, then `close`, one side, then
    `both`; `arm_control` `move` 0.05 rad on one joint (the other boxes empty); `arm_control`
    `home` on one arm, starting near zero. After each arm move check `ros2 control list_controllers`
    shows the original controller active again, and that Agent Core received the completion.
    Then trigger Agent Core's interrupt during a 3 s `base_drive` move: the base must stop.
12. **Not part of acceptance**: pressing the emergency stop cuts motor power and lets the arms
    drop. Only do it with the arms clear and the supervisor's agreement, and record `estop`'s text
    while pressed; it is the one value this driver has never seen.

Stop at any time with `sudo bash /opt/phanthy-motus/run-pr-image.sh --down`. That sends the stop
described above first, then stops `panther_orbbec.service` if this driver started it. A camera
that something else started is left running. If the container was killed instead, stop the unit
by hand: `sudo systemctl stop panther_orbbec`.

Open questions for the vendor (UniX AI): the lift unit, and the gripper's range beyond the
operator's 0 = open / 1 = closed; the head axis
naming and the head controller's joint order; what `/diagnostics` shows while the emergency stop is
pressed; why the vendor software starts a rear-lidar `bluesea_node` on a robot that has no rear
lidar (it publishes a topic that never has data, and shares the name `bluesea_node` with the front
one); the lidar frame names in the SDK document, which differ from the robot's; the Gemini 2
camera's profile and intrinsics.
