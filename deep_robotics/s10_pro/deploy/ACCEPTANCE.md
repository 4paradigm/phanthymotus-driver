# Deployment and real-canvas acceptance

## Submission checkpoint, 2026-10-22

The field deployment remains the locked six-card driver at source `53e517db`,
image `s10-pro-motion:53e517db`, ID
`sha256:0f3959dd8e3bdaae184591473589dff7e9456e2551d1d4d544fe0c7c9f61db14`.
The new DDS high-level motion candidate in this submission has not replaced that
running driver. Further deployment and physical testing are paused for this PR.

The production constructors set all three execution gates to `False`:
`DDSProcessTransport.execution_enabled`,
`DDSMotionBackend.execution_authorized`, and
`MotionPlugin.execution_authorized`. Starting the motion card acquires only a
motion-feedback observation lease. It does not start a command worker, create a
command publisher, change robot mode or grant control authority. Canvas settings
cannot change these process-level gates.

An ARM64 Docker candidate passed isolated real-ROS synthetic motion smoke:
feedback-dependent state `1 → 17`, bounded velocity followed by zero and stopping
confirmation, and lie confirmation. Host and candidate recursive definitions
matched for the nine motion-related message types. See the PR for the submission
check results. These checks do not establish robot motion acceptance.

The persistent field application directory is `/userdata/phanthy-motus-s10` and
the user-facing Core URL is `https://10.21.33.106:15678`. Credentials and access
tokens stay outside this repository. Browser control remains disabled in Work
settings; visible point-cloud rendering, screenshots and refresh persistence
have not been accepted.

## Historical field evidence and its version

The five-card image `s10-pro-readonly:3a1cb138`, ID
`sha256:ffdaa0eb9187299028d63f2fde87de842cb2fe8e129e6c56bc10617de271dc28`,
passed the following field checks before the locked motion-card upgrade:

- All six sensor outputs passed a 30-second Core WebSocket check: state about
  1 Hz; joints, IMU and odometry about 4.95 Hz; lidar metadata about 2 Hz. Lidar
  delivered 27 binary frames at about 0.90 Hz, 6000 points each, maximum gap
  3.01 seconds. JSON parsing and unhealthy-frame counts after warmup were zero.
  The timestamp-free initial cloud cache was excluded. This verifies the Jazzy
  driver → Humble Core → WebSocket path, not browser rendering or vendor sample
  frequencies.
- Three independent sensor-card lifecycle rounds passed. Stopping one card
  stopped its output while other cards continued. After the last sensor card
  stopped, the reader and its six source subscriptions were absent. Restarts
  created one reader and one subscription per source. With the new motion
  observer, the reader also remains leased while that observer is running;
  repeat the final-owner check with the motion observer stopped.
- Suspending only the new application reader made the five cards stale. Cloud
  binary output stopped while stale metadata continued. Resuming it restored
  fresh data; terminating it caused automatic reader restart and recovery.
  Vendor processes and robot networking were not interrupted.
- An uninterrupted 600.09-second Core DDS observation received 599 state,
  2946 joints, 2970 IMU, 2966 odometry, 576 cloud and 1194 lidar-info frames.
  Every JSON frame was fresh, each cloud had 6000 points and parsing had no
  errors.
- The 21-item HTTPS API audit passed, including 401 for unauthenticated access.
  Automatic project startup was off and `project_running=false` at audit time.

The later `53e517db` six-card locked deployment passed real sensor-data and locked
motion-status channel checks. Neither set of historical results substitutes for
repeating acceptance on the new PR image. Working evidence is retained privately
outside the repository; database copies and credentials are not PR artifacts.

## Before a subsequent deployment

1. Follow the team PR/image workflow and record the generated image, source
   commit and build result. Do not assume a locally tested candidate is the bot
   image. Preserve the running application while waiting for that image.
2. Confirm robot Wi-Fi and the NOS address. Read current services, free ports and
   writable persistent paths. Preserve the independent S10 application and any
   user edits; do not repeat initial provisioning over its data or touch the
   separate Franka/PiPER stacks.
3. Save an application-only container/config inventory and affected backups
   privately. Do not put credentials or database copies in Git.
4. Run sensor `preflight.py --public-contract` and motion
   `motion_preflight.py --public-contract` in the native NOS Jazzy environment,
   then compare the container's recursive message contracts. The installed
   native message package is 1.0.23; version names alone do not establish wire
   compatibility. Repeat whenever either message package or image changes.
5. Verify the production execution gates remain false and that no command worker
   or application command publisher starts during idle, sensor use, motion
   observation, stopping or graph inspection. No action command belongs to this
   deployment check.

## Supported Docker topology

Core and driver are separate containers on the same NOS. Core uses the existing
Humble application image; the driver uses its own Jazzy image and compiled public
message definitions. Do not mount host Jazzy binaries into the Humble container.
The optional systemd unit is an unused diagnostic/development template, not the
field deployment mechanism.

The NOS currently has no Docker Compose plugin. Field containers are managed
with Docker CLI; the application's `docker-compose.yml` records the intended
configuration. `service.yml` is the standard platform installation template.
Use the actual deployment tools available on the robot when applying bot output.

Both containers use host networking. Platform DDS uses domain 42 with the
loopback-only `dds-local.xml`; MCP listens on 15745 and the independent namespace
is `s10_pro`. The reader uses vendor domain 0 and explicitly excludes the
platform profile. It never executes vendor `setup_ros2.sh`, edits XML, publishes
commands or calls robot services. The default command worker is dormant.

For the container reader, install this application's `s10-reader-udp.xml` as a
real regular file, mount it read-only at
`/opt/phanthy-motus/s10-reader-udp.xml`, and set external
`reader.vendor_profile` to that path. The field host source is
`/userdata/phanthy-motus-s10/s10-reader-udp.xml`; repository `service.yml` uses the
standard `/opt/phanthy-motus` installation prefix. Adjust only the host mount
sources for an independent installation.

This child-only XML selects UDPv4 and disables builtin transports. Default
container transport received drdds state but missed local standard IMU,
odometry and cloud; host IPC alone did not fix this. The dedicated UDP profile
restored all three. Private IPC is retained. Vendor XML and services remain
unchanged, and the platform parent keeps its domain-42 loopback profile.

The S10 Core has its own persistent database and access configuration. For a new
installation, verify architecture and standard-message transport before reusing
an application image. Do not clone another robot's live database, secrets or
canvas state. Keep smart control and automatic project startup disabled during
sensor acceptance, and preserve existing application state on an occupied host.

## Repeat on the exact PR image

Record source commit, image ID, config hash, Core ID/version and user-openable
URL. These are pending acceptance items for the new candidate:

1. Repeat MCP/API and Core WebSocket checks for five fresh sensor streams and
   motion observation. Motion must report execution locked, and the command
   worker must remain absent. No physical action is required.
2. Repeat three independent lifecycle rounds and reader interruption/recovery.
   Stop every sensor card and the motion observer before checking final reader
   exit. Verify no source subscription leaks or command publications.
3. Complete an uninterrupted ten-minute Core DDS observation and record delivered
   rates, maximum gaps and source ages. Refresh settings are not vendor sample
   frequencies.
4. Once real-browser access is available, inspect the saved six-card layout,
   settings and ports. Start sensors individually and together; verify fresh
   state, 16 joint records, IMU and odometry. Inspect visible updating point
   geometry together with lidar metadata, since a renderer may retain old
   geometry after a stream stops.
5. Refresh the browser and check layout/settings persistence. Save screenshots
   against the same deployed version. Browser inspection must be performed
   before it can be recorded as passed.

## Motion development boundary

The high-level backend uses `/MOTION_STATE`, `/NAV_CMD` and `/MOTION_INFO` through
an independent bounded command process. It does not require TLS. The optional
ASDU/TLS transport remains experimental and is not selected by production;
previous AOS TLS diagnostics sent no application frames.

The handheld-reported `v2.5.5` has not been mapped to the robot system/control
service version. Fresh motion feedback does not prove ownership, navigation
mode, handheld takeover priority or a robot-side loss-of-command stopping
bound. The guide's two-second heartbeat timeout concerns status reporting and
is not evidence of a motion deadman.

Before any later proposal to enable execution, establish robot identity/version
matching, independently verifiable control authority and navigation-mode rules,
robot-side stopping behavior, and on-site readiness/takeover conditions. The current
backend conservatively requires ownership even to publish zero velocity; a
contract claiming special stopping rights after authority loss does not by
itself implement those rights. Do not bypass the publisher's ownership checks.

The public `sdk_deploy` repository is a low-level control/RL path whose
initialization can publish joint reset commands. Do not run it as a connectivity
probe, and do not use M20 control encodings for S10. No stand, lie, velocity,
mode-switch, reset or emergency-stop command has been sent to this robot by
this work. Tests with fictional authority and synthetic ROS publishers are
limited to `--network none` containers; both smoke scripts reject non-loopback
interfaces.

## Application-only rollback

Stop only the current `embodied-deep-robotics-s10-pro` container with Docker CLI;
do not invoke unavailable Compose tooling. Verify its parent/reader processes
have exited. Restore only the backed-up S10 image/config and independent canvas
state, checking for user edits before any database restore. Preserve the
persistent application directory and prior image for recovery.

Do not restart vendor AOS/NOS services or the Franka/PiPER stacks. Stopping or
rolling back this application does not disable robot motion and is not an
emergency stop.
