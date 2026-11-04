# Franka FR3 joint control, state and external camera driver

One ARM64 container exposes `joint_control` plus read-only `joint_states` and `ext_camera` sensor
cards, plus the `model` resource automatically fetched for visualization. Each arm-state/control
instance has its own persistent `robot_ip` setting in the canvas gear menu; camera
instances select a USB serial independently. Multiple arm cards
using the same IP share one state reader; different IPs have independent readers
and retry timers. No IP, credentials or installation-specific configuration is
embedded in the image.

The gear menu automatically lists discovered FR3s by model, serial number and
IP. Choose a robot and save; this does not connect to FCI until the card starts.
Choose **手动输入 IP** and fill **手动 IP** under advanced settings for an
unlisted robot. This option remains available even when no devices are found,
so a saved manual selection keeps the same form after a restart or scan failure.
Existing IP-only configurations continue to work when starting a card.

Discovery reads Desk HTML and one `/admin/api/robot/nameplate` WebSocket message;
it never uses FCI or sends control commands. It runs asynchronously on startup,
then refreshes its cache at most every two minutes when tool descriptions are
requested and no card is reading state. Core's normal registration refresh
fetches the schema; reopen the gear menu after discovery, or refresh the console
if it still has an older description. No Core changes or additional cards are
required. Discovery does not change an already selected IP.

Only UP physical wired interfaces with directly connected private IPv4 /24–/30
subnets are scanned (up to four, 12 concurrent probes, a 60-second scan budget).
When the budget expires, unfinished probes are cancelled and already identified
robots remain available.
Wi-Fi, bridges and broad LAN subnets are excluded. Hosts receive bounded HTTPS
probes; only matching FR3 Desk pages and connected FR3 nameplates are listed.
Discovery is a device-type filter on a trusted local network, not authentication:
Desk certificates name `robot.franka.de`, so IP discovery skips certificate
verification, sends no credentials, ignores proxies/cookies and rejects redirects.
The observed Desk identity endpoint may change with firmware. Discovery failure
never blocks manual configuration or state reads, and discovery is not proof of
FCI readiness. Configured IPs known to this driver process remain selectable if
temporarily absent.

**Core dropdown limitation:** Core saves card settings without sending them to
the driver until startup. An IP saved before discovery finishes, or before a
driver restart, may therefore be absent from the driver's dropdown. Core shows
an empty selection in that case and saving it would omit the saved IP. Before
saving, choose the manual option and enter the original IP; do not choose another
arm to dismiss an empty selection. Starting an existing card without editing its
configuration continues to use its saved IP. Fully preserving unknown saved
options requires a Core form change, which is outside this driver change.

| Card | Type | Configuration | Output |
| --- | --- | --- | --- |
| `joint_states` | multi-instance sensor | discovered `robot_ip` or advanced `manual_robot_ip`; `publish_hz` (1–30, default 10 Hz) | `sensor/skeleton`, seven actual joint angles in radians |
| `model` | resource | none | FR3 visualization URDF |

## Requirements

- ARM64 Ubuntu 22.04 / ROS 2 Humble container environment.
- Supported FR3 robot on a wired, reachable network, with a unique IP per robot.
- FCI installed and activated by the operator in Desk. The driver never takes a
  control token, unlocks brakes, activates FCI, performs recovery or commands motion.
- The image pins libfranka **0.18.1**, FCI protocol 10, for robot system **5.9+**
  per the [official compatibility matrix](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html).
  Firmware 5.10.0 is the initial deployment target. Older protocols require a
  separately validated SDK build; Panda and mixed robot models are not advertised
  as supported by this image.

This release only observes state. It does not validate the host for 1 kHz motion
control. The 10 Hz card setting is a display rate, not a control-loop rate.

## Local build and deployment

The repository build context contains the driver directory plus `common/`.
For a local image without the registry push behavior of `build.sh`:

```bash
# Run from the repository root.
context=$(mktemp -d)
cp -a franka/fr3/. "$context/"
cp -a common "$context/common"
docker build -t franka-fr3:local-review "$context"
# Optional: --build-arg ROS_BASE_IMAGE=ros:humble
```

The Dockerfile defaults to the public TUNA Ubuntu mirror and the official ROS
repository. Override `APT_MIRROR` and `ROS_APT_MIRROR` with `docker build
--build-arg` when needed. The repository's `build.sh --mirror` option currently
selects the Python package mirror only; it does not override these APT sources.

The SDK and its `common` submodule are built from checksum-verified official
source archives mirrored on the project's Tencent COS bucket, following the
`unitree/g1` download pattern. Override `LIBFRANKA_URL` and
`LIBFRANKA_COMMON_URL` with `docker build --build-arg` to use another mirror
of the same archives; SHA-256 verification remains mandatory. SDK archives are
not stored in this repository. Downloads have bounded timeouts and retries.
The archives preserve their upstream licenses and notices; a small CMake patch
supplies the pinned release version for package metadata without requiring
`.git`. Source revisions, download URLs, checksums and the patch are documented
in [`vendor/SOURCE.md`](vendor/SOURCE.md). Base-image, APT and SDK downloads
require network access.

The runtime image uses an explicit libfranka SONAME link and
`LD_LIBRARY_PATH=/usr/local/lib` for the copied SDK libraries. This avoids a
final-stage `ldconfig` invocation, which can crash under some ARM64 QEMU
versions. The build still runs the reader without arguments and requires exit
code 2 to verify library loading without connecting to a robot.

`deploy/service.yml` follows the resource-center service-fragment convention
(`__IMAGE__` is replaced by the selected image). For standalone local Compose,
wrap that fragment in a top-level `services:` mapping and replace `__IMAGE__`
with `franka-fr3:local-review`. Keep the host networking, domain 42 and
read-only `/opt/phanthy-motus/dds-local.xml` mount; confirm that file exists on
the host before deploying. No privileged mode, robot
device mounts or real-time capabilities are required for this reader.

The driver ID is **franka-fr3-driver** and its MCP port is **15723**. It registers
as **Franka FR3** with Core, which assigns its own MCP record ID. Starting the
container registers cards and starts read-only Desk discovery; an FCI state
connection is opened only when a `joint_states` card is started.

## Canvas workflow

1. Add a `joint_states` card and select its robot with the card gear button,
   or use the manual option and advanced IP field.
2. Start the project. The card reports `loading` until a valid sample arrives,
   then `running`. Its info response includes `connected`, `readiness`,
   `data_age_ms` and `error` for diagnosis without a separate status card.
3. In the monitoring dashboard, view the `joints` stream for the FR3 skeleton.
   The renderer fetches the driver's `model` resource automatically; it is not
   a separate sensor card and needs no IP or canvas wiring.
4. Repeat with another IP for another FR3.

Core persists per-card IPs. Its sidebar can also save shared defaults; an
explicit card IP overrides them. Stop a card before changing its IP or frequency.
Deleting/stopping one card releases only its own lease. The last consumer closes
that IP's reader. One disconnected robot does not prevent other robots being read.

Topics use `/franka/<card_id>/state/joints`, following the `state/joints`
suffix used by the RealMan and other robot drivers. The card ID provides an
instance namespace so several arms/cards cannot overwrite each other's poses.
For example, `card-franka-joints` publishes to
`/franka/card_franka_joints/state/joints`. The topic is available before IP
configuration or connection. Hyphens in card IDs become underscores; collisions
are rejected. IP settings are marked `x-sensitive` for Solution export redaction.

The reader rejects malformed/nonfinite samples and stops publishing joint poses
after the 2-second freshness deadline. A renderer may retain its last pose;
check the joint card's info response for freshness and errors instead of
interpreting that pose as a live reading.
If DDS publication fails, the card reports `state: error` and
`publication_error`, retries on the next tick, and leaves other cards running.
Reader connection attempts time out after 5 seconds and retry with bounded
backoff. The SDK runs in a child process per IP so blocked I/O can be cancelled
without blocking other devices or killing the driver.

## Current Core visualization limitation

Core `release.261011.46afd50` passes the placeholder driver ID `detail` from the
canvas's “查看数据流” panel to the skeleton renderer. That panel consequently
cannot fetch this driver's URDF. The monitoring dashboard passes the real driver
ID and is the supported validation path for this release. No Core files are
modified by this driver. Core currently fetches one model per driver, so all
instances in this release use the same FR3 chain. The startup dialog also groups
progress by tool name rather than instance ID; verify each card using its own
IP, info response and topic when using several robots.

## Development checks

```bash
python3 -m pip install -r franka/fr3/requirements.txt pytest
python3 -m pytest tests/test_franka_hardware.py tests/test_franka_driver.py tests/test_franka_discovery.py
python3 -m pytest tests/test_dockerfile_copies.py tests/test_common_lifecycle.py
python3 scripts/check_service_yml.py franka/fr3
```

Tests use process and DDS fixtures. Hardware validation must additionally confirm
real FCI samples and DDS reception inside Core; successful HTTP registration is
not sufficient. Two physical FR3s have been verified for discovery; only one has
been validated for FCI state reads. Multi-IP reader isolation is tested with
fixtures and an unreachable second IP, not claimed as a multi-arm FCI test.


## Hardware validation scope

Validated on an ARM64 Jetson running Ubuntu 22.04.5, an FR3 with system 5.10.0
and operator-activated FCI, libfranka 0.18.1, and Core
`release.261011.46afd50` (with a separate local fix for the canvas detail panel).
Acceptance covers live DDS samples, matching URDF joint names and units,
same-IP reader sharing, an unavailable second IP, SDK process recovery,
stop/restart cleanup, and both Web visualization entry points. No motion
commands are used in these checks.

Only `joint_states` needs to be placed on the canvas. If the monitoring dashboard
was opened before project startup, reopen it after startup so Core refreshes
the topic formats. Tests cover multiple sessions with fixtures; multiple physical
arms and their network capacity still require hardware validation.

## External RealSense camera

This development version retains the read-only joint monitor and
model, and adds only `ext_camera`. It contains no joint-control, gripper or master
plugin. Camera capture is independent of robot-controller power and FCI.

Select the device by USB serial in each card. The serial is deployment configuration,
not embedded in the image. RGB and infrared publish JPEG; depth publishes
`image/depth-zlib`: zlib-compressed 640x480 little-endian uint16 millimetres,
with zero indicating invalid or unrepresentable depth. Depth is not RGB-aligned.
The card's `info.topic_out` reports the exact selected channel and topic.
Multiple cards on one serial share a capture session. Saving a running card automatically switches its camera/channel. Same-camera
channel changes update the publishing route without reopening USB capture.
Idle cards save configuration without starting capture. In the canvas, close
the preview, select a channel with the card gear, save, and reopen the preview.
An already-open preview remains subscribed to its original topic. Missing cameras retain their selection and
retry without substituting another serial. Inspect `fresh`, `frames_published`,
`state` and `error`; a frozen renderer image alone is not proof of a live camera.

The backend is local to `franka/fr3/camera_backend.py` and `realsense.py`;
no RealMan robot SDK is required. The service fragment exposes host USB/video
nodes using device cgroup rules, without privileged mode. Its `/dev` mount is
read-only, while device I/O remains permitted and `/dev/shm` is a writable tmpfs.
Do not open another camera capture program on the same serial during validation.

Offline tests: `python -m pytest tests/test_franka_ext_camera.py tests/test_franka_realsense.py`.
Physical RGB/depth/infrared, USB reconnect and ARM64 image acceptance remain required.

For the 2026-11-02 development build, the verified network overrides are:

```bash
docker build --network host \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple \
  -t franka-fr3:camera-standalone-20261102 <prepared-context>
```

These overrides select dependency mirrors only; Core source and runtime are not modified.

## Camera frame and timing semantics

Each physical camera/channel uses a stable optical frame ID containing its serial
identity. Multiple cards selecting that same camera/channel share the frame ID;
different cameras do not. This is an identity label, not a camera-to-arm transform.
`info.frame_id` matches the image header.

`timestamp_source=publication_clock` identifies ROS publication time; hardware
capture time is not currently exposed. `fps` remains the SDK profile rate, not a
measured capture or publish rate. Depth reports `depth_unit=mm`,
`invalid_depth_value=0`, and `depth_aligned_to_rgb=false`. It is native depth,
not registered to the RGB image. Infrared is the left reflected-near-infrared
intensity image; it measures neither temperature nor distance.

## Camera interface declaration

Each ext_camera instance has one dynamic output port. tools/list declares one
placeholder; instance info returns its exact topic and image/jpeg or
image/depth-zlib format. Consumers must resolve info before subscribing.
camera_info.id is the stable family-level intel/realsense/<channel> identifier,
independent of USB discovery and display names. It does not assert a D435i lens
calibration: optics remain unknown. Physical identity remains in serial/frame_id.

## Joint control

`joint_control` is an actuator with per-instance robot_ip and speed_deg_s.
Start binds the selected robot; it never initiates motion. `set` supplies absolute
J1–J7 degrees (at least one), `jog` applies a signed increment once, and
`stopmotion` requests a confirmed controlled stop while retaining the card.
Project/card stop releases its lease. Motion interrupts use stopmotion, including
all active arms when the platform sends no instance_id. Omitted joints use fresh
actual state. Targets retain a 0.01 rad joint-limit margin, speed is 0.1–10 deg/s,
and the seventh-order trajectory bounds acceleration and jerk. Motions exceeding
the fixed 120 s budget are rejected before backend submission without changing speed.

Python manages cards and shared sessions; the C++ libfranka 0.18 backend owns one
connection per robot IP. The standard Jetson deployment uses
`FRANKA_REALTIME_POLICY=standard` and leaves `FRANKA_REMOTE_HOST` unset.
It permits the stock Jetson kernel with libfranka's `kIgnore` option and
still requires FIFO scheduling. Set `FRANKA_REALTIME_POLICY=enforce`
to require PREEMPT_RT as well. The native binary defaults to enforce when
run outside the deployment. Check scheduling and network performance on
the target before motion acceptance.

Configure `robot_ip` on each card before starting it. The driver validates it
at runtime. The schema omits the required marker to support Core versions
whose manual actuator call omits the instance ID. The driver binds such calls
to a unique started card using the supplied output topic.

Both `set` and `jog` can run for several seconds depending on travel and
speed. They use one asynchronous contract, returning an action ID and
reporting completed, cancelled or error through ACP. The state stream
uses J1–J7 order. `actual_deg` and `target_deg` use degrees, and fields
ending in `_rad`, `motion.target` and `motion.max_error` use radians.
`desired_rad` is the controller's instantaneous desired position.

Keep the previous driver container and configuration when deploying a new
image. To roll back to an AOOSTAR deployment, stop the replacement driver
and restore the previous container with its remote backend settings.

Core interruption and card lifecycle
remain separate; stopping without backend confirmation is an error. These are
software interlocks, not collision-free path planning or a substitute for the
physical emergency stop. Neither Cartesian control nor automatic obstacle
avoidance is provided. Build/installation alone does not pass physical acceptance.
