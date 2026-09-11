# RealMan RM75-6F-V Driver

This Linux ARM64 image exposes RM75 controller data and bounded joint motion to
Phanthy Motus through MCP. It calls the official RealMan API2 Python SDK
directly; it does not build or run the official ROS2 `rm_driver`.

```text
Card / Agent Core -> MCP Driver -> RealMan API2 SDK -> arm controller
```

The deployment service connects to `192.168.1.18:8080` with motion enabled by
default; set `RM75_ARM_IP` on the host to select a different controller. Before
starting it, verify the network, physical E-stop, work area and joint order.
Every movement still requires explicit per-call confirmation:

```bash
confirm_motion=true
```

Available tools are:

- `connection`: SDK connection state.
- `joint_states`: seven joint angles in radians, plus raw SDK degrees; also
  publishes a 10 Hz `sensor/skeleton` stream for the Canvas URDF renderer.
- `robot_info`, `software_info`, `arm_all_state`, `controller_state`: read-only
  API2 queries.
- `model`: simplified RM75-6F-V URDF for live skeleton display. Its seven
  movable joint names exactly match the `joint_states` skeleton stream.
- `joint_control`: bounded joint-space motion and controlled stop.
- `ext_camera`: multi-instance upper-computer USB camera card. A RealSense
  instance can publish RGB, depth, or left infrared without going through the
  RealMan controller.

## Upper-computer RealSense camera

The camera is connected to the Linux upper computer that runs this Driver,
not to the RM75 controller. Camera capture and arm control therefore have
independent data paths:

```text
ext_camera -> pyrealsense2 -> upper-computer RealSense USB camera
joint_control -> RealMan API2 -> RM75 controller TCP port 8080
```

`ext_camera` is a multi-instance sensor. Configure each card with an
`instance_id`, a RealSense selected by serial number, and one `channel`:

| channel | topic | format |
| --- | --- | --- |
| `rgb` | `/{namespace}/ext_camera/{instance_id}/rgb` | `image/jpeg` |
| `depth` | `/{namespace}/ext_camera/{instance_id}/depth` | `image/depth-zlib` |
| `infrared` | `/{namespace}/ext_camera/{instance_id}/infrared` | `image/jpeg` |

Hyphens in `instance_id` become underscores in the ROS topic. Discovery and
capture use `pyrealsense2` through `/dev/bus/usb`; the selected camera is bound
by its stable serial number rather than a dynamic `/dev/videoN` path. Existing
single-camera Canvas projects that saved an old `/dev/videoN` selection are
migrated automatically when they next start.

Depth is 640x480 little-endian uint16 millimetres compressed with zlib. Zero
is invalid/unrepresentable depth. Infrared is the left Y8 stream rendered as a
grayscale JPEG; it is reflected near-infrared intensity, not temperature.
RGB, depth and infrared instances on one camera share one SDK pipeline. The
pipeline captures each physical stream once and fans it out to every card that
selected that channel, so three cards do not compete for the D435.

The image pins `pyrealsense2==2.56.5.9235`, matching the repository's verified
Linux ARM64 / Python 3.10 runtime. The image does not install the `v4l-utils`
command-line package. Do not run `realsense-viewer` or another capture process
against the same D435 while the card is active.

The deployment is not privileged and does not mount the host's complete `/dev`
or any `/dev/videoN` node. It mounts `/dev/bus/usb` and allows USB character
device major 189 so librealsense can enumerate, capture and reconnect the
selected camera. The arm's API2 TCP connection needs neither grant. With no
camera attached, the container still starts and all arm/gripper cards remain
available; `ext_camera` reports an empty device list until a RealSense is
connected. The USB-bus mount exposes connected USB descriptors to the container
but does not grant access to unrelated host block, input, serial, GPU or media
devices.

The observed target hardware is an Intel RealSense D435 (USB ID `8086:0b07`).
On USB 3, the shared pipeline uses RGB 1280x720 and depth/infrared 640x480 at
15 fps. A USB 2 connection falls back to 640x480 at 6 fps for all streams.

For a supervised hardware check, create separate RGB, depth and infrared card
instances, start them, then confirm that `frames_published` increases and the
three topics render independently. USB disconnect becomes an explicit error;
after reconnect the same serial-number selection can be started again.

The deployment enables its motion capability, and every `set` call must still
include `confirm_motion=true`. `joint1_deg` through `joint7_deg` are absolute
targets in degrees. An omitted joint defaults to its measured position at the
start of the request, while the supplied targets are sent together as one API2
`movej` trajectory so the controller plans all joints concurrently. The
Driver rejects non-finite values, targets outside the
official RM75 limits, speed above 10 percent, disabled
joints, and any reported arm or joint error. It sends non-blocking API2
`rm_movej`, monitors the measured joints until they reach the target, and
registers `stopmotion` as the Agent Core `on_interrupt_motion` hook so it
bypasses an active ACP barrier. The card does not expose a
fixed `timeout_seconds`: after a 2-second startup grace period, the driver asks
for a controlled slow stop and reports `motion_stalled` through ACP if the
maximum joint error has not improved by at least 0.05 degrees for 10 seconds.
It also derives an internal deadline from commanded distance and speed, capped
at 300 seconds, as a final safeguard.

The first supervised hardware test should change exactly one joint by no more
than 1 degree at 1 percent speed. A reachable physical E-stop and a clear work
area are required. Software interlocks do not replace the robot safety system.

The HTTP service listens on port `15718` and provides `/health` and `/mcp`.
The normal Agent Core runtime still initializes its ROS/DDS transport, but robot
communication itself goes directly through API2 TCP port `8080`.

Only the Python SDK wrapper is stored in Git. The operator's licensed Linux
ARM64 `libapi_c.so` must be installed on the robot host at
`/opt/realman/rm_api2/libs/linux_arm/libapi_c.so`; `service.yml` mounts that
directory read-only at the path expected by API2. The expected SHA-256 is
`5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9`.
Verify the operator-provided file before starting the enabled Driver:

```bash
echo "5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9  /opt/realman/rm_api2/libs/linux_arm/libapi_c.so" \
  | sha256sum -c -
```

The image can still be smoke-tested manually with `RM_DRIVER_ENABLED=0` without
the library, but the deployment service defaults to a live, motion-capable
connection. An enabled connection fails with an explicit mount error when the
library is absent. Set `RM_API2_LIB_DIR` to override the host directory.
Following the standard ACP contract in `README_dev.md`, `joint_control.set` immediately returns a
unique `action_id`; its background monitor later reports exactly one
`completed`, `error`, or `cancelled` terminal result to `/api/acp/complete`.
The callback reads `AGENT_CORE_URL` inside the worker-thread function and uses
the same HTTPS behavior as the documented G1/R1 implementation. Agent Core
owns pending-action barrier release and completion-event delivery.

The immediate card result contains only `state` and `action_id`. Completion
callbacks keep the standard status and a short reason; full final joint evidence
is available from `joint_control.info` in `last_completion`. Its `callback` is
`accepted` only when Core acknowledges the same ID, or `failed` with an error.
Acceptance confirms HTTP receipt, not pending-action matching or a visible UI
trigger. No extra `/api/event` notifications are sent. This Driver cannot repair
a stalled Core decision loop; a missing UI trigger alone is not proof of motion
or callback failure.

The component installs `python3-pip` to install the pinned camera wheels,
`python3-yaml` because the shared runtime loads `config.yaml`, and
`ros-humble-rmw-fastrtps-cpp` because the shared runtime creates the Agent Core
ROS 2 participant. The camera layer installs the ARM64 `pyrealsense2` wheel,
NumPy 1.23.5 and headless OpenCV 4.11.0.86 through pip. The wheel supplies its
own RealSense implementation but dynamically loads `libusb-1.0.so.0`. The build
downloads Ubuntu's signed `libusb-1.0-0` runtime package and extracts only that
shared object into `/opt/realman/libusb`; it does not execute or suppress package
maintainer scripts and does not claim the package is installed in dpkg. Direct
The SDK USB backend avoids `v4l-utils`; no compiler, desktop OpenCV backend or
ROS build tooling is installed. Every download/extraction step must succeed and
the apt layer must finish with an empty `dpkg --audit`; package-install failures
and partially configured package states are never accepted.
See `vendor/SOURCE.md` for provenance notes.

The RM75 API2 TCP path itself does not require privileged mode or host devices.
The bundled upper-computer camera receives only the USB bus access required by
RealSense enumeration and capture, as described above.
Skeleton publication skips disconnected/busy SDK clients, retries failed samples
at most every two seconds, and logs once per outage until a successful sample.
An already-running SDK TCP query still holds the SDK lock until it returns.
