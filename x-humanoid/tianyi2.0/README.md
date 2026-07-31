# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

## System and camera utility cards

The three cards are grouped into the existing runtime modules rather than
implemented as one Python file per capability:

| Tool | Type | Runtime module | Purpose |
|---|---|---|---|
| `bag_recorder` | `actuator` | `SystemPlugin` | Start/stop the official rolling ROS bag recorder and inspect sessions |
| `software_manifest` | `resource` | `SystemPlugin` | Read the complete robot software and firmware version inventory |
| `camera_geometry` | `resource` | `CameraPlugin` | Read RGB/depth intrinsics and depth-to-color extrinsics |

### Bag recorder

`bag_recorder` exposes four bounded actions: `start_recording`,
`stop_recording`, `status`, and `list_sessions`. It always launches the fixed
official command `ros2 launch utils record_trigger.py`; MCP callers cannot
inject a shell command, topic list, setup path, or output path. The process runs
inside the host namespaces through `nsenter`. A host process scan plus a
host-wide `flock` reject duplicate starts across driver restarts and concurrent
instances. Managed recordings are stopped with `SIGINT` before bounded
`SIGTERM`/`SIGKILL` fallbacks; a recorder started outside this driver is
reported but never signalled automatically.

The official recorder reads its topic configuration from
`/home/ubuntu/ros2ws/install/utils/lib/utils/bag_record/config/record.json` and
writes rolling sessions under `/home/ubuntu/bags`. The Tianyi documentation
defines 100 MB segments and a 4 GB rolling limit; those retention settings stay
owned by the official recorder rather than being duplicated in this driver.
Before launch, the card validates that the setup and recorder JSON exist and
that the JSON is structurally valid, then probes the child process for an
immediate startup failure.

### Software manifest

`software_manifest` reads
`/home/ubuntu/ros2ws/version_info.json` through the host mount namespace. The
resource preserves the complete JSON object, including current x86, Orin and
firmware fields plus future unknown fields. Reads are limited to 1 MiB and
invalid UTF-8, non-standard JSON constants, non-object roots and filesystem
errors return stable error codes. MCP arguments cannot override the configured
path.

### Camera geometry

`camera_geometry` caches:

- `/ob_camera_head/color/camera_info`
- `/ob_camera_head/depth/camera_info`
- `/ob_camera_head/depth_to_color`

Each camera stream returns resolution, `fx/fy/cx/cy`, distortion coefficients,
K/R/P matrices, ROI, frame ID and source timestamp. The optional
depth-to-color transform returns a 3×3 rotation matrix and translation in
metres. The driver builds the minimal official-compatible
`orbbec_camera_msgs/Extrinsics` interface and subscribes with transient-local
QoS so it can receive the one-shot calibration message after the camera service
has already started.

Missing streams are reported through `availability`, `missing` and
`optional_missing`; RGB streaming remains usable when geometry is partial.
Depth-to-color calibration does not include a camera-to-robot-base transform,
which still requires TF or a separate hand-eye calibration.

`SystemPlugin` requires the same `--pid host` and `--privileged` deployment
settings already used by `CameraPlugin` to reach the host system and filesystem.

## Head gesture card

`head_gesture` turns safe, bounded head-position commands into cancellable
semantic sequences.

| Item | Value |
|---|---|
| Tool name | `head_gesture` |
| Tool type | `actuator` |
| Robot-side output | `/head/cmd_pos` |
| Message type | `bodyctrl_msgs/msg/CmdSetMotorPosition` |
| Actions | `nod`, `shake`, `scan`, `tilt`, `reset`, `stop` |

Yaw, pitch and roll are clamped to the limits already documented by the raw
`head` card. Starting a new gesture cancels the remaining frames of the previous
gesture; `stop` cancels future frames without issuing an additional pose. A nod
moves from neutral to positive pitch (down) and back to neutral; it never passes
through negative pitch (up).

Before publishing, the card checks fresh `/head/status` data, all three head
motors, motor error codes, emergency-stop state and power state. It then waits
up to two seconds for newer status and verifies that the head moved or was
already at the target. A verified call returns `feedback_verified: true`;
failures return `state: error`, a stable diagnostic `code`, details in the MCP
result, and protocol-level `isError: true`.

| Action | Dashboard defaults | Allowed range |
|---|---|---|
| `nod` | cycles 2, amplitude 12°, speed 30°/s | cycles 1–5, amplitude 5–20°, speed 5–60°/s |
| `shake` | cycles 2, amplitude 25°, speed 30°/s | cycles 1–5, amplitude 5–45°, speed 5–60°/s |
| `scan` | cycles 2, amplitude 25°, speed 30°/s, hold 1.0s/side | cycles 1–5, amplitude 5–45°, speed 5–60°/s, hold 0.2–3.0s/side |
| `tilt` | left, amplitude 12°, speed 30°/s, hold 0.8s | side left/right, amplitude 5–20°, speed 5–60°/s, hold 0.2–3.0s |
| `reset` | speed 30°/s | speed 5–60°/s |
| `stop` | no parameters | no parameters |
