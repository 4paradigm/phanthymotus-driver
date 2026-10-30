# Phanthy Motus Drivers

[中文文档](README_zh.md) | [Official Website](https://motus.phanthy.com)

Hardware drivers for the **[Phanthy Motus](https://github.com/4paradigm/phanthymotus)** embodied AI platform.

Each driver is a standalone [MCP](https://modelcontextprotocol.io) HTTP server that exposes hardware capabilities as tools. Drivers automatically register with the [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) on startup.


## Available Drivers

| Driver | Hardware | Port | Description |
|--------|----------|------|-------------|
| `pico/4ultra` | PICO 4 Ultra | 15742 | Independent teleop_device; signed App, pairing, DDS controller input |
| `unitree/g1` | Unitree G1 Humanoid | 15701 | Locomotion, arm control, mic, speaker, LED, state monitoring |
| `unitree/go1` | Unitree Go1 (EDU) Quadruped | 15715 | State, locomotion, camera (RGB/depth/pointcloud), ext peripherals, URDF |
| `unitree/go2` | Unitree Go2 Quadruped | 15703 | Locomotion, obstacle avoidance, voice, video, navigation |
| `unitree/r1` | Unitree R1 (EDU) Humanoid | 15702 | Mic, TTS, LED, locomotion, stereo camera, state monitoring |
| `dji/M300` | DJI Matrice 300 RTK | 15702 | Flight control, telemetry, perception, HMS, aircraft info |
| `dji/mavic3e` | DJI Mavic 3E/3T | 15702 | Flight control, camera (wide/zoom/IR), gimbal, waypoint missions, perception, IR thermometry |
| `dji/mavic4e` | DJI Mavic 4E/4T | 15703 | Flight control, camera, gimbal, waypoint missions, telemetry, perception |
| `engineai/t800` | EngineAI T800 Development Edition | 15708 | ROS2/Native SDK, full state, dance/gesture sequences, virtual gamepad, locomotion and low-level joint control |
| `noetix/bumi` | Noetix Bumi-EDU Humanoid | 15704 | Mic, speaker, locomotion, RealSense camera, state monitoring |
| `x-humanoid/tianyi2.0` | X-humanoid Tianyi 2.0 Pro Bundle | 15707 | 35DOF (wheeled chassis + dual arms + dexterous hands + head + navigation) |
| `deep_robotics/lynx_m20` | DEEPRobotics Lynx M20 | 15716 | Official ROS 2/Fast DDS interfaces and basic_server TCP/UDP native control, with Standard/Pro capability isolation |
| `chasing/qianjiao_p200_pro` | Chasing Qianjiao P200 Pro ROV | 15739 | MAVLink v1/UDP 6DOF motion control, lock/unlock, heartbeat and link status |
| [`limx/tron2`](limx/tron2/README.md) | LimX TRON2 dual-arm robot | 15791 | Head/wrist RGB cameras, joint skeleton with gripper outlines, arm MoveJ and verified saved-home reset |
| `pnpbotics/adam` | PNPbotics Adam Humanoid | 15702 | State, locomotion (gRPC), upper body control, dexterous hands, 3D model |
| `linkerhand/l20_lite` | LinkerHand L20 Lite | 15733 | USB-RS485/Modbus 115200, 10-joint position/speed/torque control and state reads |

## Quick Start

### Prerequisites

- Docker (ARM64)
- A running [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) instance

### Deploy via Web Dashboard (Recommended)

The easiest way to deploy a driver is through the Agent Core Web Dashboard. Navigate to **Deploy** in the top menu — you can browse reviewed and published driver versions, select a version, and deploy with one click. No manual build required.

### Build & Deploy Your Own Driver

If you need to build from source or develop a custom driver:

For G1 image build details, see the [G1 build instructions](unitree/g1/README.md#构建).

```bash
cp .env.example .env  # Fill in registry credentials

# Build a specific driver
./build.sh unitree/g1
./build.sh engineai/t800
```

When run without arguments, `build.sh` shows an interactive multi-select menu to choose which drivers to build. You can also pass driver paths directly for CI usage:

```bash
# Build multiple drivers
./build.sh unitree/g1 phanthy/remote_control
```

Once the driver container starts, it registers itself with Agent Core at `http://<agent-core>:15678/api/mcp`. You can then see the device and its tools in the Web Dashboard.

### Run Locally (without Docker)

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## How It Works

1. Driver starts as an MCP HTTP server on its designated port
2. Driver sends a registration request to Agent Core
3. Agent Core discovers the driver's tools via MCP `initialize` and `tools/list`
4. Tools become available to the LLM agent and appear in the Web Dashboard
5. The LLM agent can invoke tools via MCP `tools/call`
6. A tool that takes a file declares `format: file` + `uploadTo: "mcp"`; the bytes
   reach this container via `POST /file/upload` (`common/file_intake.py`), and the
   reply names the path *inside* the container — no shared mount. See
   [README_dev.md § File Arguments](README_dev.md#file-arguments-format-file-and-post-fileupload).

### G1 Navigation Sensors

The read-only `navigation_sensors` Driver plugin launches an isolated worker
process which subscribes directly to the MID360 raw DDS streams instead of
converting the body `/ubuntu/state/imu` JSON. Keeping raw LiDAR conversion and
IMU forwarding out of the full Driver process prevents the legacy LiDAR,
camera, and MCP threads from starving navigation input callbacks. The worker
publishes two algorithm-independent navigation sensor topics:

- `/ubuntu/navigation/lidar` — `sensor_msgs/msg/PointCloud2`,
  RELIABLE + KEEP_LAST(2), with `x/y/z/intensity/tag/line/timestamp` fields;
- `/ubuntu/navigation/imu` — `sensor_msgs/msg/Imu`, RELIABLE + KEEP_LAST(200);

The MID360 converter accepts only little-endian, tightly packed PointCloud2 rows
(`row_step == width * point_step`). Big-endian clouds, organized clouds with row
padding, and undersized rows are dropped instead of being decoded incorrectly.

For each navigation stream, duplicate or decreasing normalized timestamps are
dropped, never clamped to manufacture a newer measurement. Diagnostics count
these as `cloud_non_increasing_timestamps` / `imu_non_increasing_timestamps`;
subsequent strictly increasing samples can resume normally.

The native MID360 IMU message uses quaternion orientation, angular velocity in
rad/s, linear acceleration in m/s², and the three standard 3×3 covariance
arrays. The existing body `imu` card remains unchanged.

With `navigation_sensors` enabled, `lidar_cloud` exposes the unchanged legacy
cloud first, native PointCloud2 on port `navigation_lidar` second, and the IMU
ROS topic descriptor on port `navigation_imu` third. The cloud descriptor retains
its `associated_imu` metadata with the configured IMU topic, `sensor/imu` format,
ROS type, QoS, frame and timestamp semantics. Consumers can use the IMU port or
this metadata rather than deriving a topic name from the cloud topic. The IMU
normally uses `/ubuntu/navigation/imu`; there is no independent `lidar_imu` card.
The current controlled semantic spatial card connects `navigation_lidar` and
subscribes to the IMU via its `imu_topic` configuration (default
`/ubuntu/navigation/imu`); it does not automatically bind the IMU port or read
`associated_imu`. For custom topics, configure and verify the matching IMU topic.
Both cloud outputs advertise `sensor/pointcloud`, so select `navigation_lidar`,
not the legacy binary cloud, for FAST-LIVO2.

The `lidar_cloud` card owns both navigation streams. Stopping it releases both
requests and the worker once neither stream remains active. Its `info` reports
IMU readiness separately under `outputs.navigation_imu`; the top-level readiness
retains the historical cloud-only contract. Restart discards queued frames from
the old epoch.

LiDAR and IMU each retain their own MID360 sample time, corrected from the
shared source clock into the ROS system-time domain. These are independently
sampled streams, not synchronized pairs; corrected stamps are not bit-for-bit
raw DDS stamps. Samples are dropped while clock offset estimation is not ready
or after an invalid/reset observation; the Driver never invents a source
timestamp. The fixed upside-down mounting rotation is applied equally to cloud
and IMU. The existing `/ubuntu/lidar/cloud` legacy card remains
available for Canvas. Safety uses `/ubuntu/lidar/cloud_internal`, produced from
the same conversion independently of the public card's publishing gate.

Navigation consumers such as LiDAR-inertial mapping and path planning can bind
to these topics without the Driver naming or selecting a specific algorithm.
The body IMU JSON is approximately 20 Hz, has no source timestamp, and is not a
valid substitute for the MID360 built-in IMU. Before accepting a navigation
run, verify the isolated worker delivers approximately 10 Hz LiDAR and 200 Hz
IMU; materially lower rates or repeated source-stamp gaps invalidate the run.

### G1 Self-Describing Camera Frames

The RealSense plugin keeps the legacy `/ubuntu/camera/rgb` compressed image
as the first `camera_rgb` output, with stable port `rgb`; the
`/ubuntu/camera/depth` image and distance topics are unchanged. It additionally
exposes one latest-only, BEST_EFFORT self-describing RGB frame stream, independent
of navigation: `camera_rgb`, port `rgb_frame` → `/ubuntu/camera/rgb_frame`
(`phanthy.sensor.camera_rgb_frame.v1`).

Stopping a camera card gates its own outputs, not its sibling, distance, or
photo/video capture. Capture uses an on-demand internal
JPEG channel. With no active outputs or capture requests, the shared process
pauses hardware capture. A stop timeout is an error, never an idle success.

Migration: replace saved Canvas `navigation_lidar` with `lidar_cloud`'s
`navigation_lidar` port and `camera_rgb_frame` with `camera_rgb`'s `rgb_frame`.
Where supported, replace saved `lidar_imu` or `navigation_imu` card connections
with the `lidar_cloud` card's `navigation_imu` port; the cloud descriptor's
`associated_imu` metadata also remains available. The current controlled semantic
spatial card instead uses its configured `imu_topic` as described above. Consumer
and Canvas support for ports or metadata is required; hardware/ROS acceptance is
pending, and local tests are not a deployment claim.

The RGB stream uses `std_msgs/msg/UInt8MultiArray` to transport a `PSE1` binary
envelope: a fixed little-endian header (`magic`, JSON metadata length, binary
payload length), canonical JSON metadata, then the JPEG payload.
Every frame repeats its active-profile intrinsics, stable `calibration_id`,
RealSense Depth-to-RGB transform, source/Driver-receive timing, and the
configured LiDAR-to-RGB calibration. The `base_to_camera` entry is a
`target_from_source`, row-major 4x4 `T_base_camera` which transforms points
from `camera_color_optical_frame` into `base_link`. It is included in the
`calibration_id`, so consumers do not need to
know the robot model. Invalid, warming-up, reset, or
out-of-order source timestamps are published as explicitly unavailable; the
Driver does not replace them with publish time.

The bundled LiDAR-to-camera and base-to-camera transforms are derived from the
pinned official G1 URDF and are intentionally marked `factory_nominal`. They are
not measured
per-robot calibration and must not be relabeled `validated_on_device` until a
projection overlay and pixel-residual acceptance run has been recorded on that
G1. Missing or invalid calibration is represented as `unavailable`, never as
an identity transform. Camera reconnects rebuild the profile calibration and
therefore update `calibration_id` when serial, resolution, intrinsics, depth
scale, or extrinsics change.

## Writing a New Driver

Want to add support for new hardware? See the **[Driver Development Guide](README_dev.md)** for the full specification, including:

- MCP protocol implementation (JSON-RPC 2.0 methods)
- Tool definition spec (`inputSchema`, `configSchema`, `multiInstance`, `x-action-params`)
- Instance management (`multiInstance` flag, `scope` for config fields)
- Config field formats — masked secrets (`format: "password"`, `x-sensitive`) and a QR code
  for "install the app on your phone" / "point your phone at this robot" (`format: "qr"`)
- Plugin lifecycle (`__init__`, `get_tool`, `start`, `stop`, `dispatch`)
- `driver.yaml` and `config.yaml` metadata format
- Registration and heartbeat with Agent Core
- Port allocation (15700–15799 range)

Quick overview:
- Each driver implements MCP JSON-RPC 2.0 over HTTP (`initialize`, `tools/list`, `tools/call`)
- Tool naming convention: `{device}_{action}` (e.g., `loco_move`, `mic_start`)
- Driver port range: **15700–15799**

### Topic Inference via `info` Action

**All** tools that produce or consume ROS2 topics must implement an `info` action. The Agent Core canvas calls `info(instance_id, input_topic)` immediately after a card is placed or wired, and uses the returned `topic_out`/`topic_in` as the **authoritative** topic path. Static definitions in `tool.topic_out` are used only as a fallback when `info()` is unavailable.

**Rule: driver owns topic path logic; canvas only reads the result.**

| Tool type | `info()` input | `topic_out` computation |
|-----------|---------------|------------------------|
| Static sensor (mic, imu, camera…) | — | Return fixed `self._topic` |
| multiInstance sensor (ext_mic, ext_camera) | `instance_id` | `/{namespace}/{tool}/{instance_id}/…` (replace `-` → `_`) |
| Processor (asr, tts) | `input_topic` | `{input_topic}/{tool_name}` |

Example for a static sensor:
```python
def dispatch(self, action: str, args: dict) -> dict | None:
    if action == "info":
        return {"state": self.state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
    return None
```

> **Note:** ROS2 topic names only allow alphanumerics, `_`, `~`, `{`, `}`. Canvas card IDs
> contain hyphens (e.g. `card-abc123`), so drivers must sanitize `instance_id` before
> embedding it in a topic path: `instance_id.replace('-', '_')`.

The Agent Core canvas calls this endpoint immediately after a card is placed or wired,
so output port labels are populated without waiting for `start`.

---

## Data Rendering in Agent Core Dashboard

The Agent Core Web Dashboard renders live data streams based on the `format` field declared in a tool's `topic_out`. Each format is matched to a specialized renderer:

| Format | Renderer | Description |
|--------|----------|-------------|
| `audio/pcm-16k` | Audio waveform | PCM audio visualizer with playback |
| `video/mjpeg` | Video stream | Motion JPEG video display |
| `image/jpeg` | Image | Static JPEG image |
| `data/json` | Text / KV panel | JSON key-value display |
| `text/*` | Text | Plain text display |
| `sensor/skeleton` | 3D Skeleton | URDF-based 3D skeleton with joint rotation |
| `sensor/pose2d` | 2D human skeleton | COCO-17 human keypoints overlaid on the frame, with a per-person posture and activity label |
| `sensor/lidar*` | Lidar scan | 2D/3D lidar point visualization |
| `sensor/pointcloud` | Point cloud | 3D point cloud renderer |
| `sensor/mapping` | 2D Map | Occupancy grid / SLAM map |
| `state/odom` | Text / KV panel | `motus.odom/1` — what the robot reports about its own motion |
| `control/velocity` | Text / KV panel | `motus.control/1` `twist` — a command stream **into** a driver |

### Skeleton Rendering (`sensor/skeleton`)

For robot state monitoring, declare `"format": "sensor/skeleton"` in `topic_out`. The dashboard will:

1. Call your driver's `model` tool (type: `resource`) to fetch the URDF
2. Parse the URDF kinematic chain in the browser
3. Render a 3D skeleton with joint positions from the URDF
4. Apply real-time joint angles from `sensor/skeleton` topic data

**Requirements for skeleton support:**

- A `model` tool (type `resource`) that returns `{"urdf": "<URDF XML>"}` via MCP
- A `joints` tool (type `sensor`) with `topic_out` format `sensor/skeleton`
- Joint data published as `{"joints": [{"idx": 0, "name": "joint_name", "q": angle}, ...]}`
- **Joint names in data must match URDF joint names exactly** (e.g., `FL_hip_joint` not `FL_hip`)
- **`dispatch()` must return a plain dict** (e.g. `{"urdf": "..."}`) — do NOT return pre-wrapped MCP content arrays (see README_dev.md § "dispatch() Return Value Format")

---

## Being Driven by an Execution Model (`motus.control/1`)

A VLA policy, a navigation stack or a grasp policy produces **tens of commands per
second**, and each one is not a question. MCP `tools/call` is the wrong shape for
that — it is the control plane: low frequency, request/response, authorised. So
those commands go on the **data plane** instead: a `control/*` DDS topic, exactly
as a speaker already takes its audio on `topic_in: audio/pcm-16k` while its
start/stop go through tools.

To make a robot drivable this way, add a **servo card** to its driver. Two exist
to copy from: `realman/rm75_6f_v/servo.py` (single 7-DOF arm) and
`x-humanoid/tianyi2.0/servo.py` (dual arm + hands, using the descriptor's
`groups`).

A servo card is two things:

| | What | Where |
|---|---|---|
| **Descriptor** | what this robot accepts — dof, `joint_names` (the order *is* the meaning of `values`), units, limits, rate, `force_torque` | your card's `info()` |
| **`ControlSink`** | runs every incoming command through freshness, source arbitration, step clamping, hard limits and the watchdog | `common/control/sink.py` |

**Do not write those checks yourself.** The arm's safety properties come from the
sink, which is ROS-free, takes an injected clock, and is tested without a robot.
What belongs in your file is only what is specific to this machine: unit
conversion at the SDK boundary, the vendor's motion-enable gate, and which call
stops it.

Four rules that are easy to get wrong:

- **A URDF is not a descriptor.** It has no units, no control rate, no statement of
  absolute vs incremental, no normalisation range. Reference it via `urdf_ref` for
  FK and collision geometry; do not derive the action interface from it.
- **`force_torque` must be present even as `null`.** `parse_descriptor` rejects a
  descriptor that omits it — omitting it is how a robot ends up assumed to have a
  protection it does not have.
- **Build the descriptor from the same source as your feedback.** If the state you
  publish and the commands you accept come from two hand-written tables, they will
  eventually disagree about what the machine can do.
- **A pause is not a safe state.** When commands stop, the robot holds and then
  resumes *without warning* the moment a valid one lands. `ttl_ms` is the only
  thing keeping it from resuming on a stale command.

Full field spec, the check chain and its failure verdicts: **README_dev.md
§ "Continuous Control (`motus.control/1`)"**. Architecture, how a model is
attached at the other end, and the requirements that will be added to servo cards
as models get more capable (torque/impedance control, jitter bounds, true sensor
timestamps, chunk-level acknowledgement): **`phanthymotus/docs/vla-integration.md`**.

---

## Audio Requirements for ASR Compatibility

Any driver that publishes audio for use with the Perception ASR plugin must meet the following requirements. Failure to comply will result in the ASR receiving audio but producing no output (the VAD silently discards non-conforming frames).

### ROS2 Message Type

```
audio_msgs/AudioChunk
  std_msgs/Header header
  string format          # must be exactly "audio/pcm-16k"
  uint8[] data           # raw PCM bytes
```

### PCM Format

| Parameter | Required value |
|-----------|---------------|
| Encoding | 16-bit signed integer, little-endian (PCM_S16_LE) |
| Sample rate | **16 000 Hz** |
| Channels | **Mono (1 channel)** |
| `format` field | `"audio/pcm-16k"` |

### Chunk Size

| Parameter | Constraint |
|-----------|-----------|
| Minimum | **1 024 bytes** (512 samples ≈ 32 ms) |
| Recommended | 1 024 – 4 096 bytes (32 – 128 ms) |

Chunks smaller than 1 024 bytes are **silently discarded** by the VAD. This is the most common cause of "ASR receives audio but never outputs text."

### The 48 kHz USB Mic Pitfall

Most USB audio interfaces capture at 48 000 Hz natively. After downsampling to 16 000 Hz, a 512-frame ALSA period yields only **170 samples (340 bytes)** — below the minimum. You must accumulate resampled output into a buffer and only publish when 512 samples are ready:

```python
TARGET = 1024  # bytes — 512 int16 samples @ 16 kHz
_buf = bytearray()

# Inside the capture loop, after resampling to 16 kHz:
_buf += resampled_bytes
while len(_buf) >= TARGET:
    chunk, _buf = bytes(_buf[:TARGET]), _buf[TARGET:]
    msg = AudioChunk()
    msg.format = "audio/pcm-16k"
    msg.data = list(chunk)
    publisher.publish(msg)
```

This pattern is already applied to the `ext_mic` plugin in `unitree/g1/ext_devices.py`.

See [perception/README.md](https://github.com/4paradigm/phanthymotus/blob/main/perception/README.md) in the main repository for full VAD tuning options.

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and PR guidelines.

## License

[Apache License 2.0](LICENSE)
