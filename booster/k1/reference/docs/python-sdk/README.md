# BoosterOS Python SDK (`boosteros`) — reference notes

Fetched from https://docs.booster.tech/zh-CN/docs/developer-guide/booster-os-python-sdk/ via
playwright/curl during driver implementation. This is a condensed reference, not a full mirror —
see `reference/sdk/` for the full C++ SDK + ROS2 IDL source, and the live docs site for anything
not covered here.

## SDK overview

- PyPI package: `boosteros` (also `booster_robotics_sdk_python` as its low-level dependency,
  both public on PyPI).
- Supports Booster K1 / T1 / T2. Requires Python >= 3.10 (boosteros==1.1.1) — **note:**
  boosteros>=1.1.3 requires Python >=3.12, incompatible with the ROS humble base image
  (Ubuntu 22.04 / Python 3.10) used by this driver; pinned to 1.1.1 in `requirements.txt`.
- Runs against a real K1/T1/T2 or a Booster Studio virtual robot only — no other environment.
- Firmware >= v1.7 required.

## Install & quick start

```bash
python3 -m pip install boosteros            # base
python3 -m pip install "boosteros[brain]"   # + Speech/Detection deps (matplotlib, ultralytics, onnx...)
```

```python
from boosteros.robots.booster import BoosterRobot

robot = BoosterRobot()
info = robot.robot_info
# RobotInfo(manufacturer='Booster Robotics', model='Booster K1', name='vr1', serial_number='rivt9', ...)

mode = robot.get_mode()          # e.g. 'prepare'
joints = robot.list_joints()     # 22 for K1

joint_states = robot.get_joint_states()
imu = robot.get_imu()
img = robot.get_image(img_type="rgb")
```

## BoosterRobot(...) — initialization

```python
BoosterRobot(
    network_interface: str = "",       # reserved, always ""
    virtual_robot_name: str = "",       # only for multi-virtual-robot mode
    *,
    timeout: float = 5.0,               # init/service-discovery timeout (s)
    callback_workers: int = 4,          # sensor-callback worker threads, >= 1
    enable_tf_listener: bool = True,    # get_transform() needs this
    **kwargs,                           # domain_id: int, dds_profile: str (advanced/rare)
) -> BoosterRobot
```

**Must be a singleton per robot** — each instance opens its own ROS node, subscriptions, and
control publisher; more than one causes command conflicts. Raises `LocoClientInitError` if no
motion service is discovered within `timeout`.

## Info / state retrieval

`robot_info` (property) · `get_mode()` · `list_gaits()` · `get_gait()` · `list_frames()` ·
`list_joints()` · `list_actions()` · `list_sensors()`

## Sensor snapshot + subscription APIs

Snapshot (blocking, one-shot): `get_image(img_type=...)` · `get_camera_info()` · `get_imu()` ·
`get_odom()` · `get_joint_states()` · `get_battery()` · `get_fall_down_state()` ·
`get_transform(...)`

Subscription (push callback, returns a `SensorSubscription` with `.unsubscribe()`):
`subscribe_image(callback, *, img_type=..., queue_size=0, overflow="drop_oldest")` ·
`subscribe_imu(callback, *, imu_id="", queue_size=0, overflow=...)` ·
`subscribe_odom(callback, ...)` · `subscribe_battery(callback, ...)` ·
`subscribe_fall_down_state(callback, ...)`

**No `subscribe_joint_states`** — this driver polls `get_joint_states()` on a 10 Hz ROS2 timer
instead (see `device.py::_K1StateNode._poll_joints`).

### IMUState

`linear_acceleration` [x,y,z] m/s², `angular_velocity` [x,y,z] rad/s, `orientation` [x,y,z,w]
quaternion (**note the order** — CLAUDE.md's `sensor/skeleton` spec wants `imu_quat` as
`[w,x,y,z]`, so `device.py` reorders it), `rpy` [roll,pitch,yaw] rad, `timestamp`, `frame_id`.

### JointState / JointStates

Per-joint: `name`, `position`, `velocity`, `effort`, `acceleration`, `temperature`.
`JointStates` is an ordered collection with `.joints`, `.names`, `.get_joint(name)`,
`__getitem__`, `.to_numpy()`.

### JointCommand (for `set_joints`)

`JointCommand(name: str, position: float, kp: float | None, kd: float | None, ...)` — at least
`name`+`position`; explicit `kp`/`kd` recommended for position control. K1 has 22 joints
(2 neck + 4×2 arm + 6×2 leg); T1 has 23 (extra waist joint inserted at index 10).

## Motion control APIs

- `set_mode(mode: str) -> None` — e.g. `"prepare"`, `"walk"`, `"custom"`.
- `set_gait(gait: str) -> None`
- `set_velocity(vx: float, vy: float, vyaw: float) -> None` — robot-frame planar velocity,
  must be in `"walk"` mode first. Start small (<=0.3 m/s vx for first tests).
- `upper_body_control(enable: bool) -> None` — in `"walk"` mode, hand head+both-arms (first 10
  joints) to the caller while legs keep walking under the system. Differs from `"custom"` mode,
  which also takes over the legs.
- `set_joints(joint_commands: list[JointCommand]) -> None` — batch joint command; the name set
  must be either *all* joints or exactly the first 10 (upper body).
- `set_head_angle(pitch: float, yaw: float) -> None` — rad; pitch positive = down, yaw positive
  = left; out-of-limit does not raise, the robot just stops responding.
- `reset_odom() -> None`

## Advanced task APIs (all async, return `TaskHandle`)

- `get_active_tasks()`
- `do_action(action_id: str, *, on_done=None, on_status_change=None) -> TaskHandle[None]` —
  `action_id` from `list_actions()`.
- `get_up(*, on_done=None, on_status_change=None) -> TaskHandle[None]` — not cancellable.
- `execute_trajectory(trajectory: TrajectoryData, *, on_done=None, on_status_change=None)` —
  `TrajectoryData.load("/path/to/demo.btraj")`, a file produced by the hand-guiding (teach-mode)
  recorder — out of scope for this driver's initial card set (see config.yaml comment), so
  `action.execute_trajectory` here only accepts a path already on the robot's filesystem.
- `play_sound(audio: str | PathLike | AudioData | Iterable[AudioData], *, volume=None, on_done=None, on_status_change=None)`
  — `audio` is a **local path on the robot** (`.wav`/`.mp3`/`.pcm`) or an in-memory `AudioData`;
  there is no over-the-wire byte upload in the SDK itself.

### TaskHandle

`.trace_id` (unique per call — used as the ACP `action_id`), `.task_id`, `.type`, `.group`,
`.status` (`TaskStatus`: RUNNING/SUCCEEDED/FAILED/CANCELLED), `.error`, `.wait(timeout=None)`,
`.cancel() -> bool`, `.done()`, `.running()`, `.task_info()`,
`.add_done_callback(fn)` / `.add_status_change_callback(fn)` — used here to bridge into this
platform's Action Completion Protocol (`device.py::_bind_task_acp`).

## Audio manager (`robot.audio_manager`)

`get_system_volume()` / `set_system_volume(volume: float)` (0.0–1.0, system-wide output, distinct
from `play_sound(..., volume=)`'s per-call volume) · `start_recording()` / `stop_recording()` /
`is_recording()` / `get_recording_duration()` · `record_stream()` / `play_stream()` (real-time
streaming — not exposed as MCP tools here, out of scope for a one-shot JSON-RPC call).

## Vision & speech (`boosteros[brain]`, optional)

`robot.speech`: `recognize_stream(...)`, `chat(text)`, `list_voices()`.
`robot.detection`: `list_models()`, `load_model(name)`, `detect(image)`, `plot(...)`.
Gated behind `config.yaml`'s `plugins.speech.enabled` / `plugins.detection.enabled` (default
`false`) since they require the `[brain]` extra (matplotlib/ultralytics/onnx/onnxruntime).

## Excluded from this driver (see config.yaml comment)

- `robot.hand_guiding_manager` (teach-mode recording) — niche, and `execute_trajectory`'s
  `.btraj` input already depends on it.
- `robot.soccer_kick_manager` (auto soccer-kick task) — RoboCup-specific demo feature.

## Common data types index

Time/Header: `Time`, `Duration`, `Header`.
Sensor/state: `AnyImage` (`Image` | `CompressedImage`), `BoundingBox2D`, `DetectionResult`,
`CameraInfo`, `RegionOfInterest`, `IMUState`, `OdomState`, `Transform`, `BatteryState`,
`FallDownState`, `AudioData`, `ImageType`, `PcmSampleFormat`.
Joints: `JointState`, `JointStates`, `JointCommand`.
Robot metadata: `RobotInfo`, `JointLimits`, `JointInfo`, `ActionInfo`, `RobotModeName`,
`RobotGaitName`, `SensorInfo`, `SensorType`.
Tasks/subscriptions: `TaskHandle`, `TaskInfo`, `TaskStatus`, `OverflowPolicy`,
`SensorSubscription`, `AudioPlaybackStreamHandle`.
Trajectory: `TrajectoryMeta`, `JointTrajectoryPoint`, `TrajectoryData`.
