# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus four higher-level cards backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

## New cards

### `motion_state`

Read-only whole-body motion telemetry from `HighController`.

- `snapshot` with `detail: summary`: current activity, workmode context, protection flag, body orientation, angular velocity, linear acceleration, whole-body joint activity statistics and active motor faults.
- `snapshot` with `detail: joints`: only the position, velocity, torque, temperature and error for each of all 21 joints, plus source/freshness metadata; it does not repeat the summary.
- `history`: recent motion-start/stop, mode, protection, controller-read and motor-fault events. Select `detail: none`; `limit` is optional and defaults to 20.
- `clear_history`: clear the in-memory motion event history. It requires no parameters and ignores unrelated fields supplied by older canvas clients.
- ROS2 output: `/<namespace>/motion/state`, JSON.

The default polling and topic publication rate is 2 Hz (`poll_interval_s: 0.5`).

Every snapshot identifies `Noetix HighController/CycloneDDS` as its source and includes freshness and sample-age fields. It deliberately excludes battery data, which belongs to the existing `battery` card. The SDK does not expose world-frame position or translational velocity, so the card reports only documented IMU and joint measurements and does not invent odometry.

### `arm`

Eight-DOF dual-arm card for Bumi EDU models using `LowController`.

- `get_state`: current position, velocity, torque, temperature and error for both arms.
- `get_limits`: arm limits from `resource/bumi_model.urdf`.
- `move`: asynchronous single-arm or dual-arm position trajectory.
- `status`: current action state.
- `cancel`: stop target interpolation while retaining the last LowController command.

`LowController.set_joint()` always writes all 21 motors. For this reason, arm writes are disabled by default. Read access remains available. Enabling writes requires all of the following values to be verified on the exact physical robot and supplied in `config.yaml`:

```yaml
plugins:
  arm:
    enabled: true
    write_enabled: true
    high_low_arbitration_verified: true
    low_control_recovery_verified: true
    verified_speed_limit_rad_s: REPLACE_WITH_VERIFIED_VALUE
    verified_trajectory_update_hz: REPLACE_WITH_VERIFIED_VALUE
    verified_position_tolerance_rad: REPLACE_WITH_VERIFIED_VALUE
    verified_feedback_timeout_s: REPLACE_WITH_VERIFIED_VALUE
    verified_max_action_duration_s: REPLACE_WITH_VERIFIED_VALUE
    verified_joint_kp: [REPLACE_WITH_21_VERIFIED_VALUES]
    verified_joint_kd: [REPLACE_WITH_21_VERIFIED_VALUES]
```

Do not enable writes by copying Tianyi, another Bumi profile, or example KP/KD values. The takeover and recovery sequence between HighController and LowController must also be confirmed before an actuator test.

### `media_system`

MediaController system management.

- `status`: work status, change reason and last system error.
- `get_config` / `set_config`: timeout, cue, response text, audio routing, video routing and internal 3A selection.
- `get_wake_words`: read the current wake-word string. The SDK does not expose wake-word mutation.
- `wakeup`, `sleep`, `restart`.
- `pause` / `resume`: audio capture, audio playback, video capture, or all paths.

All mutating MediaController calls are serialized with the documented minimum 500 ms interval. `set_config` reads the configuration back before returning `completed`.

On the canvas, enter `set_config.config` as a JSON object, for example
`{"audio_cue":true,"timeout_ms":30000}`. The driver accepts both this JSON text form and a native object supplied by Agent Core. Fields omitted from the object are left unchanged.

### `diagnostics`

Low-risk, read-only checks. It does not move the robot or play test audio.

- `quick_check` / `full_check`: controller and process checks.
- `motion_check`: HighController, workmode, protection, BMS alarm and motor errors.
- `media_check`: MediaController status/error and microphone process.
- `vision_check`: camera process.
- `report`: last report.

Each check is reported as `passed`, `warning`, or `failed`; the aggregate result never treats missing hardware data as success.

## Safe first-device verification

Before any arm write test:

1. Keep `arm.write_enabled: false` and verify `arm.get_state` returns eight correctly named joints with plausible positions and no motor errors.
2. Compare `arm.get_limits` with the vendor URDF installed on the robot.
3. Confirm with Noetix how the robot enters and exits LowController, and how the remaining 13 motors must be commanded while controlling the arms.
4. Verify the full 21-value KP/KD profile for this robot variant.
5. Place the robot on a rated support fixture, clear the arm workspace, keep the physical emergency control reachable, and monitor workmode, motor errors and temperatures.
6. Only then enable writes and command one arm joint through a small, vendor-approved displacement and speed.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
